#include <AP_HAL/AP_HAL.h>

#include "AP_NavUKF.h"

#include "AP_NavUKF_core.h"

#if UKF_FEATURE_OPTFLOW_FUSION

#include <GCS_MAVLink/GCS.h>
#include <AP_DAL/AP_DAL.h>

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of optical flow measurements
void NavUKF_core::SelectFlowFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !optFlowFusionDelayed) {
        optFlowFusionDelayed = true;
        return;
    } else {
        optFlowFusionDelayed = false;
    }

    of_elements ofDataDelayed;      // OF data at the fusion time horizon

    // Check for data at the fusion time horizon
    const bool flowDataToFuse = storedOF.recall(ofDataDelayed, imuDataDelayed.time_ms);

    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 1000);
    // check is the terrain offset estimate is still valid - if we are using range finder as the main height reference, the ground is assumed to be at 0
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000) || (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER);
    // Perform tilt check
    bool tiltOK = (prevTnb.c.z > frontend->DCM33FlowMin);
    // Constrain measurements to zero if takeoff is not detected and the height above ground
    // is insufficient to achieve acceptable focus. This allows the vehicle to be picked up
    // and carried to test optical flow operation
    if (!takeOffDetected && ((terrainState - stateStruct.position.z) < 0.5f)) {
        ofDataDelayed.flowRadXYcomp.zero();
        ofDataDelayed.flowRadXY.zero();
        flowDataValid = true;
    }

    // if have valid flow or range measurements, fuse data into a 1-state EKF to estimate terrain height
    if (((flowDataToFuse && (frontend->_flowUse == FLOW_USE_TERRAIN)) || rangeDataToFuse) && tiltOK) {
        // Estimate the terrain offset (runs a one state EKF)
        EstimateTerrainOffset(ofDataDelayed);
    }

#if UKF_FEATURE_OPTFLOW_AGL_KF
    // Update the IMU-aided AGL KF every IMU step when enabled, regardless of flow/RF data presence.
    if (frontend->option_is_enabled(NavUKF::Option::AglKfForOptflow)) {
        UpdateAglKf();
    }
#endif

    // Fuse optical flow data into the main filter
    if (flowDataToFuse && tiltOK) {
        const bool fuse_optflow = (frontend->_flowUse == FLOW_USE_NAV) && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::OPTFLOW, core_index);
        // Set the flow noise used by the fusion processes
        R_LOS = sq(MAX(frontend->_flowNoise, 0.05f));
        // Fuse the optical flow X and Y axis data into the main filter sequentially
        FuseOptFlow(ofDataDelayed, fuse_optflow);
    }
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optical flow rates and range finder measurements
Equations generated using https://github.com/PX4/ecl/tree/master/EKF/matlab/scripts/Terrain%20Estimator
*/
void NavUKF_core::EstimateTerrainOffset(const of_elements &ofDataDelayed)
{
    // horizontal velocity squared
    ftype velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);

    // don't fuse flow data if LOS rate is misaligned, without GPS, or insufficient velocity, as it is poorly observable
    // don't fuse flow data if it exceeds validity limits
    // don't update terrain offset if ground is being used as the zero height datum in the main filter
    bool cantFuseFlowData = ((frontend->_flowUse != FLOW_USE_TERRAIN)
    || !gpsIsInUse
    || PV_AidingMode == AID_RELATIVE 
    || velHorizSq < 25.0f 
    || (MAX(ofDataDelayed.flowRadXY[0],ofDataDelayed.flowRadXY[1]) > frontend->_maxFlowRate));

    if ((!rangeDataToFuse && cantFuseFlowData) || (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)) {
        // skip update
        inhibitGndState = true;
    } else {
        inhibitGndState = false;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation after bad gps causing bad numerical conditioning
        ftype distanceTravelledSq = sq(stateStruct.position[0] - prevPosN) + sq(stateStruct.position[1] - prevPosE);
        distanceTravelledSq = MIN(distanceTravelledSq, 100.0f);
        prevPosN = stateStruct.position[0];
        prevPosE = stateStruct.position[1];

        // in addition to a terrain gradient error model, we also have the growth in uncertainty due to the copter's vertical velocity
        ftype timeLapsed = MIN(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f);
        ftype Pincrement = (distanceTravelledSq * sq(frontend->_terrGradMax)) + sq(timeLapsed)*P[6][6];
        Popt += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (rangeDataToFuse) {
            // reset terrain state if rangefinder data not fused for 5 seconds
            if (imuSampleTime_ms - gndHgtValidTime_ms > 5000) {
                terrainState = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd) + stateStruct.position.z;
            }

            // predict range
            ftype predRngMeas = MAX((terrainState - stateStruct.position[2]),rngOnGnd) / prevTnb.c.z;
            // Copy required states to local variable names
            ftype q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            ftype q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            ftype q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            ftype q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            ftype R_RNG = frontend->_rngNoise.get();

            // calculate Kalman gain
            ftype SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            ftype K_RNG = Popt/(SK_RNG*(R_RNG + Popt/sq(SK_RNG)));

            // Calculate the innovation variance for data logging
            varInnovRng = (R_RNG + Popt/sq(SK_RNG));

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

            // Calculate the measurement innovation
            innovRng = predRngMeas - rangeDataDelayed.rng;

            // calculate the innovation consistency test ratio
            auxRngTestRatio = sq(innovRng) / (sq(MAX(0.01f * (ftype)frontend->_rngInnovGate, 1.0f)) * varInnovRng);

            // Check the innovation test ratio and don't fuse if too large
            if (auxRngTestRatio < 1.0f) {
                // correct the state
                terrainState -= K_RNG * innovRng;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position[2] + rngOnGnd);

                // correct the covariance
                Popt = Popt - sq(Popt)/(SK_RNG*(R_RNG + Popt/sq(SK_RNG))*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                Popt = MAX(Popt,0.0f);

                // record the time we last updated the terrain offset state
                gndHgtValidTime_ms = imuSampleTime_ms;
            }
        }

        if (!cantFuseFlowData) {

            Vector3F relVelSensor;          // velocity of sensor relative to ground in sensor axes
            Vector2F losPred;               // predicted optical flow angular rate measurement
            ftype q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            ftype q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            ftype q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            ftype q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            ftype K_OPT;
            ftype H_OPT;
            Vector2F auxFlowObsInnovVar;

            // predict range to centre of image
            ftype flowRngPred = MAX((terrainState - stateStruct.position.z),rngOnGnd) / prevTnb.c.z;

            // constrain terrain height to be below the vehicle
            terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

            // calculate relative velocity in sensor frame
            relVelSensor = prevTnb*stateStruct.velocity;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred.x =   relVelSensor.y / flowRngPred;
            losPred.y = - relVelSensor.x / flowRngPred;

            // calculate innovations
            auxFlowObsInnov = losPred - ofDataDelayed.flowRadXYcomp;

            // calculate observation jacobians 
            ftype t2 = q0*q0;
            ftype t3 = q1*q1;
            ftype t4 = q2*q2;
            ftype t5 = q3*q3;
            ftype t6 = stateStruct.position.z - terrainState;
            ftype t7 = 1.0f / (t6*t6);
            ftype t8 = q0*q3*2.0f;
            ftype t9 = t2-t3-t4+t5;

            // prevent the state variances from becoming badly conditioned
            Popt = MAX(Popt,1E-6f);

            // calculate observation noise variance from parameter
            ftype flow_noise_variance = sq(MAX(frontend->_flowNoise, 0.05f));

            // Fuse Y axis data

            // Calculate observation partial derivative
            H_OPT = t7*t9*(-stateStruct.velocity.z*(q0*q2*2.0-q1*q3*2.0)+stateStruct.velocity.x*(t2+t3-t4-t5)+stateStruct.velocity.y*(t8+q1*q2*2.0));

            // calculate innovation variance
            auxFlowObsInnovVar.y = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.y;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.y = sq(auxFlowObsInnov.y) / (sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.y);

            // don't fuse if optical flow data is outside valid range
            if (auxFlowTestRatio.y < 1.0f) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov.y;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

                // update intermediate variables used when fusing the X axis
                t6 = stateStruct.position.z - terrainState;
                t7 = 1.0f / (t6*t6);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming badly conditioned
                Popt = MAX(Popt,1E-6f);

                // record the time we last updated the terrain offset state
                gndHgtValidTime_ms = imuSampleTime_ms;
            }

            // fuse X axis data
            H_OPT = -t7*t9*(stateStruct.velocity.z*(q0*q1*2.0+q2*q3*2.0)+stateStruct.velocity.y*(t2-t3+t4-t5)-stateStruct.velocity.x*(t8-q1*q2*2.0));

            // calculate innovation variances
            auxFlowObsInnovVar.x = H_OPT * Popt * H_OPT + flow_noise_variance;

            // calculate Kalman gain
            K_OPT = Popt * H_OPT / auxFlowObsInnovVar.x;

            // calculate the innovation consistency test ratio
            auxFlowTestRatio.x = sq(auxFlowObsInnov.x) / (sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f)) * auxFlowObsInnovVar.x);

            // don't fuse if optical flow data is outside valid range
            if (auxFlowTestRatio.x < 1.0f) {

                // correct the state
                terrainState -= K_OPT * auxFlowObsInnov.x;

                // constrain the state
                terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);

                // correct the covariance
                Popt = Popt - K_OPT * H_OPT * Popt;

                // prevent the state variances from becoming badly conditioned
                Popt = MAX(Popt,1E-6f);
            }
        }
    }
}

/*
 * Fuse angular motion compensated optical flow rates using an unscented transform.
 * Requires a valid terrain height estimate.
 *
 * really_fuse should be true to actually fuse into the main filter, false to only calculate variances
*/
void NavUKF_core::FuseOptFlow(const of_elements &ofDataDelayed, bool really_fuse)
{
    Vector2 losPred;

    ftype pd = stateStruct.position.z;

    // Default is the terrain estimator AGL (terrainState - pd, where pd is the main filter's vertical position)
    // constrain height above ground to be above range measured on ground
    ftype heightAboveGndEst = MAX((terrainState - pd), rngOnGnd);

#if UKF_FEATURE_OPTFLOW_SRTM
    // if ground offset (aka terrainState) is not valid, use SRTM altitude
    terrain_srtm_alt_valid = ((imuSampleTime_ms - terrain_srtm_alt_ms) < 5000);
    if (!gndOffsetValid && terrain_srtm_alt_valid) {
        heightAboveGndEst = MAX((terrain_srtm_alt - pd), rngOnGnd);
    }
#endif

#if UKF_FEATURE_OPTFLOW_AGL_KF
    // AGL KF override: use the IMU-aided AGL KF estimate when enabled and valid,
    // instead of terrainState-pd which can drift when the main filter's vertical position
    // state is unreliable (e.g. poor altitude source, sensor outage, or ground effect).
    if (frontend->option_is_enabled(NavUKF::Option::AglKfForOptflow) && aglKfValid) {
        heightAboveGndEst = MAX(aglKfH, rngOnGnd);
    }
#endif

    // calculate range from ground plain to centre of sensor fov assuming flat earth
    ftype range = constrain_ftype((heightAboveGndEst/prevTnb.c.z),rngOnGnd,1000.0f);

    // correct range for flow sensor offset body frame position offset
    // the corrected value is the predicted range from the sensor focal point to the
    // centre of the image on the ground assuming flat terrain
    Vector3F posOffsetBody = ofDataDelayed.body_offset - accelPosOffset;
    if (!posOffsetBody.is_zero()) {
        Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
        range -= posOffsetEarth.z / prevTnb.c.z;
    }

#if APM_BUILD_TYPE(APM_BUILD_Rover)
    // override with user specified height (if given, for rover)
    if (ofDataDelayed.heightOverride > 0) {
        range = ofDataDelayed.heightOverride;
    }
#endif

    ut_obs.pos_offset_body = posOffsetBody;
    ut_obs.body_rate = ofDataDelayed.bodyRadXYZ;
    ut_obs.range = range;

    const UKFObs flow_obs[2] = { UKFObs::FlowX, UKFObs::FlowY };
    const ftype z_meas[2] = { ofDataDelayed.flowRadXYcomp.x, ofDataDelayed.flowRadXYcomp.y };

    for (uint8_t obsIndex=0; obsIndex<=1; obsIndex++) {
        uint32_t kalman_mask = (1u << 24) - 1;
        if (inhibitDelAngBiasStates) {
            kalman_mask &= ~((1u << 10) | (1u << 11) | (1u << 12));
        }
        if (inhibitDelVelBiasStates || badIMUdata) {
            kalman_mask &= ~((1u << 13) | (1u << 14) | (1u << 15));
        } else {
            for (uint8_t index = 0; index < 3; index++) {
                if (dvelBiasAxisInhibit[index]) {
                    kalman_mask &= ~(1u << (index + 13));
                }
            }
        }
        if (inhibitMagStates) {
            kalman_mask &= ~((1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21));
        }
        if (inhibitWindStates || treatWindStatesAsTruth) {
            kalman_mask &= ~((1u << 22) | (1u << 23));
        }

        if (ukfComputeUpdate(z_meas[obsIndex], R_LOS, flow_obs[obsIndex], kalman_mask,
                             flowInnov[obsIndex], flowVarInnov[obsIndex])) {
            if (obsIndex == 0) {
                faultStatus.bad_xflow = true;
            } else {
                faultStatus.bad_yflow = true;
            }
            losPred[obsIndex] = flowInnov[obsIndex] + z_meas[obsIndex];
            continue;
        }
        if (obsIndex == 0) {
            faultStatus.bad_xflow = false;
        } else {
            faultStatus.bad_yflow = false;
            flowInnovTime_ms = dal.millis();
        }
        losPred[obsIndex] = flowInnov[obsIndex] + z_meas[obsIndex];

        flowTestRatio[obsIndex] = sq(flowInnov[obsIndex]) / (sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f)) * flowVarInnov[obsIndex]);

        if (really_fuse && (flowTestRatio[obsIndex] < 1.0f) && (ofDataDelayed.flowRadXY.x < frontend->_maxFlowRate) && (ofDataDelayed.flowRadXY.y < frontend->_maxFlowRate)) {
            prevFlowFuseTime_ms = imuSampleTime_ms;
            if (!flowFusionActive) {
                flowFusionActive = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u fusing optical flow",(unsigned)imu_index);
            }
            if (ukfApplyUpdate(flowInnov[obsIndex], flowVarInnov[obsIndex])) {
                if (obsIndex == 0) {
                    faultStatus.bad_xflow = true;
                } else {
                    faultStatus.bad_yflow = true;
                }
            }
        }
    }

    // store optical flow rates for use in external calibration
    flowCalSample.timestamp_ms = imuSampleTime_ms;
    flowCalSample.flowRate.x = ofDataDelayed.flowRadXY.x;
    flowCalSample.flowRate.y = ofDataDelayed.flowRadXY.y;
    flowCalSample.bodyRate.x = ofDataDelayed.bodyRadXYZ.x;
    flowCalSample.bodyRate.y = ofDataDelayed.bodyRadXYZ.y;
    flowCalSample.losPred.x = losPred[0];
    flowCalSample.losPred.y = losPred[1];
}

// retrieve latest corrected optical flow samples (used for calibration)
bool NavUKF_core::getOptFlowSample(uint32_t& timestamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const
{
    if (flowCalSample.timestamp_ms != 0) {
        timestamp_ms = flowCalSample.timestamp_ms;
        flowRate = flowCalSample.flowRate;
        bodyRate = flowCalSample.bodyRate;
        losPred = flowCalSample.losPred;
        return true;
    }
    return false;
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

#if UKF_FEATURE_OPTFLOW_AGL_KF
/*
 * 2-state IMU-aided AGL Kalman filter
 *
 * State:       x  = [h_agl (m), v_agl (m/s)]'   (AGL is "up")
 * Transition:  F  = [[1, imuDt], [0, 1]]
 * Input:       u  = -velDotNED.z * imuDt             (gravity-included accel)
 * Noise:       Qvel = sq(UKF_ACC_P_NSE * imuDt)
 *              Qhgt = sq(UKF_TERR_GRAD) * horizDist²
 *
 * Prediction:  x(k+1) = F*x(k) + [0, u]'
 *              P(k+1) = F*P*F' + Q
 *
 * Observation: z = rng * prevTnb.c.z,  H = [1, 0],  R = sq(UKF_RNG_M_NSE)
 * Update:      innov = z - H*x,  innovVar = H*P*H' + R
 *              K = P*H' / innovVar
 *              x += K * innov
 *              P  = (I-KH)*P*(I-KH)' + K*R*K'
 */
void NavUKF_core::UpdateAglKf()
{
    const ftype imuDt = imuDataDelayed.delVelDT;
    const uint32_t aglKfRngTimeout_ms = 5000;   // mark filter invalid / hard-reset after this gap without RF fusion

    // h_agl(k+1) = h_agl(k) + v_agl(k)*imuDt
    aglKfH += aglKfV * imuDt;

    // v_agl(k+1) = v_agl(k) - velDotNED.z*imuDt
    // velDotNED.z is NED-down acceleration (positive = downward, includes gravity).
    // Negate: downward acceleration reduces AGL rate.
    aglKfV -= velDotNED.z * imuDt;

    // First-order decay of v_agl toward zero when RF is absent (tau = 2 s).
    // Without range measurements v_agl is unobservable; accumulated IMU bias
    // error will cause it to drift, pulling h_agl to the floor during
    // subsequent climbs.  The decay limits that drift.
    // At the aglKfRngTimeout_ms validity timeout (5 s), |v| is at most
    // exp(-5/2) ~ 8% of its value at last RF fusion, so the hard reset finds v near zero.
    if (!rangeDataToFuse) {
        const ftype tauV = 2.0f;
        aglKfV *= expf(-imuDt / tauV);
    }

    // AGL cannot go below the on-ground sensor reading
    aglKfH = MAX(aglKfH, rngOnGnd);

    // ----- Covariance prediction: P = F*P*F' + Q -----
    //
    // F = [[1, imuDt],   state-transition matrix
    //      [0,  1]]
    //
    // Process noise Q:
    //
    // Qhgt — terrain-induced AGL uncertainty during forward flight.
    //   As the vehicle moves horizontally by dist over terrain with unknown gradient "g",
    //   the true AGL changes by ~g * dist.  Since "g" is unknown, we treat it as zero-mean
    //   with std-dev terrGradMax, giving variance: Qhgt = terrGradMax² * horizDist²
    //   horizDist² = (vx²+vy²)*imuDt² is the squared horizontal distance travelled this step.
    //   Capped at 1 m² so a single large-velocity step can't blow up the covariance.
    //   The intended effect is that P[0][0] grows quickly during fast horizontal flight over rough terrain,
    //   allowing the RF measurement to pull h_agl back when the next reading arrives.
    //
    // Qvel — unmodelled vertical accelerations (IMU noise, vibration, model error).
    //   Uses the same accNoise parameter as CovariancePrediction (sq(imuDt*accNoise)),
    //   so the velocity uncertainty budget is consistent with the main EKF.
    //   The intended effect is that P[1][1] grows every step when RF is absent, reflecting accumulating IMU integration error in v_agl.
    //
    const ftype horizDistSq = MIN(sq(stateStruct.velocity.x * imuDt)
                                  + sq(stateStruct.velocity.y * imuDt), 1.0f);  // cap at 1 m²
    const ftype Qvel = sq(frontend->_accNoise * imuDt);   // matches CovariancePrediction: sq(imuDt*accNoise)
    const ftype Qhgt = sq(frontend->_terrGradMax) * horizDistSq;

    // Capture before overwrite (P is symmetric, so P[0][1] == P[1][0])
    const ftype P00 = aglKfP[0][0];
    const ftype P01 = aglKfP[0][1];   // == P[1][0]
    const ftype P11 = aglKfP[1][1];

    // Expanded F*P*F' + Q:
    aglKfP[0][0] = P00 + imuDt * (P01 + P01) + sq(imuDt) * P11 + Qhgt;
    aglKfP[0][1] = aglKfP[1][0] = P01 + imuDt * P11;
    aglKfP[1][1] = P11 + Qvel;

    // Cap covariance to prevent runaway during prolonged RF absence
    aglKfP[0][0] = MIN(aglKfP[0][0], 100.0f);  // 10 m std-dev cap
    aglKfP[1][1] = MIN(aglKfP[1][1], 100.0f);  // 10 m/s std-dev cap

    // mark invalid if RF has been absent too long
    if (!rangeDataToFuse) {
        if (imuSampleTime_ms - lastAglRngFuseTime_ms > aglKfRngTimeout_ms) {
            aglKfValid = false;
        }
        return;
    }

    // Only fuse when vehicle tilt is within acceptable limits
    if (prevTnb.c.z < frontend->DCM33FlowMin) {
        return;
    }

    // After the timeout of IMU-only propagation, vertical velocity drift makes the
    // prediction unreliable.  Re-initialise directly from the rangefinder.
    if (imuSampleTime_ms - lastAglRngFuseTime_ms > aglKfRngTimeout_ms) {
        // Tilt-corrected AGL directly from rangefinder reading
        aglKfH = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
        aglKfV = 0.0f;                          // assume stationary on reset
        aglKfP[0][0] = sq(frontend->_rngNoise); // initialise h uncertainty to RF noise
        aglKfP[0][1] = aglKfP[1][0] = 0.0f;
        aglKfP[1][1] = 1.0f;                    // 1 m/s velocity uncertainty after reset
        lastAglRngFuseTime_ms = imuSampleTime_ms;
        aglKfValid = true;
        return;  // skip measurement update this cycle (just used the reading for reset)
    }

    // Measurement update — fuse tilt-corrected rangefinder reading
    //
    // Observation model: z = h_agl,  H = [1, 0]
    //   z_meas = rng * cos(tilt) = rng * prevTnb.c.z
    //
    const ftype hgtMeas = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);

    // Measurement noise variance R (reuse UKF_RNG_M_NSE)
    const ftype measNoiseVar = sq(frontend->_rngNoise);

    // Innovation and innovation covariance
    // hgtInnov = hgtMeas - H*x = hgtMeas - h_agl
    // innovVar = H*P*H' + R    = P[0][0] + R
    const ftype hgtInnov = hgtMeas - aglKfH;
    const ftype innovVar = aglKfP[0][0] + measNoiseVar;

    // reject outliers (RF glitches, specular reflections, etc.)
    // gate is expressed as a multiplier on the 1-sigma bound.
    const ftype innovGate = MAX(0.01f * (ftype)frontend->_rngInnovGate, 1.0f);
    if (sq(hgtInnov) > sq(innovGate) * innovVar) {
        // Innovation too large, likely a glitch.  Inflate both height and velocity
        // uncertainty so the next valid reading can correct both states more aggressively.
        aglKfP[0][0] = MIN(aglKfP[0][0] * 2.0f, 100.0f);
        aglKfP[1][1] = MIN(aglKfP[1][1] * 2.0f, 100.0f);
        return;
    }

    // Kalman gain:  K = P*H' / innovVar = [P[0][0]/innovVar, P[1][0]/innovVar]'
    // (H = [1, 0], so P*H' = first column of P)
    const ftype Kh = aglKfP[0][0] / innovVar;
    const ftype Kv = aglKfP[1][0] / innovVar;

    // State update:  x += K * hgtInnov
    aglKfH += Kh * hgtInnov;
    aglKfV += Kv * hgtInnov;
    aglKfH  = MAX(aglKfH, rngOnGnd);  // enforce physical constraint after update

    // Covariance update P = (I-KH)*P*(I-KH)' + K*R*K'
    // With H = [1, 0], (I-KH) = [[1-Kh, 0], [-Kv, 1]]:
    //   P[0][0] = (1-Kh)²*Phh + Kh²*R
    //   P[0][1] = (1-Kh)*(Phv - Kv*Phh) + Kh*Kv*R
    //   P[1][1] = Pvv - 2*Kv*Phv + Kv²*innovVar
    const ftype oneMinusKh = 1.0f - Kh;
    const ftype Phh = aglKfP[0][0];
    const ftype Phv = aglKfP[0][1];
    const ftype Pvv = aglKfP[1][1];

    aglKfP[0][0] = MAX(sq(oneMinusKh) * Phh + sq(Kh) * measNoiseVar, 0.0f);
    aglKfP[0][1] = aglKfP[1][0] = oneMinusKh * (Phv - Kv * Phh) + Kh * Kv * measNoiseVar;
    aglKfP[1][1] = MAX(Pvv - 2.0f * Kv * Phv + sq(Kv) * innovVar, 0.0f);

    lastAglRngFuseTime_ms = imuSampleTime_ms;
    aglKfValid = true;
}

#endif  // UKF_FEATURE_OPTFLOW_AGL_KF

#endif  //  UKF_FEATURE_OPTFLOW_FUSION
