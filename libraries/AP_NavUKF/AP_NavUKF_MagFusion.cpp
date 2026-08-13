#include <AP_HAL/AP_HAL.h>

#include "AP_NavUKF.h"
#include "AP_NavUKF_core.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_DAL/AP_DAL.h>

// minimum GPS horizontal speed required to use GPS ground course for yaw alignment (m/s)
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
  #define GPS_VEL_YAW_ALIGN_MIN_SPD 5.0F
#else
  #define GPS_VEL_YAW_ALIGN_MIN_SPD 1.0F
#endif

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
void NavUKF_core::controlMagYawReset()
{

    // Vehicles that can use a zero sideslip assumption (Planes) are a special case
    // They can use the GPS velocity to recover from bad initial compass data
    // This allows recovery for heading alignment errors due to compass faults
    if (assume_zero_sideslip() && (!finalInflightYawInit || !yawAlignComplete) && inFlight) {
        gpsYawResetRequest = true;
        return;
    } else {
        gpsYawResetRequest = false;
    }

    // Quaternion and delta rotation vector that are re-used for different calculations
    Vector3F deltaRotVecTemp;
    QuaternionF deltaQuatTemp;

    bool flightResetAllowed = false;
    bool initialResetAllowed = false;
    if (!finalInflightYawInit) {
        // Use a quaternion division to calculate the delta quaternion between the rotation at the current and last time
        deltaQuatTemp = stateStruct.quat / prevQuatMagReset;
        prevQuatMagReset = stateStruct.quat;

        // convert the quaternion to a rotation vector and find its length
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);

        // check if the spin rate is OK - high spin rates can cause angular alignment errors
        bool angRateOK = deltaRotVecTemp.length() < 0.1745f;

        initialResetAllowed = angRateOK && tiltAlignComplete;
        flightResetAllowed = angRateOK && !onGround;

    }

    // reset the limit on the number of magnetic anomaly resets for each takeoff
    if (onGround) {
        magYawAnomallyCount = 0;
    }

    // Check if conditions for a interim or final yaw/mag reset are met
    bool finalResetRequest = false;
    bool interimResetRequest = false;
    if (flightResetAllowed && !assume_zero_sideslip()) {
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
        // for sub, we'd like to be far enough away from metal structures like docks and vessels
        // diving 0.5m is reasonable for both open water and pools
        finalResetRequest = (stateStruct.position.z  - posDownAtTakeoff) > UKF_MAG_FINAL_RESET_ALT_SUB;
#else
        // check that we have reached a height where ground magnetic interference effects are insignificant
        // and can perform a final reset of the yaw and field states
        finalResetRequest = (stateStruct.position.z  - posDownAtTakeoff) < -UKF_MAG_FINAL_RESET_ALT;
#endif

        // check for increasing height
        bool hgtIncreasing = (posDownAtLastMagReset-stateStruct.position.z) > 0.5f;
        ftype yawInnovIncrease = fabsF(innovYaw) - fabsF(yawInnovAtLastMagReset);

        // check for increasing yaw innovations
        bool yawInnovIncreasing = yawInnovIncrease > 0.25f;

        // check that the yaw innovations haven't been caused by a large change in attitude
        deltaQuatTemp = quatAtLastMagReset / stateStruct.quat;
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);
        bool largeAngleChange = deltaRotVecTemp.length() > yawInnovIncrease;

        // if yaw innovations and height have increased and we haven't rotated much
        // then we are climbing away from a ground based magnetic anomaly and need to reset
        interimResetRequest = !finalInflightYawInit
                                && !finalResetRequest
                                && (magYawAnomallyCount < MAG_ANOMALY_RESET_MAX)
                                && hgtIncreasing
                                && yawInnovIncreasing
                                && !largeAngleChange;
    }

    // an initial reset is required if we have not yet aligned the yaw angle
    bool initialResetRequest = initialResetAllowed && !yawAlignComplete;

    // a combined yaw angle and magnetic field reset can be initiated by:
    magYawResetRequest = magYawResetRequest || // an external request
            initialResetRequest || // an initial alignment performed by all vehicle types using magnetometer
            interimResetRequest || // an interim alignment required to recover from ground based magnetic anomaly
            finalResetRequest; // the final reset when we have achieved enough height to be in stable magnetic field environment

    // Perform a reset of magnetic field states and reset yaw to corrected magnetic heading
    if (magYawResetRequest && use_compass()) {
        // send initial alignment status to console
        if (!yawAlignComplete) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u MAG%u initial yaw alignment complete",(unsigned)imu_index, (unsigned)magSelectIndex);
        }

        // set yaw from a single mag sample
        setYawFromMag();

        // send in-flight yaw alignment status to console
        if (finalResetRequest) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u MAG%u in-flight yaw alignment complete",(unsigned)imu_index, (unsigned)magSelectIndex);
        } else if (interimResetRequest) {
            magYawAnomallyCount++;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "UKF IMU%u MAG%u ground mag anomaly, yaw re-aligned",(unsigned)imu_index, (unsigned)magSelectIndex);
        }

        // clear the complete flags if an interim reset has been performed to allow subsequent
        // and final reset to occur
        if (interimResetRequest) {
            finalInflightYawInit = false;
            finalInflightMagInit = false;
        }

        // mag states
        if (!magFieldLearned) {
            resetMagFieldStates();
        }
    }

    if (magStateResetRequest) {
        resetMagFieldStates();
    }
}

// this function is used to do a forced re-alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavUKF_core::realignYawGPS(bool emergency_reset)
{
    // get quaternion from existing filter states and calculate roll, pitch and yaw angles
    Vector3F eulerAngles;
    stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

    if (gpsDataDelayed.vel.xy().length_squared() > sq(GPS_VEL_YAW_ALIGN_MIN_SPD)) {
        // calculate course yaw angle
        ftype velYaw = atan2F(stateStruct.velocity.y,stateStruct.velocity.x);

        // calculate course yaw angle from GPS velocity
        ftype gpsYaw = atan2F(gpsDataDelayed.vel.y,gpsDataDelayed.vel.x);

        // Check the yaw angles for consistency
        ftype yawErr = MAX(fabsF(wrap_PI(gpsYaw - velYaw)),fabsF(wrap_PI(gpsYaw - eulerAngles.z)));

        // If the angles disagree by more than 45 degrees and GPS innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
        bool badMagYaw = ((yawErr > 0.7854f) && (velTestRatio > 1.0f) && (PV_AidingMode == AID_ABSOLUTE)) || !yawAlignComplete;

        // get yaw variance from GPS speed uncertainty
        const ftype gpsVelAcc = fmaxF(gpsSpdAccuracy, ftype(frontend->_gpsHorizVelNoise));
        const ftype gps_yaw_variance = sq(asinF(constrain_float(gpsVelAcc/gpsDataDelayed.vel.xy().length(), -1.0F, 1.0F)));
        if (gps_yaw_variance < sq(radians(GPS_VEL_YAW_ALIGN_MAX_ANG_ERR))) {
            yawAlignGpsValidCount++;
        } else {
            yawAlignGpsValidCount = 0;
        }

        // correct yaw angle using GPS ground course if compass yaw bad
        if (badMagYaw) {
            // attempt to use EKF-GSF estimate if available as it is more robust to GPS glitches
            // by default fly forward vehicles use ground course for initial yaw unless the GSF is explicitly selected as the yaw source
            const bool useGSF = !assume_zero_sideslip() || (yaw_source_last == AP_NavEKF_Source::SourceYaw::GSF);
            if (useGSF && EKFGSF_resetMainFilterYaw(emergency_reset)) {
                return;
            }

            if (yawAlignGpsValidCount >= GPS_VEL_YAW_ALIGN_COUNT_THRESHOLD) {
                yawAlignGpsValidCount = 0;
                // keep roll and pitch and reset yaw
                rotationOrder order;
                bestRotationOrder(order);
                resetQuatStateYawOnly(gpsYaw, gps_yaw_variance, order);

                // reset the velocity and position states as they will be inaccurate due to bad yaw
                ResetVelocity(resetDataSource::GPS);
                ResetPosition(resetDataSource::GPS);

                // send yaw alignment information to console
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u yaw aligned to GPS velocity",(unsigned)imu_index);

                if (use_compass()) {
                    // request a mag field reset which may enable us to use the magnetometer if the previous fault was due to bad initialisation
                    magStateResetRequest = true;
                    // clear the all sensors failed status so that the magnetometers sensors get a second chance now that we are flying
                    allMagSensorsFailed = false;
                }
            }
        } else if (yawAlignGpsValidCount >= GPS_VEL_YAW_ALIGN_COUNT_THRESHOLD) {
                // There is no need to do a yaw reset
                yawAlignGpsValidCount = 0;
                recordYawResetsCompleted();
        }
    } else {
        yawAlignGpsValidCount = 0;
    }
}

// align the yaw angle for the quaternion states to the given yaw angle which should be at the fusion horizon
void NavUKF_core::alignYawAngle(const yaw_elements &yawAngData)
{
    // update quaternion states and covariances
    resetQuatStateYawOnly(yawAngData.yawAng, sq(MAX(yawAngData.yawAngErr, 1.0e-2)), yawAngData.order);

    // send yaw alignment information to console
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u yaw aligned",(unsigned)imu_index);
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
void NavUKF_core::SelectMagFusion()
{
    // clear the flag that lets other processes know that the expensive magnetometer fusion operation has been performed on that time step
    // used for load levelling
    magFusePerformed = false;

    // Store yaw angle when moving for use as a static reference when not moving
    if (!onGroundNotMoving) {
        if (fabsF(prevTnb[0][2]) < fabsF(prevTnb[1][2])) {
            // A 321 rotation order is best conditioned because the X axis is closer to horizontal than the Y axis
            yawAngDataStatic.order = rotationOrder::TAIT_BRYAN_321;
            yawAngDataStatic.yawAng = atan2F(prevTnb[0][1], prevTnb[0][0]);
        } else {
            // A 312 rotation order is best conditioned because the Y axis is closer to horizontal than the X axis
            yawAngDataStatic.order = rotationOrder::TAIT_BRYAN_312;
            yawAngDataStatic.yawAng = atan2F(-prevTnb[1][0], prevTnb[1][1]);
        }
        yawAngDataStatic.yawAngErr = MAX(frontend->_yawNoise, 0.05f);
        yawAngDataStatic.time_ms = imuDataDelayed.time_ms;
    }

    // Handle case where we are not using a yaw sensor of any type and attempt to reset the yaw in
    // flight using the output from the GSF yaw estimator or GPS ground course.
    if ((yaw_source_last == AP_NavEKF_Source::SourceYaw::GSF) ||
        (!use_compass() &&
         yaw_source_last != AP_NavEKF_Source::SourceYaw::GPS &&
         yaw_source_last != AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK &&
         yaw_source_last != AP_NavEKF_Source::SourceYaw::EXTNAV)) {

        if ((!yawAlignComplete || yaw_source_reset) && ((yaw_source_last != AP_NavEKF_Source::SourceYaw::GSF) || (EKFGSF_yaw_valid_count >= GSF_YAW_VALID_HISTORY_THRESHOLD))) {
            realignYawGPS(false);
            yaw_source_reset = false;
        } else {
            yaw_source_reset = false;
        }

        if (imuSampleTime_ms - lastSynthYawTime_ms > 140) {
            // use the EKF-GSF yaw estimator output as this is more robust than the EKF can achieve without a yaw measurement
            // for non fixed wing platform types
            ftype gsfYaw, gsfYawVariance;
            const bool didUseEKFGSF = yawAlignComplete && (yaw_source_last == AP_NavEKF_Source::SourceYaw::GSF) && EKFGSF_getYaw(gsfYaw, gsfYawVariance) && !assume_zero_sideslip() && fuseEulerYaw(yawFusionMethod::GSF);

            // fallback methods
            if (!didUseEKFGSF) {
                if (onGroundNotMoving) {
                    // fuse last known good yaw angle before we stopped moving to allow yaw bias learning when on ground before flight
                    fuseEulerYaw(yawFusionMethod::STATIC);
                } else if (onGround || PV_AidingMode == AID_NONE || (P[0][0]+P[1][1]+P[2][2]+P[3][3] > 0.01f)) {
                    // prevent uncontrolled yaw variance growth that can destabilise the covariance matrix
                    // by fusing a zero innovation
                    fuseEulerYaw(yawFusionMethod::PREDICTED);
                }
            }
            magTestRatio.zero();
            yawTestRatio = 0.0f;
            lastSynthYawTime_ms = imuSampleTime_ms;
        }
        return;
    }

    // Handle case where we are using GPS yaw sensor instead of a magnetomer
    if (yaw_source_last == AP_NavEKF_Source::SourceYaw::GPS || yaw_source_last == AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK) {
        bool have_fused_gps_yaw = false;
        if (storedYawAng.recall(yawAngDataDelayed,imuDataDelayed.time_ms)) {
            if (tiltAlignComplete && (!yawAlignComplete || yaw_source_reset)) {
                alignYawAngle(yawAngDataDelayed);
                yaw_source_reset = false;
                have_fused_gps_yaw = true;
                lastSynthYawTime_ms = imuSampleTime_ms;
                last_gps_yaw_fuse_ms = imuSampleTime_ms;
                recordYawResetsCompleted();
            } else if (tiltAlignComplete && yawAlignComplete) {
                have_fused_gps_yaw = fuseEulerYaw(yawFusionMethod::GPS);
                if (have_fused_gps_yaw) {
                    last_gps_yaw_fuse_ms = imuSampleTime_ms;
                }
            }
            last_gps_yaw_ms = imuSampleTime_ms;
        } else if (tiltAlignComplete && !yawAlignComplete) {
            // External yaw sources can take significant time to start providing yaw data so
            // wuile waiting, fuse a 'fake' yaw observation at 7Hz to keeop the filter stable
            if (imuSampleTime_ms - lastSynthYawTime_ms > 140) {
                yawAngDataDelayed.yawAngErr = MAX(frontend->_yawNoise, 0.05f);
                // update the yaw angle using the last estimate which will be used as a static yaw reference when movement stops
                if (!onGroundNotMoving) {
                    // prevent uncontrolled yaw variance growth by fusing a zero innovation
                    fuseEulerYaw(yawFusionMethod::PREDICTED);
                } else {
                    // fuse last known good yaw angle before we stopped moving to allow yaw bias learning when on ground before flight
                    fuseEulerYaw(yawFusionMethod::STATIC);
                }
                lastSynthYawTime_ms = imuSampleTime_ms;
            }
        } else if (tiltAlignComplete && yawAlignComplete && onGround && imuSampleTime_ms - last_gps_yaw_fuse_ms > 10000) {
            // handle scenario where we were using GPS yaw previously, but the yaw fusion has timed out.
            yaw_source_reset = true;
        }

        if (yaw_source_last == AP_NavEKF_Source::SourceYaw::GPS) {
            // no fallback
            return;
        }

        // get new mag data into delay buffer
        readMagData();

        if (have_fused_gps_yaw) {
            if (gps_yaw_mag_fallback_active) {
                gps_yaw_mag_fallback_active = false;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u yaw external",(unsigned)imu_index);
            }
            // update mag bias from GPS yaw
            gps_yaw_mag_fallback_ok = learnMagBiasFromGPS();
            return;
        }

        // we don't have GPS yaw data and are configured for
        // fallback. If we've only just lost GPS yaw
        if (imuSampleTime_ms - last_gps_yaw_ms < 10000) {
            // don't fallback to magnetometer fusion for 10s
            return;
        }
        if (!gps_yaw_mag_fallback_ok) {
            // mag was not consistent enough with GPS to use it as
            // fallback
            return;
        }
        if (!inFlight) {
            // don't fall back if not flying but reset to GPS yaw if it becomes available
            return;
        }
        if (!gps_yaw_mag_fallback_active) {
            gps_yaw_mag_fallback_active = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u yaw fallback active",(unsigned)imu_index);
        }
        // fall through to magnetometer fusion
    }

#if UKF_FEATURE_EXTERNAL_NAV
    // Handle case where we are using an external nav for yaw
    const bool extNavYawDataToFuse = storedExtNavYawAng.recall(extNavYawAngDataDelayed, imuDataDelayed.time_ms);
    if (yaw_source_last == AP_NavEKF_Source::SourceYaw::EXTNAV) {
        if (extNavYawDataToFuse) {
            if (tiltAlignComplete && (!yawAlignComplete || yaw_source_reset)) {
                alignYawAngle(extNavYawAngDataDelayed);
                yaw_source_reset = false;
            } else if (tiltAlignComplete && yawAlignComplete) {
                fuseEulerYaw(yawFusionMethod::EXTNAV);
            }
            last_extnav_yaw_fusion_ms = imuSampleTime_ms;
        } else if (tiltAlignComplete && !yawAlignComplete) {
            // External yaw sources can take significant time to start providing yaw data so
            // while waiting, fuse a 'fake' yaw observation at 7Hz to keep the filter stable
            if (imuSampleTime_ms - lastSynthYawTime_ms > 140) {
                // update the yaw angle using the last estimate which will be used as a static yaw reference when movement stops
                if (!onGroundNotMoving) {
                    // prevent uncontrolled yaw variance growth by fusing a zero innovation
                    fuseEulerYaw(yawFusionMethod::PREDICTED);
                } else {
                    // fuse last known good yaw angle before we stopped moving to allow yaw bias learning when on ground before flight
                    fuseEulerYaw(yawFusionMethod::STATIC);
                }
                lastSynthYawTime_ms = imuSampleTime_ms;
            }
        }
    }
#endif // UKF_FEATURE_EXTERNAL_NAV

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    if (magHealth) {
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > frontend->magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    if (yaw_source_last != AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK) {
        // check for and read new magnetometer measurements. We don't
        // read for GPS_COMPASS_FALLBACK as it has already been read
        // above
        readMagData();
    }

    // check for availability of magnetometer or other yaw data to fuse
    magDataToFuse = storedMag.recall(magDataDelayed,imuDataDelayed.time_ms);

    // Control reset of yaw and magnetic field states if we are using compass data
    if (magDataToFuse) {
        if (yaw_source_reset && (yaw_source_last == AP_NavEKF_Source::SourceYaw::COMPASS ||
                                 yaw_source_last == AP_NavEKF_Source::SourceYaw::GPS_COMPASS_FALLBACK)) {
            magYawResetRequest = true;
            yaw_source_reset = false;
        }
        controlMagYawReset();
    }

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    bool dataReady = (magDataToFuse && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // use the simple method of declination to maintain heading if we cannot use the magnetic field states
        if(inhibitMagStates || magStateResetRequest || !magStateInitComplete) {
            magFusionSel = MagFuseSel::FUSE_YAW;
            fuseEulerYaw(yawFusionMethod::MAGNETOMETER);

            // zero the test ratio output from the inactive 3-axis magnetometer fusion
            magTestRatio.zero();

        } else {
            magFusionSel = MagFuseSel::FUSE_MAG;
            // if we are not doing aiding with earth relative observations (eg GPS) then the declination is
            // maintained by fusing declination as a synthesised observation
            // We also fuse declination if we are using the WMM tables
            if (PV_AidingMode != AID_ABSOLUTE ||
                (frontend->_mag_ef_limit > 0 && have_table_earth_field)) {
                FuseDeclination(0.34f);
            }
            // fuse the three magnetometer componenents using sequential fusion for each axis
            FuseMagnetometer();
            // zero the test ratio output from the inactive simple magnetometer yaw fusion
            yawTestRatio = 0.0f;
        }
    }

    // If the final yaw reset has been performed and the state variances are sufficiently low
    // record that the earth field has been learned.
    if (!magFieldLearned && finalInflightMagInit) {
        magFieldLearned = (P[16][16] < sq(0.01f)) && (P[17][17] < sq(0.01f)) && (P[18][18] < sq(0.01f));
    }

    // record the last learned field variances
    if (magFieldLearned && !inhibitMagStates) {
        earthMagFieldVar.x = P[16][16];
        earthMagFieldVar.y = P[17][17];
        earthMagFieldVar.z = P[18][18];
        bodyMagFieldVar.x = P[19][19];
        bodyMagFieldVar.y = P[20][20];
        bodyMagFieldVar.z = P[21][21];
    }
}

/*
 * Fuse magnetometer measurements using an unscented transform.
*/
void NavUKF_core::FuseMagnetometer()
{
    // Sequential unscented fusion of magnetometer XYZ. Observation is
    // body-frame field: DCM(q)*earth_mag + body_mag.
    const ftype R_MAG = sq(constrain_ftype(frontend->_magNoise, 0.01f, 0.5f)) + sq(frontend->magVarRateScale*imuDataDelayed.delAng.length() / imuDataDelayed.delAngDT);
    const UKFObs mag_obs[3] = { UKFObs::MagX, UKFObs::MagY, UKFObs::MagZ };
    const ftype mag_meas[3] = { magDataDelayed.mag.x, magDataDelayed.mag.y, magDataDelayed.mag.z };

    uint32_t kalman_mask = (1<<10)-1;
    if (!inhibitDelAngBiasStates) {
        kalman_mask |= (1<<10) | (1<<11) | (1<<12);
    }
    if (!inhibitDelVelBiasStates) {
        for (uint8_t index = 0; index < 3; index++) {
            if (!dvelBiasAxisInhibit[index]) {
                kalman_mask |= (1<<(index + 13));
            }
        }
    }
    if (!inhibitMagStates) {
        kalman_mask |= (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21);
    }
    if (!inhibitWindStates && !treatWindStatesAsTruth) {
        kalman_mask |= (1<<22) | (1<<23);
    }

    for (uint8_t i = 0; i <= 2; i++) {
        if (ukfComputeUpdate(mag_meas[i], R_MAG, mag_obs[i], kalman_mask, innovMag[i], varInnovMag[i])) {
            return;
        }
        magTestRatio[i] = sq(innovMag[i]) / (sq(MAX(0.01f * (ftype)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]);
    }

    magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);
    if (!magHealth) {
        return;
    }

    faultStatus.bad_xmag = false;
    faultStatus.bad_ymag = false;
    faultStatus.bad_zmag = false;

    for (uint8_t obsIndex = 0; obsIndex <= 2; obsIndex++) {
        magFusePerformed = true;
        if (FuseScalarUT(mag_meas[obsIndex], R_MAG, mag_obs[obsIndex], kalman_mask,
                         innovMag[obsIndex], varInnovMag[obsIndex])) {
            forceCovariancePSD(P, sigma_prop, uint8_t(stateIndexLim + 1));
            return;
        }
        if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
            MagTableConstrain();
        }
    }
}

bool NavUKF_core::fuseEulerYaw(yawFusionMethod method)
{
    ftype gsfYaw, gsfYawVariance;
    if (method == yawFusionMethod::GSF) {
        if (!EKFGSF_getYaw(gsfYaw, gsfYawVariance)) {
            return false;
        }
    }

    // yaw measurement error variance (rad^2)
    ftype R_YAW;
    switch (method) {
    case yawFusionMethod::GPS:
        R_YAW = sq(yawAngDataDelayed.yawAngErr);
        break;

    case yawFusionMethod::GSF:
        R_YAW = gsfYawVariance;
        break;

    case yawFusionMethod::STATIC:
        R_YAW = sq(yawAngDataStatic.yawAngErr);
        break;

    case yawFusionMethod::MAGNETOMETER:
    case yawFusionMethod::PREDICTED:
    default:
        R_YAW = sq(frontend->_yawNoise);
        break;

#if UKF_FEATURE_EXTERNAL_NAV
    case yawFusionMethod::EXTNAV:
        R_YAW = sq(MAX(extNavYawAngDataDelayed.yawAngErr, 0.05f));
        break;
#endif
    }

    // determine if a 321 or 312 Euler sequence is best
    rotationOrder order;
    switch (method) {
    case yawFusionMethod::GPS:
        order = yawAngDataDelayed.order;
        break;

    case yawFusionMethod::STATIC:
        order = yawAngDataStatic.order;
        break;

    case yawFusionMethod::MAGNETOMETER:
    case yawFusionMethod::GSF:
    case yawFusionMethod::PREDICTED:
    default:
        // determined automatically
        order = (fabsF(prevTnb[0][2]) < fabsF(prevTnb[1][2])) ? rotationOrder::TAIT_BRYAN_321 : rotationOrder::TAIT_BRYAN_312;
        break;

#if UKF_FEATURE_EXTERNAL_NAV
    case yawFusionMethod::EXTNAV:
        order = extNavYawAngDataDelayed.order;
        break;
#endif
    }

    // Predicted yaw and zero-yaw rotation used to form the yaw measurement
    ftype yawAngPredicted;
    Matrix3F Tbn_zeroYaw;

    if (order == rotationOrder::TAIT_BRYAN_321) {
        Vector3F euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);
        yawAngPredicted = euler321.z;
        Tbn_zeroYaw.from_euler(euler321.x, euler321.y, 0.0f);
    } else if (order == rotationOrder::TAIT_BRYAN_312) {
        Vector3F euler312 = stateStruct.quat.to_vector312();
        yawAngPredicted = euler312.z;
        Tbn_zeroYaw.from_euler312(euler312.x, euler312.y, 0.0f);
    } else {
        return false;
    }

    ftype yawAngMeasured = yawAngPredicted;
    switch (method) {
    case yawFusionMethod::MAGNETOMETER:
    {
        Vector3F magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;
        yawAngMeasured = wrap_PI(-atan2F(magMeasNED.y, magMeasNED.x) + MagDeclination());
        break;
    }

    case yawFusionMethod::GPS:
        yawAngMeasured = yawAngDataDelayed.yawAng;
        break;

    case yawFusionMethod::STATIC:
        yawAngMeasured = yawAngDataStatic.yawAng;
        break;

    case yawFusionMethod::GSF:
        yawAngMeasured = gsfYaw;
        break;

    case yawFusionMethod::PREDICTED:
    default:
        yawAngMeasured = yawAngPredicted;
        break;

#if UKF_FEATURE_EXTERNAL_NAV
    case yawFusionMethod::EXTNAV:
        yawAngMeasured = extNavYawAngDataDelayed.yawAng;
        break;
#endif
    }

    const UKFObs yaw_obs = (order == rotationOrder::TAIT_BRYAN_321) ? UKFObs::Yaw321 : UKFObs::Yaw312;
    const uint32_t kalman_mask = (1u << 24) - 1;
    ftype varInnov;
    if (ukfComputeUpdate(yawAngMeasured, R_YAW, yaw_obs, kalman_mask, innovYaw, varInnov, true)) {
        faultStatus.bad_yaw = true;
        return false;
    }
    faultStatus.bad_yaw = false;

    yawTestRatio = sq(innovYaw) / (sq(MAX(0.01f * (ftype)frontend->_yawInnovGate, 1.0f)) * varInnov);

    if (yawTestRatio > 1.0f) {
        magHealth = false;
        if (inFlight) {
            return false;
        }
    } else {
        magHealth = true;
    }

    const ftype innovFusion = constrain_ftype(innovYaw, -0.5f, 0.5f);
    faultStatus.bad_yaw = ukfApplyUpdate(innovFusion, varInnov);

    return true;
}

/*
 * Fuse declination angle using an unscented transform.
*/
void NavUKF_core::FuseDeclination(ftype declErr)
{
    const ftype R_DECL = sq(declErr);
    const ftype magN = stateStruct.earth_magfield.x;
    const ftype magE = stateStruct.earth_magfield.y;

    if (magN < 1e-3f) {
        return;
    }
    if ((sq(magE) + sq(magN)) < 1e-4f) {
        return;
    }

    uint32_t kalman_mask = (1<<10)-1;
    if (!inhibitDelAngBiasStates) {
        kalman_mask |= (1<<10) | (1<<11) | (1<<12);
    }
    if (!inhibitDelVelBiasStates) {
        for (uint8_t index = 0; index < 3; index++) {
            if (!dvelBiasAxisInhibit[index]) {
                kalman_mask |= (1<<(index + 13));
            }
        }
    }
    if (!inhibitMagStates) {
        kalman_mask |= (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21);
    }
    if (!inhibitWindStates && !treatWindStatesAsTruth) {
        kalman_mask |= (1<<22) | (1<<23);
    }

    const ftype magDecAng = MagDeclination();
    ftype innovation, varInnov;
    if (ukfComputeUpdate(magDecAng, R_DECL, UKFObs::Declination, kalman_mask, innovation, varInnov, true)) {
        return;
    }
    innovation = constrain_ftype(innovation, -0.5f, 0.5f);
    faultStatus.bad_decl = ukfApplyUpdate(innovation, varInnov);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// align the NE earth magnetic field states with the published declination
void NavUKF_core::alignMagStateDeclination()
{
    // don't do this if we already have a learned magnetic field
    if (magFieldLearned) {
        return;
    }

    // get the magnetic declination
    ftype magDecAng = MagDeclination();

    // rotate the NE values so that the declination matches the published value
    Vector3F initMagNED = stateStruct.earth_magfield;
    ftype magLengthNE = initMagNED.xy().length();
    stateStruct.earth_magfield.x = magLengthNE * cosF(magDecAng);
    stateStruct.earth_magfield.y = magLengthNE * sinF(magDecAng);

    if (!inhibitMagStates) {
        // zero the corresponding state covariances if magnetic field state learning is active
        ftype var_16 = P[16][16];
        ftype var_17 = P[17][17];
        zeroStatesVarCov(16, 17);
        P[16][16] = var_16;
        P[17][17] = var_17;

        // fuse the declination angle to establish covariances and prevent large swings in declination
        // during initial fusion
        FuseDeclination(0.1f);

    }
}

// record a magnetic field state reset event
void NavUKF_core::recordMagReset()
{
    magStateResetRequest = false;
    magStateInitComplete = true;
    if (inFlight) {
        finalInflightMagInit = true;
    }
    // take a snap-shot of the vertical position, quaternion  and yaw innovation to use as a reference
    // for post alignment checks
    posDownAtLastMagReset = stateStruct.position.z;
    quatAtLastMagReset = stateStruct.quat;
    yawInnovAtLastMagReset = innovYaw;
}

/*
  learn magnetometer biases from GPS yaw. Return true if the
  resulting mag vector is close enough to the one predicted by GPS
  yaw to use it for fallback
*/
bool NavUKF_core::learnMagBiasFromGPS(void)
{
    if (!have_table_earth_field) {
        // we need the earth field from WMM
        return false;
    }
    if (!inFlight) {
        // don't start learning till we've started flying
        return false;
    }

    mag_elements mag_data;
    if (!storedMag.recall(mag_data, imuDataDelayed.time_ms)) {
        // no mag data to correct
        return false;
    }

    // combine yaw with current quaternion to get yaw corrected quaternion
    QuaternionF quat = stateStruct.quat;
    if (yawAngDataDelayed.order == rotationOrder::TAIT_BRYAN_321) {
        Vector3F euler321;
        quat.to_euler(euler321.x, euler321.y, euler321.z);
        quat.from_euler(euler321.x, euler321.y, yawAngDataDelayed.yawAng);
    } else if (yawAngDataDelayed.order == rotationOrder::TAIT_BRYAN_312) {
        Vector3F euler312 = quat.to_vector312();
        quat.from_vector312(euler312.x, euler312.y, yawAngDataDelayed.yawAng);
    } else {
        // rotation order not supported
        return false;
    }

    // build the expected body field from orientation and table earth field
    Matrix3F dcm;
    quat.rotation_matrix(dcm);
    Vector3F expected_body_field = dcm.transposed() * table_earth_field_ga;

    // calculate error in field
    Vector3F err = (expected_body_field - mag_data.mag) + stateStruct.body_magfield;

    // learn body frame mag biases
    stateStruct.body_magfield -= err * UKF_GPS_MAG_LEARN_RATE;

    // check if error is below threshold. If it is then we can
    // fallback to magnetometer on failure of external yaw
    ftype err_length = err.length();

    // we allow for yaw backback to compass if we have had 50 samples
    // in a row below the threshold. This corresponds to 10 seconds
    // for a 5Hz GPS
    const uint8_t fallback_count_threshold = 50;

    if (err_length > UKF_GPS_MAG_LEARN_LIMIT) {
        gps_yaw_fallback_good_counter = 0;
    } else if (gps_yaw_fallback_good_counter < fallback_count_threshold) {
        gps_yaw_fallback_good_counter++;
    }
    bool ok = gps_yaw_fallback_good_counter >= fallback_count_threshold;
    if (ok) {
        // mark mag healthy to prevent a magTimeout when we start using it
        lastHealthyMagTime_ms = imuSampleTime_ms;
    }
    return ok;
}

// Reset states using yaw from EKF-GSF and velocity and position from GPS
bool NavUKF_core::EKFGSF_resetMainFilterYaw(bool emergency_reset)
{
    // Don't do a reset unless permitted by the UKF_GSF_USE_MASK and UKF_GSF_RUN_MASK parameter masks
    if ((yawEstimator == nullptr)
        || !(frontend->_gsfUseMask & (1U<<core_index))) {
        return false;
    };

    // limit the number of emergency resets
    if (emergency_reset && (EKFGSF_yaw_reset_count >= frontend->_gsfResetMaxCount)) {
        return false;
    }

    ftype yawEKFGSF, yawVarianceEKFGSF;
    if (EKFGSF_getYaw(yawEKFGSF, yawVarianceEKFGSF)) {
        // keep roll and pitch and reset yaw
        rotationOrder order;
        bestRotationOrder(order);
        resetQuatStateYawOnly(yawEKFGSF, yawVarianceEKFGSF, order);

        // record the emergency reset event
        EKFGSF_yaw_reset_request_ms = 0;
        EKFGSF_yaw_reset_ms = imuSampleTime_ms;
        EKFGSF_yaw_reset_count++;

        if ((yaw_source_last == AP_NavEKF_Source::SourceYaw::GSF) ||
            !use_compass() || (dal.compass().get_num_enabled() == 0)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u yaw aligned using GPS",(unsigned)imu_index);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "UKF IMU%u emergency yaw reset",(unsigned)imu_index);
        }

        // Fail the magnetomer so it doesn't get used and pull the yaw away from the correct value
        if (emergency_reset) {
            allMagSensorsFailed = true;
        }

        // record the yaw reset event
        recordYawResetsCompleted();

        // reset velocity and position states to GPS - if yaw is fixed then the filter should start to operate correctly
        ResetVelocity(resetDataSource::DEFAULT);
        ResetPosition(resetDataSource::DEFAULT);

        // reset test ratios that are reported to prevent a race condition with the external state machine requesting the reset
        velTestRatio = 0.0f;
        posTestRatio = 0.0f;

        return true;

    }

    return false;

}

// returns true on success and populates yaw (in radians) and yawVariance (rad^2)
bool NavUKF_core::EKFGSF_getYaw(ftype &yaw, ftype &yawVariance) const
{
    // return immediately if no yaw estimator
    if (yawEstimator == nullptr) {
        return false;
    }

    ftype velInnovLength;
    if (yawEstimator->getYawData(yaw, yawVariance) &&
        is_positive(yawVariance) &&
        yawVariance < sq(radians(GSF_YAW_ACCURACY_THRESHOLD_DEG)) &&
        (assume_zero_sideslip() || (yawEstimator->getVelInnovLength(velInnovLength) && velInnovLength < frontend->maxYawEstVelInnov))) {
        return true;
    }

    return false;
}

void NavUKF_core::resetQuatStateYawOnly(ftype yaw, ftype yawVariance, rotationOrder order)
{
    QuaternionF quatBeforeReset = stateStruct.quat;

    // check if we should use a 321 or 312 Rotation order and update the quaternion
    // states using the preferred yaw definition
    stateStruct.quat.inverse().rotation_matrix(prevTnb);
    Vector3F eulerAngles;
    if (order == rotationOrder::TAIT_BRYAN_321) {
        // rolled more than pitched so use 321 rotation order
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
        stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, yaw);
    } else if (order == rotationOrder::TAIT_BRYAN_312) {
        // pitched more than rolled so use 312 rotation order
        eulerAngles = stateStruct.quat.to_vector312();
        stateStruct.quat.from_vector312(eulerAngles.x, eulerAngles.y, yaw);
    } else {
        // rotation order not supported
        return;
    }

    // Update the rotation matrix
    stateStruct.quat.inverse().rotation_matrix(prevTnb);
    
    // calculate the change in the quaternion state and apply it to the output history buffer
    QuaternionF quat_delta = stateStruct.quat / quatBeforeReset;
    StoreQuatRotate(quat_delta);

    // assume tilt uncertainty split equally between roll and pitch
    Vector3F angleErrVarVec = Vector3F(0.5 * tiltErrorVariance, 0.5 * tiltErrorVariance, yawVariance);
    CovariancePrediction(&angleErrVarVec);

    // record the yaw reset event
    yawResetCount++;

    // record the yaw reset event
    recordYawResetsCompleted();
}
