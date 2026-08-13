#include <AP_HAL/AP_HAL.h>

#include "AP_NavUKF.h"
#include "AP_NavUKF_core.h"
#include <AP_DAL/AP_DAL.h>

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

/*
 * Fuse true airspeed measurements using an unscented transform.
*/
void NavUKF_core::FuseAirspeed()
{
    const ftype VtasPred = norm((stateStruct.velocity.y - stateStruct.wind_vel.y),
                                (stateStruct.velocity.x - stateStruct.wind_vel.x),
                                stateStruct.velocity.z);
    if (VtasPred <= 1.0f) {
        return;
    }

    uint32_t kalman_mask = 0;
    if (tasDataDelayed.allowFusion && !airDataFusionWindOnly) {
        kalman_mask = (1<<10)-1;
    }
    if (tasDataDelayed.allowFusion && !inhibitDelAngBiasStates && !airDataFusionWindOnly) {
        kalman_mask |= (1<<10) | (1<<11) | (1<<12);
    }
    if (tasDataDelayed.allowFusion && !inhibitDelVelBiasStates && !airDataFusionWindOnly) {
        for (uint8_t index = 0; index < 3; index++) {
            if (!dvelBiasAxisInhibit[index]) {
                kalman_mask |= (1<<(index + 13));
            }
        }
    }
    if (tasDataDelayed.allowFusion && !inhibitMagStates && !airDataFusionWindOnly) {
        kalman_mask |= (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21);
    }
    if (tasDataDelayed.allowFusion && !inhibitWindStates && !treatWindStatesAsTruth) {
        kalman_mask |= (1<<22) | (1<<23);
    }

    if (ukfComputeUpdate(tasDataDelayed.tas, tasDataDelayed.tasVariance, UKFObs::TAS,
                         kalman_mask, innovVtas, varInnovVtas)) {
        faultStatus.bad_airspeed = true;
        return;
    }
    faultStatus.bad_airspeed = false;

    tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * (ftype)frontend->_tasInnovGate, 1.0f)) * varInnovVtas);
    const bool isConsistent = (tasTestRatio < 1.0f) || badIMUdata;
    tasTimeout = (imuSampleTime_ms - lastTasPassTime_ms) > frontend->tasRetryTime_ms;
    if (!isConsistent) {
        lastTasFailTime_ms = imuSampleTime_ms;
    } else {
        lastTasFailTime_ms = 0;
    }

    if (tasDataDelayed.allowFusion && (isConsistent || (tasTimeout && posTimeout))) {
        lastTasPassTime_ms = imuSampleTime_ms;
        ukfApplyUpdate(innovVtas, varInnovVtas, true);
    }
}

// select fusion of true airspeed measurements
void NavUKF_core::SelectTasFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data locking out fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !airSpdFusionDelayed) {
        airSpdFusionDelayed = true;
        return;
    } else {
        airSpdFusionDelayed = false;
    }

    // get true airspeed measurement
    readAirSpdData();

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform TAS fusion

    if (tasDataToFuse && statesInitialised && !inhibitWindStates) {
        FuseAirspeed();
        tasDataToFuse = false;
        prevTasStep_ms = imuSampleTime_ms;
    }
}


// select fusion of synthetic sideslip measurements or body frame drag
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// body frame drag only works for bluff body multi rotor vehices with thrust forces aligned with the Z axis
// it requires a stable wind for best results and should not be used for aerobatic flight
void NavUKF_core::SelectBetaDragFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !sideSlipFusionDelayed) {
        sideSlipFusionDelayed = true;
        return;
    } else {
        sideSlipFusionDelayed = false;
    }

    // set true when the fusion time interval has triggered
    bool f_timeTrigger = ((imuSampleTime_ms - prevBetaDragStep_ms) >= frontend->betaAvg_ms);

    // use of air data to constrain drift is necessary if we have limited sensor data or are doing inertial dead reckoning
    bool is_dead_reckoning = ((imuSampleTime_ms - lastGpsPosPassTime_ms) > frontend->deadReckonDeclare_ms) &&
                             ((imuSampleTime_ms - lastVelPassTime_ms) > frontend->deadReckonDeclare_ms);
    const bool noYawSensor = !use_compass() && !using_noncompass_for_yaw();
    const bool f_required = (noYawSensor && (frontend->_betaMask & (1<<1))) || is_dead_reckoning;

    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    const bool f_beta_feasible = (assume_zero_sideslip() && !inhibitWindStates);

    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    if (f_beta_feasible && f_timeTrigger) {
        // unless air data is required to constrain drift, it is only used to update wind state estimates
        if (f_required || (frontend->_betaMask & (1<<0))) {
            // we are required to correct all states
            airDataFusionWindOnly = false;
        } else {
            // we are required to correct only wind states
            airDataFusionWindOnly = true;
        }
        FuseSideslip();
        prevBetaDragStep_ms = imuSampleTime_ms;
    }

#if UKF_FEATURE_DRAG_FUSION
    // fusion of XY body frame aero specific forces is done at a slower rate and only if alternative methods of wind estimation are not available
    if (!inhibitWindStates && storedDrag.recall(dragSampleDelayed,imuDataDelayed.time_ms)) {
        FuseDragForces();
    }
    dragTimeout = (imuSampleTime_ms - lastDragPassTime_ms) > frontend->dragFailTimeLimit_ms;
#endif
}

/*
 * Fuse synthetic sideslip measurement of zero using an unscented transform.
*/
void NavUKF_core::FuseSideslip()
{
    const ftype R_BETA = 0.03f;
    Vector3F vel_rel_wind(stateStruct.velocity.x - stateStruct.wind_vel.x,
                          stateStruct.velocity.y - stateStruct.wind_vel.y,
                          stateStruct.velocity.z);
    vel_rel_wind = prevTnb * vel_rel_wind;
    if (vel_rel_wind.x <= 5.0f) {
        return;
    }

    uint32_t kalman_mask = 0;
    if (!airDataFusionWindOnly) {
        kalman_mask = (1<<10)-1;
    }
    if (!inhibitDelAngBiasStates && !airDataFusionWindOnly) {
        kalman_mask |= (1<<10) | (1<<11) | (1<<12);
    }
    if (!inhibitDelVelBiasStates && !airDataFusionWindOnly) {
        for (uint8_t index = 0; index < 3; index++) {
            if (!dvelBiasAxisInhibit[index]) {
                kalman_mask |= (1<<(index + 13));
            }
        }
    }
    if (!inhibitMagStates && !airDataFusionWindOnly) {
        kalman_mask |= (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21);
    }
    if (!inhibitWindStates && !treatWindStatesAsTruth) {
        kalman_mask |= (1<<22) | (1<<23);
    }

    ftype varInnov;
    if (ukfComputeUpdate(0.0f, R_BETA, UKFObs::Beta, kalman_mask, innovBeta, varInnov)) {
        faultStatus.bad_sideslip = true;
        return;
    }
    faultStatus.bad_sideslip = false;
    innovBeta = constrain_ftype(innovBeta, -0.5f, 0.5f);
    ukfApplyUpdate(innovBeta, varInnov, true);
}

#if UKF_FEATURE_DRAG_FUSION
/*
 * Fuse X and Y body axis drag specific forces using an unscented transform.
*/
void NavUKF_core::FuseDragForces()
{
    const ftype bcoef_x = frontend->_ballisticCoef_x.get();
    const ftype bcoef_y = frontend->_ballisticCoef_y.get();
    const ftype mcoef = frontend->_momentumDragCoef.get();
    ut_obs.using_bcoef_x = bcoef_x > 1.0f;
    ut_obs.using_bcoef_y = bcoef_y > 1.0f;
    ut_obs.using_mcoef = mcoef > 0.001f;
    ut_obs.drag_bcoef_x = bcoef_x;
    ut_obs.drag_bcoef_y = bcoef_y;
    ut_obs.drag_mcoef = mcoef;

    const ftype R_ACC = sq(fmaxF(frontend->_dragObsNoise, 0.5f));
    ut_obs.drag_density_ratio = 1.0f/sq(dal.get_EAS2TAS());
    ut_obs.drag_rho = fmaxF(1.225f * ut_obs.drag_density_ratio, 0.1f);

    const uint32_t kalman_mask = (1u << 22) | (1u << 23);

    for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
        const bool using_bcoef = (axis_index == 0) ? ut_obs.using_bcoef_x : ut_obs.using_bcoef_y;
        if (!ut_obs.using_mcoef && !using_bcoef) {
            if (axis_index == 0) {
                continue;
            }
            return;
        }

        const ftype mea_acc = dragSampleDelayed.accelXY[axis_index] - stateStruct.accel_bias[axis_index] / dtEkfAvg;
        const UKFObs obs = (axis_index == 0) ? UKFObs::DragX : UKFObs::DragY;
        if (ukfComputeUpdate(mea_acc, R_ACC, obs, kalman_mask,
                             innovDrag[axis_index], innovDragVar[axis_index])) {
            return;
        }

        dragTestRatio[axis_index] = sq(innovDrag[axis_index]) / (25.0f * innovDragVar[axis_index]);
        if (dragTestRatio[axis_index] > 1.0f) {
            return;
        }
        ukfApplyUpdate(innovDrag[axis_index], innovDragVar[axis_index], true);
    }

    lastDragPassTime_ms = imuSampleTime_ms;
}
#endif // UKF_FEATURE_DRAG_FUSION

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

