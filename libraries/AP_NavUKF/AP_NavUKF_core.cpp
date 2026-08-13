#include <AP_HAL/AP_HAL.h>

#include "AP_NavUKF.h"
#include "AP_NavUKF_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_DAL/AP_DAL.h>

// constructor
NavUKF_core::NavUKF_core(NavUKF *_frontend, AP_DAL &_dal) :
    dal(_dal),
    frontend(_frontend),
    public_origin(frontend->common_EKF_origin)
{
    firstInitTime_ms = 0;
    lastInitFailReport_ms = 0;
}

// setup this core backend
bool NavUKF_core::setup_core(uint8_t _imu_index, uint8_t _core_index)
{
    imu_index = _imu_index;
    gyro_index_active = imu_index;
    accel_index_active = imu_index;
    core_index = _core_index;

    /*
      The imu_buffer_length needs to cope with the worst case sensor delay at the
      target EKF state prediction rate. Non-IMU data coming in faster is downsampled.
     */

    // Calculate the expected EKF time step
    if (dal.ins().get_loop_rate_hz() > 0) {
        dtEkfAvg = 1.0f / dal.ins().get_loop_rate_hz();
        dtEkfAvg = MAX(dtEkfAvg,EKF_TARGET_DT);
    } else {
        return false;
    }

    // find the maximum time delay for all potential sensors
    uint16_t maxTimeDelay_ms = MAX(frontend->_hgtDelay_ms ,
            MAX(frontend->_flowDelay_ms ,
                MAX(frontend->_rngBcnDelay_ms ,
                    MAX(frontend->magDelay_ms ,
                        (uint16_t)(EKF_TARGET_DT_MS)
                                  ))));

    // GPS sensing can have large delays and should not be included if disabled
    if (frontend->sources.usingGPS(core_index)) {
        // Wait for the configuration of all GPS units to be confirmed. Until this has occurred the GPS driver cannot provide a correct time delay
        float gps_delay_sec = 0;
        if (!dal.gps().get_lag(selected_gps, gps_delay_sec)) {
#if HAL_GCS_ENABLED
            const uint32_t now = dal.millis();
            if (now - lastInitFailReport_ms > 10000) {
                lastInitFailReport_ms = now;
                // provide an escalating series of messages
                MAV_SEVERITY severity = MAV_SEVERITY_INFO;
                if (now > 30000) {
                    severity = MAV_SEVERITY_ERROR;
                } else if (now > 15000) {
                    severity = MAV_SEVERITY_WARNING;
                }
                GCS_SEND_TEXT(severity, "UKF waiting for GPS config data");
            }
#endif
            return false;
        }
        // limit the time delay value from the GPS library to a max of 250 msec which is the max value the EKF has been tested for.
        maxTimeDelay_ms = MAX(maxTimeDelay_ms , MIN((uint16_t)(gps_delay_sec * 1000.0f),250));
    }

    // airspeed sensing can have large delays and should not be included if disabled
    if (dal.airspeed_sensor_enabled()) {
        maxTimeDelay_ms = MAX(maxTimeDelay_ms , frontend->tasDelay_ms);
    }

#if HAL_VISUALODOM_ENABLED
    // include delay from visual odometry if enabled
    const auto *visual_odom = dal.visualodom();
    if ((visual_odom != nullptr) && visual_odom->enabled()) {
        maxTimeDelay_ms = MAX(maxTimeDelay_ms, MIN(visual_odom->get_delay_ms(), 250));
    }
#endif

    // calculate the IMU buffer length required to accommodate the maximum delay with some allowance for jitter
    imu_buffer_length = (maxTimeDelay_ms / (uint16_t)(EKF_TARGET_DT_MS)) + 1;

    // set the observation buffer length to handle the minimum time of arrival between observations in combination
    // with the worst case delay from current time to ekf fusion time
    // allow for worst case 50% extension of the ekf fusion time horizon delay due to timing jitter
    uint16_t ekf_delay_ms = maxTimeDelay_ms + (int)(ceilF((ftype)maxTimeDelay_ms * 0.5f));
    obs_buffer_length = (ekf_delay_ms / frontend->sensorIntervalMin_ms) + 1;

    // limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
    obs_buffer_length = MIN(obs_buffer_length,imu_buffer_length);

    // calculate buffer size for optical flow data
    const uint8_t flow_buffer_length = MIN((ekf_delay_ms / frontend->flowIntervalMin_ms) + 1, imu_buffer_length);

#if UKF_FEATURE_EXTERNAL_NAV
    // calculate buffer size for external nav data
    const uint8_t extnav_buffer_length = MIN((ekf_delay_ms / frontend->extNavIntervalMin_ms) + 1, imu_buffer_length);
#endif // UKF_FEATURE_EXTERNAL_NAV

    if(!storedGPS.init(obs_buffer_length)) {
        return false;
    }
    if(!storedMag.init(obs_buffer_length)) {
        return false;
    }
    if(!storedBaro.init(obs_buffer_length)) {
        return false;
    }
    if(dal.airspeed() && !storedTAS.init(obs_buffer_length)) {
        return false;
    }
    if(dal.opticalflow_enabled() && !storedOF.init(flow_buffer_length)) {
        return false;
    }
#if UKF_FEATURE_BODY_ODOM
    if(frontend->sources.ext_nav_enabled() && !storedBodyOdm.init(obs_buffer_length)) {
        return false;
    }
    if(frontend->sources.wheel_encoder_enabled() && !storedWheelOdm.init(imu_buffer_length)) {
        // initialise to same length of IMU to allow for multiple wheel sensors
        return false;
    }
#endif // UKF_FEATURE_BODY_ODOM
    if(frontend->sources.gps_yaw_enabled() && !storedYawAng.init(obs_buffer_length)) {
        return false;
    }
#if AP_RANGEFINDER_ENABLED
    // Note: the use of dual range finders potentially doubles the amount of data to be stored
    if(dal.rangefinder() && !storedRange.init(MIN(2*obs_buffer_length , imu_buffer_length))) {
        return false;
    }
#endif
    // Note: range beacon data is read one beacon at a time and can arrive at a high rate
#if UKF_FEATURE_BEACON_FUSION
    if(dal.beacon() && !rngBcn.storedRange.init(imu_buffer_length+1)) {
        return false;
    }
#endif
#if UKF_FEATURE_EXTERNAL_NAV
    if (frontend->sources.ext_nav_enabled() && !storedExtNav.init(extnav_buffer_length)) {
        return false;
    }
    if (frontend->sources.ext_nav_enabled() && !storedExtNavVel.init(extnav_buffer_length)) {
        return false;
    }
    if(frontend->sources.ext_nav_enabled() && !storedExtNavYawAng.init(extnav_buffer_length)) {
        return false;
    }
#endif // UKF_FEATURE_EXTERNAL_NAV
    if(!storedIMU.init(imu_buffer_length)) {
        return false;
    }
    if(!storedOutput.init(imu_buffer_length)) {
        return false;
    }
#if UKF_FEATURE_DRAG_FUSION
    if (!storedDrag.init(obs_buffer_length)) {
        return false;
    }
#endif
 
    if ((yawEstimator == nullptr) && (frontend->_gsfRunMask & (1U<<core_index))) {
        // check if there is enough memory to create the EKF-GSF object
        if (dal.available_memory() < sizeof(EKFGSF_yaw) + 1024) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "UKF IMU%u GSF: not enough memory",(unsigned)imu_index);
            return false;
        }

        // try to instantiate
        yawEstimator = NEW_NOTHROW EKFGSF_yaw();
        if (yawEstimator == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "UKF IMU%uGSF: allocation failed",(unsigned)imu_index);
            return false;
        }
    }

    return true;
}
    

/********************************************************
*                   INIT FUNCTIONS                      *
********************************************************/

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavUKF_core::InitialiseVariables()
{
    // calculate the nominal filter update rate
    const auto &ins = dal.ins();
    localFilterTimeStep_ms = (uint8_t)(1000*ins.get_loop_delta_t());
    localFilterTimeStep_ms = MAX(localFilterTimeStep_ms, (uint8_t)EKF_TARGET_DT_MS);

    // initialise time stamps
    imuSampleTime_ms = frontend->imuSampleTime_us / 1000;
    prevTasStep_ms = imuSampleTime_ms;
    prevBetaDragStep_ms = imuSampleTime_ms;
    lastBaroReceived_ms = imuSampleTime_ms;
    lastVelPassTime_ms = 0;
    lastGpsPosPassTime_ms = 0;
    lastHgtPassTime_ms = 0;
    lastTasPassTime_ms = 0;
    lastSynthYawTime_ms = 0;
    lastTimeGpsReceived_ms = 0;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    rngValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowFuseTime_ms = 0;
    gndHgtValidTime_ms = 0;
    ekfStartTime_ms = imuSampleTime_ms;
    lastGpsVelFail_ms = 0;
    lastGpsAidBadTime_ms = 0;
    timeTasReceived_ms = 0;
    lastPreAlignGpsCheckTime_ms = imuSampleTime_ms;
    posNEResetCount = 0;
    posDResetCount = 0;
    lastRngMeasTime_ms = 0;

    // initialise other variables
    memset(&dvelBiasAxisInhibit, 0, sizeof(dvelBiasAxisInhibit));
	dvelBiasAxisVarPrev.zero();
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    tasTimeout = true;
    dragTimeout = true;
    badIMUdata = false;
    badIMUdata_ms = 0;
    goodIMUdata_ms = 0;
    vertVelVarClipCounter = 0;
    finalInflightYawInit = false;
    dtIMUavg = ins.get_loop_delta_t();
    dtEkfAvg = EKF_TARGET_DT;
    dt = 0;
    velDotNEDfilt.zero();
    lastKnownPositionNE.zero();
    lastKnownPositionD = 0;
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&KHP[0][0], 0, sizeof(KHP));
    flowDataValid = false;
    rangeDataToFuse  = false;
#if UKF_FEATURE_OPTFLOW_FUSION
    Popt = 0.0f;
#endif
    terrainState = 0.0f;
    prevPosN = stateStruct.position.x;
    prevPosE = stateStruct.position.y;
    inhibitGndState = false;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    PV_AidingMode = AID_NONE;
    PV_AidingModePrev = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    velAiding = false;
    waitingForGpsChecks = false;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    onGround = true;
    prevOnGround = true;
    inFlight = false;
    prevInFlight = false;
    manoeuvring = false;
    fusingStationaryZeroVel = false;
    inhibitWindStates = true;
    windStateIsObservable = false;
    treatWindStatesAsTruth = false;
    lastAspdEstIsValid = false;
    windStatesAligned = false;
    inhibitDelVelBiasStates = true;
    inhibitDelAngBiasStates = true;
    gndOffsetValid =  false;
    validOrigin = false;
    gpsSpdAccuracy = 0.0f;
    gpsPosAccuracy = 0.0f;
    gpsHgtAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    rngOnGnd = 0.05f;
#if UKF_FEATURE_OPTFLOW_AGL_KF
    // 2-state AGL KF initialisation
    // Start with generous uncertainty; the first valid RF measurement will hard-reset the state
    aglKfH = rngOnGnd;      // assume sitting on ground at minimum range
    aglKfV = 0.0f;
    aglKfP[0][0] = 25.0f;   // 5 m initial std-dev in height
    aglKfP[0][1] = 0.0f;
    aglKfP[1][0] = 0.0f;
    aglKfP[1][1] = 1.0f;    // 1 m/s initial std-dev in velocity
    aglKfValid = false;
    lastAglRngFuseTime_ms = 0;
#endif
    yawResetCount = 0;
    tiltErrorVariance = sq(M_2PI);
    tiltAlignComplete = false;
    yawAlignComplete = false;
    yawAlignGpsValidCount = 0;
    have_table_earth_field = false;
    stateIndexLim = 23;
    last_gps_idx = 0;
    delAngCorrection.zero();
    velErrintegral.zero();
    posErrintegral.zero();
    gpsGoodToAlign = false;
    gpsIsInUse = false;
    motorsArmed = false;
    prevMotorsArmed = false;
    memset(&gpsCheckStatus, 0, sizeof(gpsCheckStatus));
    gpsSpdAccPass = false;
    ekfInnovationsPass = false;
    sAccFilterState1 = 0.0f;
    sAccFilterState2 = 0.0f;
    lastGpsCheckTime_ms = 0;
    lastGpsInnovPassTime_ms = 0;
    lastGpsInnovFailTime_ms = 0;
    lastGpsVertAccPassTime_ms = 0;
    lastGpsVertAccFailTime_ms = 0;
    gpsAccuracyGood = false;
    gpsAccuracyGoodForAltitude = false;
    gpsloc_prev = {};
    gpsDriftNE = 0.0f;
    gpsVertVelFilt = 0.0f;
    gpsHorizVelFilt = 0.0f;
    ZERO_FARRAY(statesArray);
    memset(&vertCompFiltState, 0, sizeof(vertCompFiltState));
    posVelFusionDelayed = false;
#if UKF_FEATURE_OPTFLOW_FUSION
    optFlowFusionDelayed = false;
#endif
    flowFusionActive = false;
    airSpdFusionDelayed = false;
    sideSlipFusionDelayed = false;
    airDataFusionWindOnly = false;
    posResetNE.zero();
    posResetD = 0.0f;
    hgtInnovFiltState = 0.0f;
    imuDataDownSampledNew.delAng.zero();
    imuDataDownSampledNew.delVel.zero();
    imuDataDownSampledNew.delAngDT = 0.0f;
    imuDataDownSampledNew.delVelDT = 0.0f;
    imuDataDownSampledNew.gyro_index = gyro_index_active;
    imuDataDownSampledNew.accel_index = accel_index_active;
    runUpdates = false;
    framesSincePredict = 0;
    gpsYawResetRequest = false;
    delAngBiasLearned = false;
    memset(&filterStatus, 0, sizeof(filterStatus));
    activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
    prevHgtSource = activeHgtSource;
#if UKF_FEATURE_RANGEFINDER_MEASUREMENTS
    memset(&rngMeasIndex, 0, sizeof(rngMeasIndex));
    memset(&storedRngMeasTime_ms, 0, sizeof(storedRngMeasTime_ms));
    memset(&storedRngMeas, 0, sizeof(storedRngMeas));
#endif
    terrainHgtStable = true;
    ekfOriginHgtVar = 0.0f;
    ekfGpsRefHgt = 0.0;
    velOffsetNED.zero();
    posOffsetNED.zero();
    ZERO_FARRAY(velPosObs);

    // range beacon fusion variables
#if UKF_FEATURE_BEACON_FUSION
    rngBcn.InitialiseVariables();
#endif  // UKF_FEATURE_BEACON_FUSION

#if UKF_FEATURE_BODY_ODOM
    // body frame displacement fusion
    memset((void *)&bodyOdmDataNew, 0, sizeof(bodyOdmDataNew));
    memset((void *)&bodyOdmDataDelayed, 0, sizeof(bodyOdmDataDelayed));
#endif
    lastbodyVelPassTime_ms = 0;
    ZERO_FARRAY(bodyVelTestRatio);
    ZERO_FARRAY(varInnovBodyVel);
    ZERO_FARRAY(innovBodyVel);
    prevBodyVelFuseTime_ms = 0;
    bodyOdmMeasTime_ms = 0;
    bodyVelFusionDelayed = false;
    bodyVelFusionActive = false;

    // yaw sensor fusion
    yawMeasTime_ms = 0;
    memset(&yawAngDataNew, 0, sizeof(yawAngDataNew));
    memset(&yawAngDataDelayed, 0, sizeof(yawAngDataDelayed));

#if UKF_FEATURE_EXTERNAL_NAV
    // external nav data fusion
    extNavDataDelayed = {};
    extNavMeasTime_ms = 0;
    extNavLastPosResetTime_ms = 0;
    extNavDataToFuse = false;
    extNavUsedForPos = false;
    extNavVelDelayed = {};
    extNavVelToFuse = false;
    useExtNavVel = false;
    extNavVelMeasTime_ms = 0;
#endif

    // zero data buffers
    storedIMU.reset();
    storedGPS.reset();
    storedBaro.reset();
    storedTAS.reset();
#if UKF_FEATURE_RANGEFINDER_MEASUREMENTS
    storedRange.reset();
#endif
    storedOutput.reset();
#if UKF_FEATURE_BEACON_FUSION
    rngBcn.storedRange.reset();
#endif
#if UKF_FEATURE_BODY_ODOM
    storedBodyOdm.reset();
    storedWheelOdm.reset();
#endif
#if UKF_FEATURE_EXTERNAL_NAV
    storedExtNav.reset();
    storedExtNavVel.reset();
#endif

    // initialise pre-arm message
    dal.snprintf(prearm_fail_string, sizeof(prearm_fail_string), "UKF still initialising");

    InitialiseVariablesMag();

    // emergency reset of yaw to EKFGSF estimate
    EKFGSF_yaw_reset_ms = 0;
    EKFGSF_yaw_reset_request_ms = 0;
    EKFGSF_yaw_reset_count = 0;
    EKFGSF_run_filterbank = false;
    EKFGSF_yaw_valid_count = 0;

    effectiveMagCal = effective_magCal();
}

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavUKF_core::InitialiseVariablesMag()
{
    lastHealthyMagTime_ms = imuSampleTime_ms;
    lastMagUpdate_us = 0;
    magYawResetTimer_ms = imuSampleTime_ms;
    magTimeout = false;
    allMagSensorsFailed = false;
    finalInflightMagInit = false;
    inhibitMagStates = true;
    magSelectIndex = dal.compass().get_first_usable();
    lastMagOffsetsValid = false;
    magStateResetRequest = false;
    magStateInitComplete = false;
    magYawResetRequest = false;
    posDownAtLastMagReset = stateStruct.position.z;
    yawInnovAtLastMagReset = 0.0f;
    stateStruct.quat.initialise();
    quatAtLastMagReset = stateStruct.quat;
    magFieldLearned = false;
    storedMag.reset();
    storedYawAng.reset();
#if UKF_FEATURE_EXTERNAL_NAV
    storedExtNavYawAng.reset();
#endif
    needMagBodyVarReset = false;
    needEarthBodyVarReset = false;
    magFusionSel = MagFuseSel::NOT_FUSING;
}

/*
Initialise the states from accelerometer data. This assumes measured acceleration 
is dominated by gravity. If this assumption is not true then the EKF will require
timee to reduce the resulting tilt error. Yaw alignment is not performed by this
function, but is perfomred later and initiated the SelectMagFusion() function
after the tilt has stabilised.
*/

bool NavUKF_core::InitialiseFilterBootstrap(void)
{
    // update sensor selection (for affinity)
    update_sensor_selection();

    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && dal.gps().status(preferred_gps) < AP_GPS_FixType::FIX_3D) {
        dal.snprintf(prearm_fail_string,
                     sizeof(prearm_fail_string),
                     "UKF init failure: No GPS lock");
        statesInitialised = false;
        return false;
    }

    // read all the sensors required to start the EKF the states
    readIMUData(false);  // don't allow prediction
    readMagData();
    readGpsData();
    readGpsYawData();
    readBaroData();

    if (statesInitialised) {
        // we are initialised, but we don't return true until the IMU
        // buffer has been filled. This prevents a timing
        // vulnerability with a pause in IMU data during filter startup
        return storedIMU.is_filled();
    }

    // accumulate enough sensor data to fill the buffers
    if (firstInitTime_ms == 0) {
        firstInitTime_ms = imuSampleTime_ms;
        return false;
    } else if (imuSampleTime_ms - firstInitTime_ms < 1000) {
        return false;
    }

    // set re-used variables to zero
    InitialiseVariables();

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3F initAccVec;

    // TODO we should average accel readings over several cycles
    initAccVec = dal.ins().get_accel(accel_index_active).toftype();

    // normalise the acceleration vector
    ftype pitch=0, roll=0;
    if (initAccVec.length() > 0.001f) {
        initAccVec.normalize();

        // calculate initial pitch angle
        pitch = asinF(initAccVec.x);

        // calculate initial roll angle
        roll = atan2F(-initAccVec.y , -initAccVec.z);
    }

    // calculate initial roll and pitch orientation
    stateStruct.quat.from_euler(roll, pitch, 0.0f);

    // initialise dynamic states
    stateStruct.velocity.zero();
    stateStruct.position.zero();

    // initialise static process model states
    stateStruct.gyro_bias.zero();
    stateStruct.accel_bias.zero();
    stateStruct.wind_vel.zero();
    stateStruct.earth_magfield.zero();
    stateStruct.body_magfield.zero();

    // set the position, velocity and height
    ResetVelocity(resetDataSource::DEFAULT);
    ResetPosition(resetDataSource::DEFAULT);
    ResetHeight();

    // initialise sources
    posxy_source_last = frontend->sources.getPosXYSource(core_index);
    yaw_source_last = frontend->sources.getYawSource(core_index);

    // define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, dal.get_home().lat);

    // initialise the covariance matrix
    CovarianceInit();

    // reset the output predictor states
    StoreOutputReset();

    // set to true now that states have be initialised
    statesInitialised = true;

    // reset inactive biases
    for (uint8_t i=0; i<INS_MAX_INSTANCES; i++) {
        inactiveBias[i].gyro_bias.zero();
        inactiveBias[i].accel_bias.zero();
    }

    // restore the navigation origin from the public origin if possible:
    if (public_origin.initialised()) {
        setOriginLLH(public_origin);
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u initialised",(unsigned)imu_index);

    // we initially return false to wait for the IMU buffer to fill
    return false;
}

// initialise the covariance matrix
void NavUKF_core::CovarianceInit()
{
    // zero the matrix
    memset(&P[0][0], 0, sizeof(P));

    // define the initial angle uncertainty as variances for a rotation vector
    Vector3F rot_vec_var;
    rot_vec_var.x = rot_vec_var.y = rot_vec_var.z = sq(0.1f);

    // reset the quaternion state covariances
    CovariancePrediction(&rot_vec_var);

    // velocities
    P[4][4]   = sq(frontend->_gpsHorizVelNoise);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(frontend->_gpsVertVelNoise);
    // positions
    P[7][7]   = sq(frontend->_gpsHorizPosNoise);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(frontend->_baroAltNoise);
    // gyro delta angle biases
    P[10][10] = sq(radians(InitialGyroBiasUncertainty() * dtEkfAvg));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // delta velocity biases
    P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * frontend->_accBiasLim * dtEkfAvg);
    P[14][14] = P[13][13];
    P[15][15] = P[13][13];
    // earth magnetic field
    P[16][16] = sq(frontend->_magNoise);
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = sq(frontend->_magNoise);
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
    // wind velocities
    P[22][22] = 0.0f;
    P[23][23]  = P[22][22];


#if UKF_FEATURE_OPTFLOW_FUSION
    // optical flow ground height covariance
    Popt = 0.25f;
#endif

}

/********************************************************
*                 UPDATE FUNCTIONS                      *
********************************************************/
// Update Filter States - this should be called whenever new IMU data is available
void NavUKF_core::UpdateFilter(bool predict)
{
    // don't run filter updates if states have not been initialised
    if (!statesInitialised) {
        return;
    }

    fill_scratch_variables();

    // update sensor selection (for affinity)
    update_sensor_selection();

    // TODO - in-flight restart method

    // Check arm status and perform required checks and mode changes
    controlFilterModes();

    // read IMU data as delta angles and velocities
    readIMUData(predict);

    // Run the EKF equations to estimate at the fusion time horizon if new IMU data is available in the buffer
    if (runUpdates) {
        // Snapshot state/Tnb before strapdown for unscented covariance prediction
        for (uint8_t i = 0; i <= stateIndexLim; i++) {
            stateBeforePredict[i] = statesArray[i];
        }
        prevTnbBeforePredict = prevTnb;

        // Predict states using IMU data from the delayed time horizon
        UpdateStrapdownEquationsNED();

        // Predict the covariance growth via unscented transform
        CovariancePrediction(nullptr);

        // Run the IMU prediction step for the GSF yaw estimator algorithm
        // using IMU and optionally true airspeed data.
        // Must be run before SelectMagFusion() to provide an up to date yaw estimate
        runYawEstimatorPrediction();

        // Update states using  magnetometer or external yaw sensor data
        SelectMagFusion();

        // Update states using GPS and altimeter data
        SelectVelPosFusion();

        // Run the GPS velocity correction step for the GSF yaw estimator algorithm
        // and use the yaw estimate to reset the main EKF yaw if requested
        // Muat be run after SelectVelPosFusion() so that fresh GPS data is available
        runYawEstimatorCorrection();

#if UKF_FEATURE_BEACON_FUSION
        // Update states using range beacon data
        SelectRngBcnFusion();
#endif

#if UKF_FEATURE_OPTFLOW_FUSION
        // Update states using optical flow data
        SelectFlowFusion();
#endif

#if UKF_FEATURE_BODY_ODOM
        // Update states using body frame odometry data
        SelectBodyOdomFusion();
#endif

        // Update states using airspeed data
        SelectTasFusion();

        // Update states using sideslip constraint assumption for fly-forward vehicles or body drag for multicopters
        SelectBetaDragFusion();

        // Update the filter status
        updateFilterStatus();

        if (imuSampleTime_ms - last_oneHz_ms >= 1000) {
            // 1Hz tasks
            last_oneHz_ms = imuSampleTime_ms;
            moveEKFOrigin();
            checkUpdateEarthField();
        }
    }

    // Wind output forward from the fusion to output time horizon
    calcOutputStates();

    /*
      this is a check to cope with a vehicle sitting idle on the
      ground and getting over-confident of the state. The symptoms
      would be "gyros still settling" when the user tries to arm. In
      that state the EKF can't recover, so we do a hard reset and let
      it try again.
     */
    if (filterStatus.value != 0) {
        last_filter_ok_ms = dal.millis();
    }
    if (filterStatus.value == 0 &&
        last_filter_ok_ms != 0 &&
        dal.millis() - last_filter_ok_ms > 5000 &&
        !dal.get_armed()) {
        // we've been unhealthy for 5 seconds after being healthy, reset the filter
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "UKF IMU%u forced reset",(unsigned)imu_index);
        last_filter_ok_ms = 0;
        statesInitialised = false;
        InitialiseFilterBootstrap();
    }
}

void NavUKF_core::correctDeltaAngle(Vector3F &delAng, ftype delAngDT, uint8_t gyro_index)
{
    delAng -= inactiveBias[gyro_index].gyro_bias * (delAngDT / dtEkfAvg);
}

void NavUKF_core::correctDeltaVelocity(Vector3F &delVel, ftype delVelDT, uint8_t accel_index)
{
    delVel -= inactiveBias[accel_index].accel_bias * (delVelDT / dtEkfAvg);
}

/*
 * Update the quaternion, velocity and position states using delayed IMU measurements
 * because the EKF is running on a delayed time horizon. Note that the quaternion is
 * not used by the EKF equations, which instead estimate the error in the attitude of
 * the vehicle when each observation is fused. This attitude error is then used to correct
 * the quaternion.
*/
void NavUKF_core::UpdateStrapdownEquationsNED()
{
    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    // apply correction for earth's rotation rate
    // % * - and + operators have been overloaded
    stateStruct.quat.rotate(delAngCorrected - prevTnb * earthRateNED*imuDataDelayed.delAngDT);

    stateStruct.quat.normalize();

    // transform body delta velocities to delta velocities in the nav frame
    // use the nav frame from previous time step as the delta velocities
    // have been rotated into that frame
    // * and + operators have been overloaded
    Vector3F delVelNav;  // delta velocity vector in earth axes
    delVelNav  = prevTnb.mul_transpose(delVelCorrected);
    delVelNav.z += GRAVITY_MSS*imuDataDelayed.delVelDT;

    // calculate the nav to body cosine matrix
    stateStruct.quat.inverse().rotation_matrix(prevTnb);

    // calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / imuDataDelayed.delVelDT;

    // apply a first order lowpass filter
    velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = velDotNEDfilt.length();
    accNavMagHoriz = velDotNEDfilt.xy().length();

    // if we are not aiding, then limit the horizontal magnitude of acceleration
    // to prevent large manoeuvre transients disturbing the attitude
    if ((PV_AidingMode == AID_NONE) && (accNavMagHoriz > 5.0f)) {
        ftype gain = 5.0f/accNavMagHoriz;
        delVelNav.x *= gain;
        delVelNav.y *= gain;
    }

    // save velocity for use in trapezoidal integration for position calcuation
    Vector3F lastVelocity = stateStruct.velocity;

    // sum delta velocities to get velocity
    stateStruct.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);

    // accumulate the bias delta angle and time since last reset by an OF measurement arrival
    delAngBodyOF += delAngCorrected;
    delTimeOF += imuDataDelayed.delAngDT;

    // limit states to protect against divergence
    ConstrainStates();

#if UKF_FEATURE_BEACON_FUSION
    // If main filter velocity states are valid, update the range beacon receiver position states
    if (filterStatus.flags.horiz_vel) {
        rngBcn.receiverPos += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);
    }
#endif
}

/*
 * Propagate PVA solution forward from the fusion time horizon to the current time horizon
 * using simple observer which performs two functions:
 * 1) Corrects for the delayed time horizon used by the EKF.
 * 2) Applies a LPF to state corrections to prevent 'stepping' in states due to measurement
 * fusion introducing unwanted noise into the control loops.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian.
 *
 * "Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements"
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void NavUKF_core::calcOutputStates()
{
    // apply corrections to the IMU data
    Vector3F delAngNewCorrected = imuDataNew.delAng;
    Vector3F delVelNewCorrected = imuDataNew.delVel;
    correctDeltaAngle(delAngNewCorrected, imuDataNew.delAngDT, imuDataNew.gyro_index);
    correctDeltaVelocity(delVelNewCorrected, imuDataNew.delVelDT, imuDataNew.accel_index);

    // apply corrections to track EKF solution
    Vector3F delAng = delAngNewCorrected + delAngCorrection;

    // convert the rotation vector to its equivalent quaternion
    QuaternionF deltaQuat;
    deltaQuat.from_axis_angle(delAng);

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    outputDataNew.quat *= deltaQuat;
    outputDataNew.quat.normalize();

    // calculate the body to nav cosine matrix
    Matrix3F Tbn_temp;
    outputDataNew.quat.rotation_matrix(Tbn_temp);

    // transform body delta velocities to delta velocities in the nav frame
    Vector3F delVelNav  = Tbn_temp*delVelNewCorrected;
    delVelNav.z += GRAVITY_MSS*imuDataNew.delVelDT;

    // save velocity for use in trapezoidal integration for position calcuation
    Vector3F lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    outputDataNew.velocity += delVelNav;

    // Implement third order complementary filter for height and height rate
    // Reference Paper :
    // Optimizing the Gains of the Baro-Inertial Vertical Channel
    // Widnall W.S, Sinha P.K,
    // AIAA Journal of Guidance and Control, 78-1307R

    // Perform filter calculation using backwards Euler integration
    // Coefficients selected to place all three filter poles at omega
    const ftype CompFiltOmega = M_2PI * constrain_ftype(frontend->_hrt_filt_freq, 0.1f, 30.0f);
    ftype omega2 = CompFiltOmega * CompFiltOmega;
    ftype pos_err = constrain_ftype(outputDataNew.position.z - vertCompFiltState.pos, -1e5, 1e5);
    ftype integ1_input = pos_err * omega2 * CompFiltOmega * imuDataNew.delVelDT;
    vertCompFiltState.acc += integ1_input;
    ftype integ2_input = delVelNav.z + (vertCompFiltState.acc + pos_err * omega2 * 3.0f) * imuDataNew.delVelDT;
    vertCompFiltState.vel += integ2_input;
    ftype integ3_input = (vertCompFiltState.vel + pos_err * CompFiltOmega * 3.0f) * imuDataNew.delVelDT;
    vertCompFiltState.pos += integ3_input; 

    // apply a trapezoidal integration to velocities to calculate position
    outputDataNew.position += (outputDataNew.velocity + lastVelocity) * (imuDataNew.delVelDT*0.5f);

    // If the IMU accelerometer is offset from the body frame origin, then calculate corrections
    // that can be added to the EKF velocity and position outputs so that they represent the velocity
    // and position of the body frame origin.
    // Note the * operator has been overloaded to operate as a dot product
    if (!accelPosOffset.is_zero()) {
        // calculate the average angular rate across the last IMU update
        // note delAngDT is prevented from being zero in readIMUData()
        Vector3F angRate = dal.ins().get_gyro(gyro_index_active).toftype();

        // Calculate the velocity of the body frame origin relative to the IMU in body frame
        // and rotate into earth frame. Note % operator has been overloaded to perform a cross product
        Vector3F velBodyRelIMU = angRate % (- accelPosOffset);
        velOffsetNED = Tbn_temp * velBodyRelIMU;

        // calculate the earth frame position of the body frame origin relative to the IMU
        posOffsetNED = Tbn_temp * (- accelPosOffset);
    } else {
        velOffsetNED.zero();
        posOffsetNED.zero();
    }

    // Detect fixed wing launch acceleration using latest data from IMU to enable early startup of filter functions
    // that use launch acceleration to detect start of flight
    if (!inFlight && !dal.get_takeoff_expected() && assume_zero_sideslip()) {
        const ftype launchDelVel = imuDataNew.delVel.x + GRAVITY_MSS * imuDataNew.delVelDT * Tbn_temp.c.x;
        if (launchDelVel > GRAVITY_MSS * imuDataNew.delVelDT) {
            dal.set_takeoff_expected();
        }
    }

    // store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
    if (runUpdates) {
        // store the states at the output time horizon
        storedOutput[storedIMU.get_youngest_index()] = outputDataNew;

        // recall the states from the fusion time horizon
        outputDataDelayed = storedOutput[storedIMU.get_oldest_index()];

        // compare quaternion data with EKF quaternion at the fusion time horizon and calculate correction

        // divide the demanded quaternion by the estimated to get the error
        QuaternionF quatErr = stateStruct.quat / outputDataDelayed.quat;

        // Convert to a delta rotation using a small angle approximation
        quatErr.normalize();
        Vector3F deltaAngErr;
        ftype scaler;
        if (quatErr[0] >= 0.0f) {
            scaler = 2.0f;
        } else {
            scaler = -2.0f;
        }
        deltaAngErr.x = scaler * quatErr[1];
        deltaAngErr.y = scaler * quatErr[2];
        deltaAngErr.z = scaler * quatErr[3];

        // calculate a gain that provides tight tracking of the estimator states and
        // adjust for changes in time delay to maintain consistent damping ratio of ~0.7
        ftype timeDelay = 1e-3f * (ftype)(imuDataNew.time_ms - imuDataDelayed.time_ms);
        timeDelay = MAX(timeDelay, dtIMUavg);
        ftype errorGain = 0.5f / timeDelay;

        // calculate a correction to the delta angle
        // that will cause the INS to track the EKF quaternions
        delAngCorrection = deltaAngErr * errorGain * dtIMUavg;

        // calculate velocity and position tracking errors
        Vector3F velErr = (stateStruct.velocity - outputDataDelayed.velocity);
        Vector3F posErr = (stateStruct.position - outputDataDelayed.position);

        if (badIMUdata) {
            // When IMU accel is bad,  calculate an integral that will be used to drive the difference
            // between the output state and internal EKF state at the delayed time horizon to zero.
            badImuVelErrIntegral += (stateStruct.velocity.z - outputDataNew.velocity.z);
        } else {
            badImuVelErrIntegral = velErrintegral.z;
        }

        // collect magnitude tracking error for diagnostics
        outputTrackError.x = deltaAngErr.length();
        outputTrackError.y = velErr.length();
        outputTrackError.z = posErr.length();

        // convert user specified time constant from centi-seconds to seconds
        ftype tauPosVel = constrain_ftype(0.01f*(ftype)frontend->_tauVelPosOutput, 0.1f, 0.5f);

        // calculate a gain to track the EKF position states with the specified time constant
        ftype velPosGain = dtEkfAvg / constrain_ftype(tauPosVel, dtEkfAvg, 10.0f);

        // use a PI feedback to calculate a correction that will be applied to the output state history
        posErrintegral += posErr;
        velErrintegral += velErr;
        Vector3F posCorrection = posErr * velPosGain + posErrintegral * sq(velPosGain) * 0.1F;
        Vector3F velCorrection;
        velCorrection.x = velErr.x * velPosGain + velErrintegral.x * sq(velPosGain) * 0.1F;
        velCorrection.y = velErr.y * velPosGain + velErrintegral.y * sq(velPosGain) * 0.1F;
        if (badIMUdata) {
            velCorrection.z = velErr.z * velPosGain + badImuVelErrIntegral * sq(velPosGain) * 0.07F;
            velErrintegral.z = badImuVelErrIntegral;
        } else {
            velCorrection.z = velErr.z * velPosGain + velErrintegral.z * sq(velPosGain) * 0.1F;
        }

        // loop through the output filter state history and apply the corrections to the velocity and position states
        // this method is too expensive to use for the attitude states due to the quaternion operations required
        // but does not introduce a time delay in the 'correction loop' and allows smaller tracking time constants
        // to be used
        output_elements outputStates;
        for (unsigned index=0; index < imu_buffer_length; index++) {
            outputStates = storedOutput[index];

            // a constant  velocity correction is applied
            outputStates.velocity += velCorrection;

            // a constant position correction is applied
            outputStates.position += posCorrection;

            // push the updated data to the buffer
            storedOutput[index] = outputStates;
        }

        // update output state to corrected values
        outputDataNew = storedOutput[storedIMU.get_youngest_index()];

    }
}

/*
 * Covariance prediction using the unscented transform.
 * Argument rotVarVecPtr is pointer to a vector defining the earth frame uncertainty variance of the quaternion states
 * used to perform a reset of the quaternion state covariances only. Set to null for normal operation.
 */
void NavUKF_core::CovariancePrediction(Vector3F *rotVarVecPtr)
{
    CovariancePredictionUT(rotVarVecPtr);
}

bool NavUKF_core::choleskyLower(ftype A[24][24], uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j <= i; j++) {
            ftype sum = A[i][j];
            for (uint8_t k = 0; k < j; k++) {
                sum -= A[i][k] * A[j][k];
            }
            if (i == j) {
                if (sum <= 1.0e-12f) {
                    return false;
                }
                A[i][j] = sqrtF(sum);
            } else {
                if (is_zero(A[j][j])) {
                    return false;
                }
                A[i][j] = sum / A[j][j];
            }
        }
        for (uint8_t j = uint8_t(i + 1); j < n; j++) {
            A[i][j] = 0.0f;
        }
    }
    return true;
}

void NavUKF_core::forceCovariancePSD(ftype A[24][24], ftype scratch[24][24], uint8_t n)
{
    // Symmetrise and floor diagonals (do not inflate variances — that delays
    // tiltAlignComplete which gates AHRS UKF primary selection).
    for (uint8_t i = 0; i < n; i++) {
        A[i][i] = fmaxF(A[i][i], 1.0e-12f);
        for (uint8_t j = 0; j < i; j++) {
            const ftype v = 0.5f * (A[i][j] + A[j][i]);
            A[i][j] = A[j][i] = v;
        }
    }

    for (uint8_t attempt = 0; attempt < 8; attempt++) {
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++) {
                scratch[i][j] = A[i][j];
            }
        }
        if (choleskyLower(scratch, n)) {
            return;
        }
        // Shrink cross-covariances; keep variances. Inflating the diagonal
        // was making tiltErrorVariance stay above the 5 deg alignment gate.
        const ftype scale = 0.5f;
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < i; j++) {
                A[i][j] *= scale;
                A[j][i] = A[i][j];
            }
        }
    }

    // Last resort: keep UT variances, drop cross-covariances (still not Jacobian)
    for (uint8_t i = 0; i < n; i++) {
        const ftype d = fmaxF(A[i][i], 1.0e-9f);
        for (uint8_t j = 0; j < n; j++) {
            A[i][j] = (i == j) ? d : 0.0f;
        }
    }
}

// Map IMU process noise into the same multiplicative-quaternion / NED-velocity
// embedding used by UT residuals. Gyro variances are scaled by 4 so that
// ||J col||^2 * 4 = 1 matches the rotation-vector variance in the 0.5*dtheta
// embedding (unscaled J Q J' under-noises attitude and over-trusts the IMU).
static void add_imu_process_noise(ftype P[24][24],
                                  const ftype q[4],
                                  const Matrix3F &Tnb,
                                  ftype daxVar, ftype dayVar, ftype dazVar,
                                  ftype dvxVar, ftype dvyVar, ftype dvzVar)
{
    const ftype J[4][3] = {
        { -0.5f * q[1], -0.5f * q[2], -0.5f * q[3] },
        {  0.5f * q[0], -0.5f * q[3],  0.5f * q[2] },
        {  0.5f * q[3],  0.5f * q[0], -0.5f * q[1] },
        { -0.5f * q[2],  0.5f * q[1],  0.5f * q[0] },
    };
    const ftype gvar[3] = { 4.0f * daxVar, 4.0f * dayVar, 4.0f * dazVar };
    for (uint8_t a = 0; a < 3; a++) {
        for (uint8_t i = 0; i < 4; i++) {
            for (uint8_t j = i; j < 4; j++) {
                const ftype add = J[i][a] * gvar[a] * J[j][a];
                P[i][j] += add;
                if (i != j) {
                    P[j][i] += add;
                }
            }
        }
    }
    const ftype avar[3] = { dvxVar, dvyVar, dvzVar };
    for (uint8_t a = 0; a < 3; a++) {
        const ftype t[3] = { Tnb[a][0], Tnb[a][1], Tnb[a][2] };
        for (uint8_t i = 0; i < 3; i++) {
            for (uint8_t j = i; j < 3; j++) {
                const ftype add = t[i] * avar[a] * t[j];
                P[4 + i][4 + j] += add;
                if (i != j) {
                    P[4 + j][4 + i] += add;
                }
            }
        }
    }
}

void NavUKF_core::CovariancePredictionUT(Vector3F *rotVarVecPtr)
{
    ftype daxVar;       // X axis delta angle noise variance rad^2
    ftype dayVar;       // Y axis delta angle noise variance rad^2
    ftype dazVar;       // Z axis delta angle noise variance rad^2
    ftype dvxVar;       // X axis delta velocity variance noise (m/s)^2
    ftype dvyVar;       // Y axis delta velocity variance noise (m/s)^2
    ftype dvzVar;       // Z axis delta velocity variance noise (m/s)^2

    // Calculate the time step used by the covariance prediction as an average of the gyro and accel integration period
    // Constrain to prevent bad timing jitter causing numerical conditioning problems with the covariance prediction
    dt = constrain_ftype(0.5f*(imuDataDelayed.delAngDT+imuDataDelayed.delVelDT),0.5f * dtEkfAvg, 2.0f * dtEkfAvg);

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.Filter height rate using a 10 second time constant filter
    ftype alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - stateStruct.velocity.z * alpha;

    // calculate covariance prediction process noise added to diagonals of predicted covariance matrix
    // error growth of first 10 kinematic states is built into auto-code for covariance prediction and driven by IMU noise parameters
    Vector14 processNoiseVariance = {};

    if (!inhibitDelAngBiasStates) {
        ftype dAngBiasVar = sq(sq(dt) * constrain_ftype(frontend->_gyroBiasProcessNoise, 0.0, 1.0));
        for (uint8_t i=0; i<=2; i++) processNoiseVariance[i] = dAngBiasVar;
    }

    if (!inhibitDelVelBiasStates) {
        // default process noise (m/s)^2
        ftype dVelBiasVar = sq(sq(dt) * constrain_ftype(frontend->_accelBiasProcessNoise, 0.0, 1.0));
        for (uint8_t i=3; i<=5; i++) {
            processNoiseVariance[i] = dVelBiasVar;
        }
    }

    if (!inhibitMagStates && lastInhibitMagStates) {
        // when starting 3D fusion we want to reset mag variances
        needMagBodyVarReset = true;
        needEarthBodyVarReset = true;
    }

    if (needMagBodyVarReset) {
        // reset body mag variances
        needMagBodyVarReset = false;
        zeroStatesVarCov(19, 21);
        P[19][19] = sq(frontend->_magNoise);
        P[20][20] = P[19][19];
        P[21][21] = P[19][19];
    }

    if (needEarthBodyVarReset) {
        // reset mag earth field variances
        needEarthBodyVarReset = false;
        zeroStatesVarCov(16, 18);
        P[16][16] = sq(frontend->_magNoise);
        P[17][17] = P[16][16];
        P[18][18] = P[16][16];
        // Fusing the declinaton angle as an observaton with a 20 deg uncertainty helps
        // to stabilise the earth field.
        FuseDeclination(radians(20.0f));
    }

    if (!inhibitMagStates) {
        ftype magEarthVar = sq(dt * constrain_ftype(frontend->_magEarthProcessNoise, 0.0f, 1.0f));
        ftype magBodyVar  = sq(dt * constrain_ftype(frontend->_magBodyProcessNoise, 0.0f, 1.0f));
        for (uint8_t i=6; i<=8; i++) processNoiseVariance[i] = magEarthVar;
        for (uint8_t i=9; i<=11; i++) processNoiseVariance[i] = magBodyVar;
    }
    lastInhibitMagStates = inhibitMagStates;

    if (!inhibitWindStates) {
        const bool isDragFusionDeadReckoning = filterStatus.flags.dead_reckoning && !dragTimeout;
        const bool newTreatWindStatesAsTruth = isDragFusionDeadReckoning || !windStateIsObservable;
        if (newTreatWindStatesAsTruth) {
            treatWindStatesAsTruth = true;
            zeroStatesVarCov(22, 23);
        } else {
            if (treatWindStatesAsTruth) {
                treatWindStatesAsTruth = false;
                if (windStateIsObservable) {
                    // allow EKF to relearn wind states rapidly
                    P[23][23] = P[22][22] = sq(WIND_VEL_VARIANCE_MAX);
                }
            }
	        ftype windVelVar  = sq(dt * constrain_ftype(frontend->_windVelProcessNoise, 0.0f, 1.0f) * (1.0f + constrain_ftype(frontend->_wndVarHgtRateScale, 0.0f, 1.0f) * fabsF(hgtRate)));
	        if (!tasDataDelayed.allowFusion) {
	            // Allow wind states to recover faster when using sideslip fusion with a failed airspeed sesnor
	            windVelVar *= 10.0f;
	        }
	        for (uint8_t i=12; i<=13; i++) processNoiseVariance[i] = windVelVar;
        }
    }

    // IMU deltas / attitude used by sigma-point strapdown propagation come from
    // imuDataDelayed / stateBeforePredict (captured before UpdateStrapdownEquationsNED).

    bool quatCovResetOnly = false;
    if (rotVarVecPtr != nullptr) {
        // Handle special case where we are initialising the quaternion covariances using an earth frame
        // vector defining the variance of the angular alignment uncertainty. Convert he varaince vector
        // to a matrix and rotate into body frame. Use the exisiting gyro error propagation mechanism to
        // propagate the body frame angular uncertainty variances.
        const Vector3F &rotVarVec = *rotVarVecPtr;
        Matrix3F R_ef = Matrix3F (
            rotVarVec.x, 0.0f, 0.0f,
            0.0f, rotVarVec.y, 0.0f,
            0.0f, 0.0f, rotVarVec.z);
        Matrix3F Tnb;
        stateStruct.quat.inverse().rotation_matrix(Tnb);
        Matrix3F R_bf = Tnb * R_ef * Tnb.transposed();
        daxVar = R_bf.a.x;
        dayVar = R_bf.b.y;
        dazVar = R_bf.c.z;
        quatCovResetOnly = true;
        zeroStatesVarCov(0, 3);
    } else {
        ftype _gyrNoise = constrain_ftype(frontend->_gyrNoise, 0.0f, 1.0f);
        daxVar = dayVar = dazVar = sq(dt*_gyrNoise);
    }
    ftype _accNoise = badIMUdata ? BAD_IMU_DATA_ACC_P_NSE : constrain_ftype(frontend->_accNoise, 0.0f, BAD_IMU_DATA_ACC_P_NSE);
    dvxVar = dvyVar = dvzVar = sq(dt*_accNoise);

    if (!inhibitDelVelBiasStates) {
        for (uint8_t stateIndex = 13; stateIndex <= 15; stateIndex++) {
            const uint8_t index = stateIndex - 13;

            // Don't attempt learning of IMU delta velocity bias if on ground.
            // In flight: all axes are observable from velocity/position aiding.
            // On ground and stationary: only the gravity-aligned axis (Z for a level
            // vehicle) is observable. XY biases remain unobservable until the vehicle
            // accelerates horizontally in flight.
            // On ground and moving (e.g. carried or on a boat): inhibit all axes
            // to prevent learning biases from external motion accelerations.
            const bool is_bias_observable = (fabsF(prevTnb[index][2]) > 0.8f && onGroundNotMoving) || !onGround;

            if (!is_bias_observable && !dvelBiasAxisInhibit[index]) {
                // store variances to be reinstated wben learning can commence later
                dvelBiasAxisVarPrev[index] = P[stateIndex][stateIndex];
                dvelBiasAxisInhibit[index] = true;
            } else if (is_bias_observable && dvelBiasAxisInhibit[index]) {
                P[stateIndex][stateIndex] = dvelBiasAxisVarPrev[index];
                dvelBiasAxisInhibit[index] = false;
            }
        }
    }

    // nextP scratch (alias of KHP)
    auto& nextP = KHP;

    /*
      Unscented transform covariance prediction.
      Nominal state already integrated by UpdateStrapdownEquationsNED().
      Sigma points from stateBeforePredict + P are strapdown-propagated to form P.
    */
    if (quatCovResetOnly) {
        P[0][0] = daxVar;
        P[1][1] = dayVar;
        P[2][2] = dazVar;
        P[3][3] = daxVar;
        ConstrainVariances();
        calcTiltErrorVariance();
        return;
    }

    ftype mean0[24];
    for (uint8_t i = 0; i <= stateIndexLim; i++) {
        mean0[i] = stateBeforePredict[i];
    }
    if (!drawSigmaPoints(mean0)) {
        // Stay on UT path: inject IMU noise in the tangent embedding and skip this sigma step
        add_imu_process_noise(P, mean0, prevTnb,
                              daxVar, dayVar, dazVar, dvxVar, dvyVar, dvzVar);
        if (stateIndexLim > 9) {
            for (uint8_t i = 10; i <= stateIndexLim; i++) {
                P[i][i] += processNoiseVariance[i - 10];
            }
        }
        ConstrainVariances();
        calcTiltErrorVariance();
        return;
    }
    const uint8_t n = ukf_n;
    const uint8_t n_sigma = ukf_n_sigma;
    const ftype Wm0 = ukf_Wm0;
    const ftype Wc0 = ukf_Wc0;
    const ftype Wi = ukf_Wi;

    // Propagate with RAW IMU deltas; each sigma point applies its own bias
    // and its own start-of-step attitude (see propagateSigmaPoint).
    for (uint8_t s = 0; s < n_sigma; s++) {
        Matrix3F Tnb_s;
        propagateSigmaPoint(sigma_prop[s], Tnb_s,
                            imuDataDelayed.delAng, imuDataDelayed.delVel,
                            imuDataDelayed.delAngDT, imuDataDelayed.delVelDT,
                            dtEkfAvg, earthRateNED);
    }

    ftype mean[24] = {};
    // Tangent-space quaternion mean (USQUE-style): average rotation vectors
    // relative to the propagated central sigma point, then exp-map back.
    {
        QuaternionF qref(sigma_prop[0][0], sigma_prop[0][1], sigma_prop[0][2], sigma_prop[0][3]);
        qref.normalize();
        Vector3F dtheta_bar(0.0f, 0.0f, 0.0f);
        for (uint8_t s = 0; s < n_sigma; s++) {
            const ftype W = (s == 0) ? Wm0 : Wi;
            QuaternionF q_s(sigma_prop[s][0], sigma_prop[s][1], sigma_prop[s][2], sigma_prop[s][3]);
            if ((q_s[0]*qref[0] + q_s[1]*qref[1] + q_s[2]*qref[2] + q_s[3]*qref[3]) < 0) {
                q_s[0] = -q_s[0]; q_s[1] = -q_s[1]; q_s[2] = -q_s[2]; q_s[3] = -q_s[3];
            }
            QuaternionF qerr = qref.inverse() * q_s;
            if (qerr[0] < 0) {
                qerr[0] = -qerr[0]; qerr[1] = -qerr[1]; qerr[2] = -qerr[2]; qerr[3] = -qerr[3];
            }
            Vector3F dtheta;
            qerr.to_axis_angle(dtheta);
            dtheta_bar += dtheta * W;
            for (uint8_t j = 4; j < n; j++) {
                mean[j] += W * sigma_prop[s][j];
            }
        }
        qref.rotate(dtheta_bar);
        qref.normalize();
        mean[0] = qref[0];
        mean[1] = qref[1];
        mean[2] = qref[2];
        mean[3] = qref[3];
    }

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            nextP[i][j] = 0.0f;
        }
    }
    for (uint8_t s = 0; s < n_sigma; s++) {
        const ftype W = (s == 0) ? Wc0 : Wi;
        ftype dx[24];
        // Multiplicative attitude residual mapped back into R^4 tangent at mean
        // so measurement UT Pxz uses the same error embedding as this predict.
        {
            QuaternionF q_mean(mean[0], mean[1], mean[2], mean[3]);
            QuaternionF q_s(sigma_prop[s][0], sigma_prop[s][1], sigma_prop[s][2], sigma_prop[s][3]);
            if ((q_s[0]*q_mean[0] + q_s[1]*q_mean[1] + q_s[2]*q_mean[2] + q_s[3]*q_mean[3]) < 0) {
                q_s[0] = -q_s[0]; q_s[1] = -q_s[1]; q_s[2] = -q_s[2]; q_s[3] = -q_s[3];
            }
            QuaternionF qerr = q_mean.inverse() * q_s;
            if (qerr[0] < 0) {
                qerr[0] = -qerr[0]; qerr[1] = -qerr[1]; qerr[2] = -qerr[2]; qerr[3] = -qerr[3];
            }
            Vector3F dtheta;
            qerr.to_axis_angle(dtheta);
            // δq ≈ 0.5 * q_mean ⊗ [0, δθ]
            const QuaternionF vq(0.0f, 0.5f*dtheta.x, 0.5f*dtheta.y, 0.5f*dtheta.z);
            const QuaternionF dq = q_mean * vq;
            dx[0] = dq[0]; dx[1] = dq[1]; dx[2] = dq[2]; dx[3] = dq[3];
        }
        for (uint8_t j = 4; j < n; j++) {
            dx[j] = sigma_prop[s][j] - mean[j];
        }
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = i; j < n; j++) {
                nextP[i][j] += W * dx[i] * dx[j];
            }
        }
    }

    Matrix3F Tnb0;
    {
        QuaternionF q0(mean0[0], mean0[1], mean0[2], mean0[3]);
        q0.normalize();
        q0.inverse().rotation_matrix(Tnb0);
    }
    add_imu_process_noise(nextP, mean, Tnb0,
                          daxVar, dayVar, dazVar, dvxVar, dvyVar, dvzVar);
    if (stateIndexLim > 9) {
        for (uint8_t i = 10; i <= stateIndexLim; i++) {
            nextP[i][i] += processNoiseVariance[i - 10];
        }
    }

    if ((P[7][7] + P[8][8]) > 1e4f) {
        for (uint8_t i = 7; i <= 8; i++) {
            for (uint8_t j = 0; j <= stateIndexLim; j++) {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // Do not force PSD on every UT output: repeated off-diagonal shrinkage
    // destroys cross-covariances and prevents tilt alignment. Measurement UT
    // repairs P reactively when Pzz < R or FinishFusion detects ill-conditioning.

    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        P[row][row] = nextP[row][row];
        for (uint8_t column = 0; column < row; column++) {
            P[row][column] = P[column][row] = nextP[column][row];
        }
    }

    if (!inhibitDelVelBiasStates) {
        for (uint8_t index = 0; index < 3; index++) {
            const uint8_t stateIndex = index + 13;
            if (dvelBiasAxisInhibit[index]) {
                zeroStatesVarCov(stateIndex, stateIndex);
                P[stateIndex][stateIndex] = dvelBiasAxisVarPrev[index];
            }
        }
    }

    ConstrainVariances();

    if (vertVelVarClipCounter > 0) {
        vertVelVarClipCounter--;
    }

    calcTiltErrorVariance();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    verifyTiltErrorVariance();
#endif
}



void NavUKF_core::propagateSigmaPoint(ftype x[24],
                                      Matrix3F &Tnb,
                                      const Vector3F &delAng,
                                      const Vector3F &delVel,
                                      ftype delAngDT,
                                      ftype delVelDT,
                                      ftype dt_ekf_avg,
                                      const Vector3F &earth_rate_ned)
{
    // Start-of-step attitude for THIS sigma point (not the nominal Tnb).
    QuaternionF quat(x[0], x[1], x[2], x[3]);
    quat.normalize();
    quat.inverse().rotation_matrix(Tnb);

    // Biases are stored as delta-angle / delta-velocity over dtEkfAvg (same as
    // correctDeltaAngle / correctDeltaVelocity). Apply to RAW IMU samples.
    const Vector3F gyro_bias(x[10], x[11], x[12]);
    const Vector3F accel_bias(x[13], x[14], x[15]);
    const ftype ang_scale = (dt_ekf_avg > 0) ? (delAngDT / dt_ekf_avg) : 1.0f;
    const ftype vel_scale = (dt_ekf_avg > 0) ? (delVelDT / dt_ekf_avg) : 1.0f;
    const Vector3F delAngCorr = delAng - gyro_bias * ang_scale;
    const Vector3F delVelCorr = delVel - accel_bias * vel_scale;

    // Attitude update using this sigma point's start-of-step Tnb for earth rate
    quat.rotate(delAngCorr - Tnb * earth_rate_ned * delAngDT);
    quat.normalize();

    // Velocity/position: rotate body delta-vel with start-of-step attitude
    // (matches UpdateStrapdownEquationsNED which uses prevTnb).
    Vector3F delVelNav = Tnb.mul_transpose(delVelCorr);
    delVelNav.z += GRAVITY_MSS * delVelDT;

    Vector3F velocity(x[4], x[5], x[6]);
    Vector3F position(x[7], x[8], x[9]);
    const Vector3F lastVelocity = velocity;
    velocity += delVelNav;
    position += (velocity + lastVelocity) * (delVelDT * 0.5f);

    // End-of-step Tnb for caller
    quat.inverse().rotation_matrix(Tnb);

    x[0] = quat[0]; x[1] = quat[1]; x[2] = quat[2]; x[3] = quat[3];
    x[4] = velocity.x; x[5] = velocity.y; x[6] = velocity.z;
    x[7] = position.x; x[8] = position.y; x[9] = position.z;
    // biases, mag, wind unchanged
}

void NavUKF_core::zeroStatesVarCov(uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        zero_range(&P[row][0], 0, 23);
    }

    for (row=0; row<=23; row++)
    {
        zero_range(&P[row][0], first, last);
    }
}

// reset the output data to the current EKF state
void NavUKF_core::StoreOutputReset()
{
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i] = outputDataNew;
    }
    outputDataDelayed = outputDataNew;
    // reset the states for the complementary filter used to provide a vertical position derivative output
    vertCompFiltState.pos = stateStruct.position.z;
    vertCompFiltState.vel = stateStruct.velocity.z;
}

// Reset the stored output quaternion history to current EKF state
void NavUKF_core::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// Rotate the stored output quaternion history through a quaternion rotation
void NavUKF_core::StoreQuatRotate(const QuaternionF &deltaQuat)
{
    outputDataNew.quat = outputDataNew.quat*deltaQuat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = storedOutput[i].quat*deltaQuat;
    }
    outputDataDelayed.quat = outputDataDelayed.quat*deltaQuat;
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
// if states are inactive, zero the corresponding off-diagonals
void NavUKF_core::ConstrainVariances()
{
    // Covariance constraints as of March 2025
    // This table assumes normal operations, there are additional constraints for specific failure modes like "badIMUdata" and "inhibitDelAngBiasStates".
    // +----------------------------------------------------------------------------------------------+
    // | State Index  |      State Name                 |  State Units  | Variance Constraint Range   |
    // +----------------------------------------------------------------------------------------------+
    // |  0 .. 3      | Attitude Quaternion             | unitless      | [0.0, 1.0]                  |
    // |  4 .. 5      | Velocity (North, East)          | m/s           | [1e-4, 1e3]                 |
    // |  6           | Velocity (Down)                 | m/s           | dynamic                     |
    // |  7 .. 9      | Position (North, East, Down)    | m             | [1e-4, 1e6]                 |
    // | 10 .. 12     | Gyro Bias (X, Y, Z)             | rad           | [0.0, (0.175 * dtEkfAvg)^2] |
    // | 13 .. 15     | Accel Bias (X, Y, Z)            | m/s^2         | dynamic                     |
    // | 16 .. 18     | Earth Magnetic Field (X, Y, Z)  | Gauss         | [0.0, 0.01]                 |
    // | 19 .. 21     | Body Magnetic Field (X, Y, Z)   | Gauss         | [0.0, 0.01]                 |
    // | 22 .. 23     | Wind Velocity (North, East)     | m/s           | [0.0, 400]                  |
    // +----------------------------------------------------------------------------------------------+

    for (uint8_t i=0; i<=3; i++) P[i][i] = constrain_ftype(P[i][i],0.0,1.0); // attitude error
    for (uint8_t i=4; i<=5; i++) P[i][i] = constrain_ftype(P[i][i], VEL_STATE_MIN_VARIANCE, 1.0e3); // NE velocity

    // if vibration affected use sensor observation variances to set a floor on the state variances
    if (badIMUdata) {
        P[6][6] = fmaxF(P[6][6], sq(frontend->_gpsVertVelNoise));
        P[9][9] = fmaxF(P[9][9], sq(frontend->_baroAltNoise));
    } else if (P[6][6] < VEL_STATE_MIN_VARIANCE) {
        // handle collapse of the vertical velocity variance
        P[6][6] = VEL_STATE_MIN_VARIANCE;
        // this counter is decremented by 1 each prediction cycle in CovariancePrediction
        // resulting in the count from each clip event fading to zero over 1 second which
        // is sufficient to capture collapse from fusion of the lowest update rate sensor
        vertVelVarClipCounter += EKF_TARGET_RATE_HZ;
        if (vertVelVarClipCounter > VERT_VEL_VAR_CLIP_COUNT_LIM) {
            // reset the corresponding covariances
            zeroStatesVarCov(6, 6);

            // set the variances to the measurement variance
        #if UKF_FEATURE_EXTERNAL_NAV
            if (useExtNavVel) {
                P[6][6] = sq(extNavVelDelayed.err);
            } else
        #endif
            {
                P[6][6] = sq(frontend->_gpsVertVelNoise);
            }
            vertVelVarClipCounter = 0;
        }
    }

    for (uint8_t i=7; i<=9; i++) P[i][i] = constrain_ftype(P[i][i], POS_STATE_MIN_VARIANCE, 1.0e6); // NED position

    if (!inhibitDelAngBiasStates) {
        for (uint8_t i=10; i<=12; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,sq(0.175 * dtEkfAvg));
    } else {
        zeroStatesVarCov(10, 12);
    }

    const ftype minSafeStateVar = 5E-9;
    if (!inhibitDelVelBiasStates) {

        // Find the maximum delta velocity bias state variance and request a covariance reset if any variance is below the safe minimum
        ftype maxStateVar = 0.0F;
        bool resetRequired = false;
        for (uint8_t stateIndex=13; stateIndex<=15; stateIndex++) {
            if (P[stateIndex][stateIndex] > maxStateVar) {
                maxStateVar = P[stateIndex][stateIndex];
            } else if (P[stateIndex][stateIndex] < minSafeStateVar) {
                resetRequired = true;
            }
        }

        // To ensure stability of the covariance matrix operations, the ratio of a max and min variance must
        // not exceed 100 and the minimum variance must not fall below the target minimum
        ftype minAllowedStateVar = fmaxF(0.01f * maxStateVar, minSafeStateVar);
        for (uint8_t stateIndex=13; stateIndex<=15; stateIndex++) {
            P[stateIndex][stateIndex] = constrain_ftype(P[stateIndex][stateIndex], minAllowedStateVar, sq(10.0f * dtEkfAvg));
        }

        // If any one axis has fallen below the safe minimum, all delta velocity covariance terms must be reset to zero
        if (resetRequired) {
            // reset all delta velocity bias covariances
            zeroStatesVarCov(13, 15);
            // set all delta velocity bias variances to initial values and zero bias states
            P[13][13] = sq(ACCEL_BIAS_LIM_SCALER * frontend->_accBiasLim * dtEkfAvg);
            P[14][14] = P[13][13];
            P[15][15] = P[13][13];
            stateStruct.accel_bias.zero();
        }

    } else {
        zeroStatesVarCov(13, 15);
        // set all delta velocity bias variances to a margin above the minimum safe value
        for (uint8_t i=0; i<=2; i++) {
            const uint8_t stateIndex = i + 13;
            P[stateIndex][stateIndex] = fmaxF(P[stateIndex][stateIndex], minSafeStateVar * 10.0F);
        }
    }

    if (!inhibitMagStates) {
        for (uint8_t i=16; i<=18; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,0.01f); // earth magnetic field
        for (uint8_t i=19; i<=21; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,0.01f); // body magnetic field
    } else {
        zeroStatesVarCov(16, 21);
    }

    if (!inhibitWindStates) {
        if (treatWindStatesAsTruth) {
            zeroStatesVarCov(22, 23);
        } else {
            for (uint8_t i=22; i<=23; i++) P[i][i] = constrain_ftype(P[i][i],0.0f,WIND_VEL_VARIANCE_MAX);
        }
    } else {
        zeroStatesVarCov(22, 23);
    }
}

// actually do fusion to update statesArray from Kfusion and P from KHP.
// returns true and skips fusion if variances would be driven negative.
// force skips this negative check; passing true is probably a bug!
bool NavUKF_core::FinishFusion(ftype innov, bool force /*= false*/)
{
    if (!force) {
        // Check that we are not going to drive any variances negative and skip the update if so
        for (auto s=0; s<=stateIndexLim; s++) {
            if (KHP[s][s] > P[s][s]) {
                return true;
            }
        }
    }

    // correct the state vector using kalman gains filled in by caller
    for (auto s=0; s<=stateIndexLim; s++) {
        statesArray[s] -= Kfusion[s] * innov;
    }
    stateStruct.quat.normalize();

    // update the covariance matrix as P = P - KHP (KHP was filled by caller)
    for (auto r=0; r<=stateIndexLim; r++) {
        for (auto c=0; c<=r; c++) {
            // P must end up symmetric, so average the upper and lower
            // differences, then store that result in both positions. it would
            // be faster and more numerically stable to average the KHP entries
            // instead, but we have no good proof P was symmetric before!
            const ftype lower = P[r][c] - KHP[r][c];
            const ftype upper = P[c][r] - KHP[c][r];
            const ftype res = 0.5f*(lower + upper);
            P[r][c] = res;
            P[c][r] = res;
        }
    }

    // limit the variances to prevent ill-conditioning
    ConstrainVariances(); // can change statesArray!!

    return false;
}

// constrain states using WMM tables and specified limit
void NavUKF_core::MagTableConstrain(void)
{
    // constrain to error from table earth field
    ftype limit_ga = frontend->_mag_ef_limit * 0.001f;
    stateStruct.earth_magfield.x = constrain_ftype(stateStruct.earth_magfield.x,
                                                   table_earth_field_ga.x-limit_ga,
                                                   table_earth_field_ga.x+limit_ga);
    stateStruct.earth_magfield.y = constrain_ftype(stateStruct.earth_magfield.y,
                                                   table_earth_field_ga.y-limit_ga,
                                                   table_earth_field_ga.y+limit_ga);
    stateStruct.earth_magfield.z = constrain_ftype(stateStruct.earth_magfield.z,
                                                   table_earth_field_ga.z-limit_ga,
                                                   table_earth_field_ga.z+limit_ga);
}

// constrain states to prevent ill-conditioning
void NavUKF_core::ConstrainStates()
{
    // State constraints as of March 2025
    // This table documents the limits applied to each EKF state.
    // These are designed to keep state estimates within physically realistic bounds and prevent divergence.
    // +---------------------------------------------------------------------------------------------------------+
    // | State Index  |      State Name                 |  State Units  | State Constraint Range                 |
    // +---------------------------------------------------------------------------------------------------------+
    // |  0 .. 3      | Attitude Quaternion             | unitless      | [-1.0, 1.0]                            |
    // |  4 .. 6      | Velocity (North, East, Down)    | m/s           | [-500, 500]                            |
    // |  7 .. 8      | Position (North, East)          | m             | [-50e6,50e6]                           |
    // |  9 (z)       | Position (Down / Altitude)      | m             | [-40000, 10000]                        |
    // | 10 .. 12     | Gyro Bias (X, Y, Z)             | rad           | [-0.5, 0.5] * dtEkfAvg                 |
    // | 13 .. 15     | Accel Bias (X, Y, Z)            | m/s²          | [-_accBiasLim, _accBiasLim] * dtEkfAvg |
    // | 16 .. 18     | Earth Magnetic Field (X, Y, Z)  | Gauss         | [-1.0, 1.0]                            | or constrained by MagTableConstrain() if available
    // | 19 .. 21     | Body Magnetic Field (X, Y, Z)   | Gauss         | [-0.5, 0.5]                            |
    // | 22 .. 23     | Wind Velocity (North, East)     | m/s           | [-100, 100]                            |
    // +---------------------------------------------------------------------------------------------------------+

    // quaternions are limited between +-1
    for (uint8_t i=0; i<=3; i++) statesArray[i] = constrain_ftype(statesArray[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=4; i<=6; i++) statesArray[i] = constrain_ftype(statesArray[i],-5.0e2f,5.0e2f);
    // position limit TODO apply circular limit
    for (uint8_t i=7; i<=8; i++) statesArray[i] = constrain_ftype(statesArray[i],-UKF_POSXY_STATE_LIMIT,UKF_POSXY_STATE_LIMIT);
    // height limit covers home alt on everest through to home alt at SL and balloon drop
    stateStruct.position.z = constrain_ftype(stateStruct.position.z,-4.0e4f,1.0e4f);
    // gyro bias limit (this needs to be set based on manufacturers specs)
    const ftype gyro_bias_limit = getGyroBiasLimit();
    for (uint8_t i=10; i<=12; i++) statesArray[i] = constrain_ftype(statesArray[i],-gyro_bias_limit*dtEkfAvg,gyro_bias_limit*dtEkfAvg);
    // the accelerometer bias limit is controlled by a user adjustable parameter
    for (uint8_t i=13; i<=15; i++) statesArray[i] = constrain_ftype(statesArray[i],-frontend->_accBiasLim*dtEkfAvg,frontend->_accBiasLim*dtEkfAvg);
    // earth magnetic field limit
    if (frontend->_mag_ef_limit <= 0 || !have_table_earth_field) {
        // constrain to +/-1Ga
        for (uint8_t i=16; i<=18; i++) statesArray[i] = constrain_ftype(statesArray[i],-1.0f,1.0f);
    } else {
        // use table constrain
        MagTableConstrain();
    }
    // body magnetic field limit
    for (uint8_t i=19; i<=21; i++) statesArray[i] = constrain_ftype(statesArray[i],-0.5f,0.5f);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=22; i<=23; i++) statesArray[i] = constrain_ftype(statesArray[i],-100.0f,100.0f);
    // constrain the terrain state to be below the vehicle height unless we are using terrain as the height datum
    if (!inhibitGndState) {
        terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);
    }
}

// calculate the NED earth spin vector in rad/sec
void NavUKF_core::calcEarthRateNED(Vector3F &omega, int32_t latitude) const
{
    ftype lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosF(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinF(lat_rad);
}

// set yaw from a single magnetometer sample
void NavUKF_core::setYawFromMag()
{
    if (!use_compass()) {
        return;
    }

    // read the magnetometer data
    readMagData();

    // use best of either 312 or 321 rotation sequence when calculating yaw
    rotationOrder order;
    bestRotationOrder(order);
    Vector3F eulerAngles;
    Matrix3F Tbn_zeroYaw;
    if (order == rotationOrder::TAIT_BRYAN_321) {
        // rolled more than pitched so use 321 rotation order
        stateStruct.quat.to_euler(eulerAngles);
        Tbn_zeroYaw.from_euler(eulerAngles.x, eulerAngles.y, 0.0f);
    } else if (order == rotationOrder::TAIT_BRYAN_312) {
        // pitched more than rolled so use 312 rotation order
        eulerAngles = stateStruct.quat.to_vector312();
        Tbn_zeroYaw.from_euler312(eulerAngles.x, eulerAngles.y, 0.0f);
    } else {
        // rotation order not supported
        return;
    }

    Vector3F magMeasNED = Tbn_zeroYaw * magDataDelayed.mag;
    ftype yawAngMeasured = wrap_PI(-atan2F(magMeasNED.y, magMeasNED.x) + MagDeclination());

    // update quaternion states and covariances
    resetQuatStateYawOnly(yawAngMeasured, sq(MAX(frontend->_yawNoise, 1.0e-2f)), order);
}

// update mag field states and associated variances using magnetomer and declination data
void NavUKF_core::resetMagFieldStates()
{
    // Rotate Mag measurements into NED to set initial NED magnetic field states

    // update rotation matrix from body to NED frame
    stateStruct.quat.inverse().rotation_matrix(prevTnb);

    if (have_table_earth_field && frontend->_mag_ef_limit > 0) {
        stateStruct.earth_magfield = table_earth_field_ga;
    } else {
        stateStruct.earth_magfield = prevTnb.transposed() * magDataDelayed.mag;
    }

    // set the NE earth magnetic field states using the published declination
    // and set the corresponding variances and covariances
    alignMagStateDeclination();

    // set the remaining variances and covariances
    zeroStatesVarCov(18, 21);
    P[18][18] = sq(frontend->_magNoise);
    P[19][19] = P[18][18];
    P[20][20] = P[18][18];
    P[21][21] = P[18][18];

    // record the fact we have initialised the magnetic field states
    recordMagReset();
}

// calculate the tilt error variance
void NavUKF_core::calcTiltErrorVariance()
{
    const ftype &q0 = stateStruct.quat[0];
    const ftype &q1 = stateStruct.quat[1];
    const ftype &q2 = stateStruct.quat[2];
    const ftype &q3 = stateStruct.quat[3];

    // equations generated by quaternion_error_propagation(): in derivation/generate_2.py
    // only diagonals have been used
    const ftype PS0 = q0*q1 + q2*q3;
    const ftype PS1 = PS0*q1;
    const ftype PS2 = sq(q0) - sq(q1) - sq(q2) + sq(q3);
    const ftype PS3 = PS2*q0;
    const ftype PS4 = PS0*q2;
    const ftype PS5 = PS2*q3;
    const ftype PS6 = PS0*q3;
    const ftype PS7 = PS2*q2;
    const ftype PS8 = PS0*q0;
    const ftype PS9 = PS2*q1;
    const ftype PS10 = q0*q2 - q1*q3;
    const ftype PS11 = PS10*q2;
    const ftype PS12 = PS10*q3;
    const ftype PS13 = PS10*q0;
    const ftype PS14 = PS10*q1;

    tiltErrorVariance  = 4*P[0][0]*sq(2*PS8 - PS9) + 4*P[1][1]*sq(2*PS1 + PS3) + 4*P[2][2]*sq(2*PS4 + PS5) + 4*P[3][3]*sq(-2*PS6 + PS7);
    tiltErrorVariance += 4*P[0][0]*sq(2*PS13 - PS7) + 4*P[1][1]*sq(2*PS14 - PS5) + 4*P[2][2]*sq(2*PS11 + PS3) + 4*P[3][3]*sq(2*PS12 + PS9);
    tiltErrorVariance += 16*P[0][0]*sq(PS14 - PS4) + 16*P[1][1]*sq(PS13 + PS6) + 16*P[2][2]*sq(-PS12 + PS8) + 16*P[3][3]*sq(PS1 + PS11);

    tiltErrorVariance = constrain_ftype(tiltErrorVariance, 0.0f, sq(radians(30.0f)));
}

void NavUKF_core::bestRotationOrder(rotationOrder &order)
{
    if (fabsF(prevTnb[2][0]) < fabsF(prevTnb[2][1])) {
        // rolled more than pitched so use 321 sequence
        order = rotationOrder::TAIT_BRYAN_321;
    } else {
        // pitched more than rolled so use 312 sequence
        order = rotationOrder::TAIT_BRYAN_312;
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// calculate the tilt error variance using an alternative numerical difference technique
// and log with value generated by NavUKF_core::calcTiltErrorVariance()
void NavUKF_core::verifyTiltErrorVariance()
{
#if HAL_LOGGING_ENABLED
    const Vector3f gravity_ef = Vector3f(0.0f,0.0f,1.0f);
    Matrix3f Tnb;
    const float quat_delta = 0.001f;
    float tiltErrorVarianceAlt = 0.0f;
    for (uint8_t index = 0; index<4; index++) {
        QuaternionF quat = stateStruct.quat;

        // Add a positive increment to the quaternion element and calculate the tilt error vector
        quat[index] = stateStruct.quat[index] + quat_delta;
        quat.inverse().rotation_matrix(Tnb);
        const Vector3f gravity_bf_plus = Tnb * gravity_ef;

        // Add a negative increment to the quaternion element and calculate the tilt error vector
        quat[index] = stateStruct.quat[index] - quat_delta;
        quat.inverse().rotation_matrix(Tnb);
        const Vector3f gravity_bf_minus = Tnb * gravity_ef;

        // calculate the angular difference between the two vectors using a small angle assumption
        const Vector3f tilt_diff_vec = gravity_bf_minus % gravity_bf_plus;

        // calculate the partial derivative of angle error wrt the quaternion element
        const float tilt_error_derivative = tilt_diff_vec.length() / (2.0f * quat_delta);

        // sum the contribution of the quaternion elemnent variance to the tilt angle error variance
        tiltErrorVarianceAlt += P[index][index] * sq(tilt_error_derivative);
    }

    tiltErrorVarianceAlt = MIN(tiltErrorVarianceAlt, sq(radians(30.0f)));
    /* UKFTV logging omitted */
#endif  // HAL_LOGGING_ENABLED
}
#endif

/*
  move the EKF origin to the current position at 1Hz. The public_origin doesn't move.
  By moving the EKF origin we keep the distortion due to spherical
  shape of the earth to a minimum.
 */
void NavUKF_core::moveEKFOrigin(void)
{
    // only move origin when we have a origin and we're using GPS
    if (!frontend->common_origin_valid || !filterStatus.flags.using_gps) {
        return;
    }

    // move the origin to the current state location
    Location loc = EKF_origin;
    loc.offset(stateStruct.position.x, stateStruct.position.y);
    const Vector2F diffNE = loc.get_distance_NE_ftype(EKF_origin);
    EKF_origin = loc;

    // now fix all output states
    stateStruct.position.xy() += diffNE;
    outputDataNew.position.xy() += diffNE;
    outputDataDelayed.position.xy() += diffNE;

    for (unsigned index=0; index < imu_buffer_length; index++) {
        storedOutput[index].position.xy() += diffNE;
    }
}
