#include <AP_HAL/AP_HAL.h>

#include "AP_NavUKF.h"
#include "AP_NavUKF_core.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_DAL/AP_DAL.h>

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset XY velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavUKF_core::ResetVelocity(resetDataSource velResetSource)
{
    // if reset source is not specified then use user defined velocity source
    if (velResetSource == resetDataSource::DEFAULT) {
        switch (frontend->sources.getVelXYSource(core_index)) {
        case AP_NavEKF_Source::SourceXY::GPS:
            velResetSource = resetDataSource::GPS;
            break;
        case AP_NavEKF_Source::SourceXY::BEACON:
            velResetSource = resetDataSource::RNGBCN;
            break;
        case AP_NavEKF_Source::SourceXY::EXTNAV:
            velResetSource = resetDataSource::EXTNAV;
            break;
        case AP_NavEKF_Source::SourceXY::NONE:
        case AP_NavEKF_Source::SourceXY::OPTFLOW:
        case AP_NavEKF_Source::SourceXY::WHEEL_ENCODER:
            // unhandled sources so stick with the default
            break;
        }
    }

    // reset the corresponding covariances
    zeroStatesVarCov(4, 5);

    if (PV_AidingMode != AID_ABSOLUTE) {
        stateStruct.velocity.xy().zero();
        // set the variances using the measurement noise parameter
        P[5][5] = P[4][4] = sq(frontend->_gpsHorizVelNoise);
    } else {
        // reset horizontal velocity states to the GPS velocity if available
        if ((imuSampleTime_ms - lastTimeGpsReceived_ms < 250) && (velResetSource == resetDataSource::DEFAULT || velResetSource == resetDataSource::GPS)) {
            // correct for antenna position
            gps_elements gps_corrected = gpsDataNew;
            CorrectGPSForAntennaOffset(gps_corrected);
            stateStruct.velocity.x  = gps_corrected.vel.x;
            stateStruct.velocity.y  = gps_corrected.vel.y;
            // set the variances using the reported GPS speed accuracy
            P[5][5] = P[4][4] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));
#if UKF_FEATURE_EXTERNAL_NAV
        } else if ((imuSampleTime_ms - extNavVelMeasTime_ms < 250) && (velResetSource == resetDataSource::DEFAULT || velResetSource == resetDataSource::EXTNAV)) {
            // use external nav data as the 2nd preference
            // already corrected for sensor position
            stateStruct.velocity.x = extNavVelDelayed.vel.x;
            stateStruct.velocity.y = extNavVelDelayed.vel.y;
            P[5][5] = P[4][4] = sq(extNavVelDelayed.err);
#endif // UKF_FEATURE_EXTERNAL_NAV
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range
            P[5][5] = P[4][4] = sq(25.0f);
        }
        // clear the timeout flags and counters
        velTimeout = false;
        lastVelPassTime_ms = imuSampleTime_ms;
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;

}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavUKF_core::ResetPosition(resetDataSource posResetSource)
{
    // if reset source is not specified thenn use the user defined position source
    if (posResetSource == resetDataSource::DEFAULT) {
        switch (frontend->sources.getPosXYSource(core_index)) {
        case AP_NavEKF_Source::SourceXY::GPS:
            posResetSource = resetDataSource::GPS;
            break;
        case AP_NavEKF_Source::SourceXY::BEACON:
            posResetSource = resetDataSource::RNGBCN;
            break;
        case AP_NavEKF_Source::SourceXY::EXTNAV:
            posResetSource = resetDataSource::EXTNAV;
            break;
        case AP_NavEKF_Source::SourceXY::NONE:
        case AP_NavEKF_Source::SourceXY::OPTFLOW:
        case AP_NavEKF_Source::SourceXY::WHEEL_ENCODER:
            // invalid sources so stick with the default
            break;
        }
    }

    // Store the position before the reset so that we can record the reset delta
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    zeroStatesVarCov(7, 8);

    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
        // set the variances using the position measurement noise parameter
        P[7][7] = P[8][8] = sq(frontend->_gpsHorizPosNoise);
    } else  {
        // Use GPS data as first preference if fresh data is available
        if ((imuSampleTime_ms - lastTimeGpsReceived_ms < 250) && (posResetSource == resetDataSource::DEFAULT || posResetSource == resetDataSource::GPS)) {
            // correct for antenna position
            gps_elements gps_corrected = gpsDataNew;
            CorrectGPSForAntennaOffset(gps_corrected);
            // record the ID of the GPS for the data we are using for the reset
            last_gps_idx = gps_corrected.sensor_idx;
            // calculate position
            const Location gpsloc{gps_corrected.lat, gps_corrected.lng, 0, Location::AltFrame::ABSOLUTE};
            stateStruct.position.xy() = EKF_origin.get_distance_NE_ftype(gpsloc);
            // compensate for offset  between last GPS measurement and the EKF time horizon. Note that this is an unusual
            // time delta in that it can be both -ve and +ve
            const int32_t tdiff = imuDataDelayed.time_ms - gps_corrected.time_ms;
            stateStruct.position.xy() += gps_corrected.vel.xy()*0.001*tdiff;
            // set the variances using the position measurement noise parameter
            P[7][7] = P[8][8] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
#if UKF_FEATURE_BEACON_FUSION
        } else if ((imuSampleTime_ms - rngBcn.last3DmeasTime_ms < 250) && (posResetSource == resetDataSource::DEFAULT || posResetSource == resetDataSource::RNGBCN)) {
            // use the range beacon data as a second preference
            stateStruct.position.x = rngBcn.receiverPos.x;
            stateStruct.position.y = rngBcn.receiverPos.y;
            // set the variances from the beacon alignment filter
            P[7][7] = rngBcn.receiverPosCov[0][0];
            P[8][8] = rngBcn.receiverPosCov[1][1];
#endif
#if UKF_FEATURE_EXTERNAL_NAV
        } else if ((imuSampleTime_ms - extNavDataDelayed.time_ms < 250) && (posResetSource == resetDataSource::DEFAULT || posResetSource == resetDataSource::EXTNAV)) {
            // use external nav data as the third preference
            stateStruct.position.x = extNavDataDelayed.pos.x;
            stateStruct.position.y = extNavDataDelayed.pos.y;
            // set the variances as received from external nav system data
            P[7][7] = P[8][8] = sq(extNavDataDelayed.posErr);
#endif // UKF_FEATURE_EXTERNAL_NAV
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.x = stateStruct.position.x;
        storedOutput[i].position.y = stateStruct.position.y;
    }
    outputDataNew.position.x = stateStruct.position.x;
    outputDataNew.position.y = stateStruct.position.y;
    outputDataDelayed.position.x = stateStruct.position.x;
    outputDataDelayed.position.y = stateStruct.position.y;

    // Calculate the position jump due to the reset
    posResetNE.x = stateStruct.position.x - posResetNE.x;
    posResetNE.y = stateStruct.position.y - posResetNE.y;

    posNEResetCount++;

    // clear the timeout flags and counters
    posTimeout = false;
    lastGpsPosPassTime_ms = imuSampleTime_ms;
}

#if UKF_FEATURE_POSITION_RESET
// Sets the EKF's NE horizontal position states and their corresponding variances from a supplied WGS-84 location and optionally uncertainty
// The altitude element of the location is not used. If accuracy is not known should be passed as NaN.
// Returns true if the set was successful
bool NavUKF_core::setLatLng(const Location &loc, float posAccuracy, uint32_t timestamp_ms)
{
    if ((imuSampleTime_ms - lastGpsPosPassTime_ms) < frontend->deadReckonDeclare_ms ||
        (PV_AidingMode == AID_NONE)
        || !validOrigin) {
        return false;
    }

    // Store the position before the reset so that we can record the reset delta
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    zeroStatesVarCov(7, 8);

    // handle unknown accuracy
    if (isnan(posAccuracy)) {
        posAccuracy = 0.0f; // will be ignored due to MAX below
    }

    // set the variances using the position measurement noise parameter
    P[7][7] = P[8][8] = sq(MAX(posAccuracy,frontend->_gpsHorizPosNoise));

    // Correct the position for time delay relative to fusion time horizon assuming a constant velocity
    // Limit time stamp to a range between current time and 5 seconds ago
    const uint32_t timeStampConstrained_ms = MAX(MIN(timestamp_ms, imuSampleTime_ms), imuSampleTime_ms - 5000);
    const int32_t delta_ms = int32_t(imuDataDelayed.time_ms - timeStampConstrained_ms);
    const ftype delaySec = 1E-3F * ftype(delta_ms);
    const Vector2F newPosNE = EKF_origin.get_distance_NE_ftype(loc) + stateStruct.velocity.xy() * delaySec;
    ResetPositionNE(newPosNE.x,newPosNE.y);

    return true;
}
#endif // UKF_FEATURE_POSITION_RESET


// reset the stateStruct's NE position to the specified position
//    posResetNE is updated to hold the change in position
//    storedOutput, outputDataNew and outputDataDelayed are updated with the change in position
//    posNEResetCount is incremented to record the reset
void NavUKF_core::ResetPositionNE(ftype posN, ftype posE)
{
    // Store the position before the reset so that we can record the reset delta
    const Vector3F posOrig = stateStruct.position;

    // Set the position states to the new position
    stateStruct.position.x = posN;
    stateStruct.position.y = posE;

    // Calculate the position offset due to the reset
    posResetNE.x = stateStruct.position.x - posOrig.x;
    posResetNE.y = stateStruct.position.y - posOrig.y;

    // Add the offset to the output observer states
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.x += posResetNE.x;
        storedOutput[i].position.y += posResetNE.y;
    }
    outputDataNew.position.x += posResetNE.x;
    outputDataNew.position.y += posResetNE.y;
    outputDataDelayed.position.x += posResetNE.x;
    outputDataDelayed.position.y += posResetNE.y;

    posNEResetCount++;
}

// reset the stateStruct's D position
//    posResetD is updated to hold the change in position
//    storedOutput, outputDataNew and outputDataDelayed are updated with the change in position
//    posDResetCount is incremented to record the reset
void NavUKF_core::ResetPositionD(ftype posD)
{
    // Store the position before the reset so that we can record the reset delta
    const ftype posDOrig = stateStruct.position.z;

    // write to the state vector
    stateStruct.position.z = posD;

    // Calculate the position jump due to the reset
    posResetD = stateStruct.position.z - posDOrig;

    // Add the offset to the output observer states
    outputDataNew.position.z += posResetD;
    vertCompFiltState.pos = outputDataNew.position.z;
    outputDataDelayed.position.z += posResetD;
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z += posResetD;
    }

    posDResetCount++;
}

// reset the vertical position state using the last height measurement
void NavUKF_core::ResetHeight(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetD = stateStruct.position.z;

    // write to the state vector
    stateStruct.position.z = -hgtMea;
    outputDataNew.position.z = stateStruct.position.z;
    outputDataDelayed.position.z = stateStruct.position.z;

    // reset the terrain state height
    if (onGround) {
        // assume vehicle is sitting on the ground
        terrainState = stateStruct.position.z + rngOnGnd;
    } else {
        // can make no assumption other than vehicle is not below ground level
        terrainState = MAX(stateStruct.position.z + rngOnGnd , terrainState);
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
    }
    vertCompFiltState.pos = stateStruct.position.z;

    // Calculate the position jump due to the reset
    posResetD = stateStruct.position.z - posResetD;

    posDResetCount++;

    // clear the timeout flags and counters
    hgtTimeout = false;
    lastHgtPassTime_ms = imuSampleTime_ms;

    // reset the corresponding covariances
    zeroStatesVarCov(9, 9);

    // set the variances to the measurement variance
    P[9][9] = posDownObsNoise;

    // Reset the vertical velocity state using GPS vertical velocity if we are airborne
    // Check that GPS vertical velocity data is available and can be used
    if (inFlight &&
        (gpsIsInUse || badIMUdata) &&
        frontend->sources.useVelZSource(AP_NavEKF_Source::SourceZ::GPS, core_index) &&
        gpsDataNew.have_vz &&
        (imuSampleTime_ms - gpsDataDelayed.time_ms < 500)) {
        stateStruct.velocity.z =  gpsDataNew.vel.z;
#if UKF_FEATURE_EXTERNAL_NAV
    } else if (inFlight && useExtNavVel && (activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV)) {
        stateStruct.velocity.z = extNavVelDelayed.vel.z;
#endif
    } else if (onGround) {
        stateStruct.velocity.z = 0.0f;
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.z = stateStruct.velocity.z;
    }
    outputDataNew.velocity.z = stateStruct.velocity.z;
    outputDataDelayed.velocity.z = stateStruct.velocity.z;
    vertCompFiltState.vel = outputDataNew.velocity.z;

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

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool NavUKF_core::resetHeightDatum(void)
{
    if (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER || !onGround) {
        // only allow resets when on the ground.
        // If using using rangefinder for height then never perform a
        // reset of the height datum
        return false;
    }
    // record the old height estimate
    ftype oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    dal.baro().update_calibration();

    // clear the baro data buffer
    storedBaro.reset();

    // reset the vertical position and velocity states
    stateStruct.position.z = 0.0f;
    stateStruct.velocity.z = 0.0f;
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
        storedOutput[i].velocity.z = stateStruct.velocity.z;
    }
    outputDataNew.position.z = outputDataDelayed.position.z = stateStruct.position.z;
    outputDataNew.velocity.z = outputDataDelayed.velocity.z = stateStruct.velocity.z;
    vertCompFiltState.vel = outputDataNew.velocity.z;

    // baroHgtOffset is a slow first-order filter (calcFiltBaroOffset)
    // tracking baroDataDelayed.hgt + position.z.  Post-reset baro
    // reads 0 and position.z is 0 so the steady-state offset is 0;
    // without this, hgtMea = baroDataDelayed.hgt - baroHgtOffset
    // would feed a non-zero observation into the EKF for the ~1 s
    // the filter takes to relax, producing a post-reset altitude
    // transient.
    baroHgtOffset = 0.0f;

    // adjust the height of the EKF origin so that the origin plus baro height before and after the reset is the same
    if (validOrigin) {
        if (!gpsGoodToAlign) {
            // if we don't have GPS lock then we shouldn't be doing a
            // resetHeightDatum, but if we do then the best option is
            // to maintain the old error
            EKF_origin.alt += (int32_t)(100.0f * oldHgt);
        } else {
            // if we have a good GPS lock then reset to the GPS
            // altitude. This ensures the reported AMSL alt from
            // getLLH() is equal to GPS altitude, while also ensuring
            // that the relative alt is zero
            EKF_origin.copy_alt_from(dal.gps().location());
        }
        ekfGpsRefHgt = (double)0.01 * (double)EKF_origin.alt;
    }

    // set the terrain state to zero (on ground). The adjustment for
    // frame height will get added in the later constraints
    terrainState = 0;

    return true;
}

/*
  correct GPS data for position offset of antenna phase centre relative to the IMU
 */
void NavUKF_core::CorrectGPSForAntennaOffset(gps_elements &gps_data) const
{
    // return immediately if already corrected
    if (gps_data.corrected) {
        return;
    }
    gps_data.corrected = true;

    const Vector3F posOffsetBody = dal.gps().get_antenna_offset(gps_data.sensor_idx).toftype() - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }

    // TODO use a filtered angular rate with a group delay that matches the GPS delay
    Vector3F angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
    Vector3F velOffsetBody = angRate % posOffsetBody;
    Vector3F velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
    gps_data.vel -= velOffsetEarth;

    Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
    Location::offset_latlng(gps_data.lat, gps_data.lng, -posOffsetEarth.x, -posOffsetEarth.y);
    gps_data.hgt += posOffsetEarth.z;
}

// correct external navigation earth-frame position using sensor body-frame offset
void NavUKF_core::CorrectExtNavForSensorOffset(ext_nav_elements &ext_nav_data)
{
    // return immediately if already corrected
    if (ext_nav_data.corrected) {
        return;
    }
    ext_nav_data.corrected = true;

    // external nav data is against the public_origin, so convert to offset from EKF_origin
    ext_nav_data.pos.xy() += EKF_origin.get_distance_NE_ftype(public_origin);

#if HAL_VISUALODOM_ENABLED
    const auto *visual_odom = dal.visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    const Vector3F posOffsetBody = visual_odom->get_pos_offset().toftype() - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }
    Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
    ext_nav_data.pos.x -= posOffsetEarth.x;
    ext_nav_data.pos.y -= posOffsetEarth.y;
    ext_nav_data.pos.z -= posOffsetEarth.z;
#endif
}

// correct external navigation earth-frame velocity using sensor body-frame offset
void NavUKF_core::CorrectExtNavVelForSensorOffset(ext_nav_vel_elements &ext_nav_vel_data) const
{
    // return immediately if already corrected
    if (ext_nav_vel_data.corrected) {
        return;
    }
    ext_nav_vel_data.corrected = true;

#if HAL_VISUALODOM_ENABLED
    const auto *visual_odom = dal.visualodom();
    if (visual_odom == nullptr) {
        return;
    }
    const Vector3F posOffsetBody = visual_odom->get_pos_offset().toftype() - accelPosOffset;
    if (posOffsetBody.is_zero()) {
        return;
    }
    // TODO use a filtered angular rate with a group delay that matches the sensor delay
    const Vector3F angRate = imuDataDelayed.delAng * (1.0/imuDataDelayed.delAngDT);
    ext_nav_vel_data.vel += get_vel_correction_for_sensor_offset(posOffsetBody, prevTnb, angRate);
#endif
}

// calculate velocity variance helper function
void NavUKF_core::CalculateVelInnovationsAndVariances(const Vector3F &velocity, ftype noise, ftype accel_scale, Vector3F &innovations, Vector3F &variances) const
{
    // innovations are latest estimate - latest observation
    innovations = stateStruct.velocity - velocity;

    const ftype obs_data_chk = sq(constrain_ftype(noise, 0.05, 5.0)) + sq(accel_scale * accNavMag);

    // calculate innovation variance.  velocity states start at index 4
    variances.x = P[4][4] + obs_data_chk;
    variances.y = P[5][5] + obs_data_chk;
    variances.z = P[6][6] + obs_data_chk;
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// select fusion of velocity, position and height measurements
void NavUKF_core::SelectVelPosFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !posVelFusionDelayed) {
        posVelFusionDelayed = true;
        return;
    } else {
        posVelFusionDelayed = false;
    }

#if UKF_FEATURE_EXTERNAL_NAV
    // Check for data at the fusion time horizon
    extNavDataToFuse = storedExtNav.recall(extNavDataDelayed, imuDataDelayed.time_ms);
    if (extNavDataToFuse) {
        CorrectExtNavForSensorOffset(extNavDataDelayed);
    }
    extNavVelToFuse = storedExtNavVel.recall(extNavVelDelayed, imuDataDelayed.time_ms);
    if (extNavVelToFuse) {
        CorrectExtNavVelForSensorOffset(extNavVelDelayed);

        // calculate innovations and variances for reporting purposes only
        CalculateVelInnovationsAndVariances(extNavVelDelayed.vel, extNavVelDelayed.err, frontend->extNavVelVarAccScale, extNavVelInnov, extNavVelVarInnov);

        // record time innovations were calculated (for timeout checks)
        extNavVelInnovTime_ms = dal.millis();
    }
#endif // UKF_FEATURE_EXTERNAL_NAV

    // Read GPS data from the sensor
    readGpsData();
    readGpsYawData();

    // get data that has now fallen behind the fusion time horizon
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms) && !waitingForGpsChecks;

    if (gpsDataToFuse) {
        CorrectGPSForAntennaOffset(gpsDataDelayed);
        // calculate innovations and variances for reporting purposes only
        CalculateVelInnovationsAndVariances(gpsDataDelayed.vel, frontend->_gpsHorizVelNoise.get(), frontend->gpsNEVelVarAccScale, gpsVelInnov, gpsVelVarInnov);
        // record time GPS data was retrieved from the buffer (for timeout checks)
        gpsRetrieveTime_ms = dal.millis();
    }

    // detect position source changes.  Trigger position reset if position source is valid
    const AP_NavEKF_Source::SourceXY posxy_source = frontend->sources.getPosXYSource(core_index);
    if (posxy_source != posxy_source_last) {
        posxy_source_reset = (posxy_source != AP_NavEKF_Source::SourceXY::NONE);
        posxy_source_last = posxy_source;
    }

    // initialise all possible data we may fuse
    fusePosData = false;
    fuseVelData = false;

    // Determine if we need to fuse position and velocity data on this time step
    if (gpsDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::GPS)) {

        // Don't fuse velocity data if GPS doesn't support it
        fuseVelData = frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS, core_index);
        fuseVelVertData = frontend->sources.useVelZSource(AP_NavEKF_Source::SourceZ::GPS, core_index) && useGpsVertVel;
        fusePosData = true;
#if UKF_FEATURE_EXTERNAL_NAV
        extNavUsedForPos = false;
#endif

        // copy corrected GPS data to observation vector
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
        }
        if (fuseVelVertData) {
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        const Location gpsloc{gpsDataDelayed.lat, gpsDataDelayed.lng, 0, Location::AltFrame::ABSOLUTE};
        const Vector2F posxy = EKF_origin.get_distance_NE_ftype(gpsloc);
        velPosObs[3] = posxy.x;
        velPosObs[4] = posxy.y;
#if UKF_FEATURE_EXTERNAL_NAV
    } else if (extNavDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::EXTNAV)) {
        // use external nav system for horizontal position
        extNavUsedForPos = true;
        fusePosData = true;
        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
#endif // UKF_FEATURE_EXTERNAL_NAV
    }

#if UKF_FEATURE_EXTERNAL_NAV
    // fuse external navigation velocity data if available
    // extNavVelDelayed is already corrected for sensor position
    if (extNavVelToFuse && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::EXTNAV, core_index)) {
        fuseVelData = true;
        velPosObs[0] = extNavVelDelayed.vel.x;
        velPosObs[1] = extNavVelDelayed.vel.y;
    }
    if (extNavVelToFuse && frontend->sources.useVelZSource(AP_NavEKF_Source::SourceZ::EXTNAV, core_index)) {
        fuseVelVertData = true;
        velPosObs[2] = extNavVelDelayed.vel.z;
    }
#endif

    // we have GPS data to fuse and a request to align the yaw using the GPS course
    if (gpsYawResetRequest) {
        realignYawGPS(false);
    }

    // Select height data to be fused from the available baro, range finder and GPS sources
    selectHeightForFusion();

    // if we are using GPS, check for a change in receiver and reset position and height
    if (gpsDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::GPS) && (gpsDataDelayed.sensor_idx != last_gps_idx || posxy_source_reset)) {
        // mark a source reset as consumed
        posxy_source_reset = false;

        // record the ID of the GPS that we are using for the reset
        last_gps_idx = gpsDataDelayed.sensor_idx;

        // reset the position to the GPS position
        const Location gpsloc{gpsDataDelayed.lat, gpsDataDelayed.lng, 0, Location::AltFrame::ABSOLUTE};
        const Vector2F posxy = EKF_origin.get_distance_NE_ftype(gpsloc);
        ResetPositionNE(posxy.x, posxy.y);

        // If we are also using GPS as the height reference, reset the height
        if (activeHgtSource == AP_NavEKF_Source::SourceZ::GPS) {
            ResetPositionD(-hgtMea);
        }
    }

#if UKF_FEATURE_EXTERNAL_NAV
    // check for external nav position reset
    if (extNavDataToFuse && (PV_AidingMode == AID_ABSOLUTE) && (posxy_source == AP_NavEKF_Source::SourceXY::EXTNAV) && (extNavDataDelayed.posReset || posxy_source_reset)) {
        // mark a source reset as consumed
        posxy_source_reset = false;
        ResetPositionNE(extNavDataDelayed.pos.x, extNavDataDelayed.pos.y);
        if (activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV) {
            ResetPositionD(-hgtMea);
        }
    }
#endif // UKF_FEATURE_EXTERNAL_NAV

    // If we are operating without any aiding, fuse in constant position of constant
    // velocity measurements to constrain tilt drift. This assumes a non-manoeuvring
    // vehicle. Do this to coincide with the height fusion.
    fusingStationaryZeroVel = false;

    if (fuseHgtData && PV_AidingMode == AID_NONE) {
        if (assume_zero_sideslip() && tiltAlignComplete && motorsArmed) {
            // handle special case where we are launching a FW aircraft without magnetometer
            fusePosData = false;
            velPosObs[0] = 0.0f;
            velPosObs[1] = 0.0f;
            velPosObs[2] = stateStruct.velocity.z;
            bool resetVelNE = !prevMotorsArmed;
            // reset states to stop launch accel causing tilt error
            if  (imuDataDelayed.delVel.x > 1.1f * GRAVITY_MSS * imuDataDelayed.delVelDT) {
                lastLaunchAccelTime_ms = imuSampleTime_ms;
                fuseVelData = false;
                fuseVelVertData = false;
                resetVelNE = true;
            } else if (lastLaunchAccelTime_ms != 0 && (imuSampleTime_ms - lastLaunchAccelTime_ms) < 10000) {
                fuseVelData = false;
                fuseVelVertData = false;
                resetVelNE = true;
            } else {
                fuseVelData = true;
                fuseVelVertData = true;
            }
            if (resetVelNE) {
                stateStruct.velocity.x = 0.0f;
                stateStruct.velocity.y = 0.0f;
            }
        } else {
            fusePosData = true;
            // When stationary on ground, fuse zero velocity
            // to constrain gyro bias and Z-axis accel bias learning. XY accel biases
            // remain unobservable until the vehicle accelerates and are separately
            // inhibited by dvelBiasAxisInhibit in CovariancePrediction.
            // Use onGroundNotMoving to avoid fusing zero velocity when the vehicle
            // is being moved (e.g. on a boat or carried by hand).
            const bool onGroundNotFlying = onGroundNotMoving;
            if (onGroundNotFlying && tiltAlignComplete) {
                fuseVelData = true;
                fusingStationaryZeroVel = true;
                velPosObs[0] = 0.0f;
                velPosObs[1] = 0.0f;
                velPosObs[2] = 0.0f;
            } else {
                fuseVelData = false;
            }
            velPosObs[3] = lastKnownPositionNE.x;
            velPosObs[4] = lastKnownPositionNE.y;
        }
    }

    // When in AID_RELATIVE or AID_ABSOLUTE mode but stationary on ground without velocity
    // aiding, fuse synthetic zero velocity to constrain gyro bias and Z-axis accel bias
    // learning. XY accel biases are unobservable on the ground and are inhibited by
    // dvelBiasAxisInhibit. Without this, configurations like optical flow where
    // PV_AidingMode is AID_RELATIVE but no velocity data is available when stationary
    // have no velocity observations at all, causing unchecked bias drift. The timeout
    // check on each velocity source ensures we only inject zero velocity when no real
    // sensor data is being fused. Use onGroundNotMoving to avoid injecting zero velocity
    // when the vehicle is being moved, and takeoff_expected for armed-on-ground.
    // Gate behind fuseHgtData to limit fusion rate to baro rate (~10Hz) and avoid
    // overconstraining the filter by fusing at IMU rate.
    const bool onGroundNotFlying = onGroundNotMoving;

    if (fuseHgtData && PV_AidingMode != AID_NONE && onGroundNotFlying) {
        // Check if we have recent velocity aiding from any source
        const uint32_t velAidTimeout_ms = 1000;
        const bool haveRecentGpsVel = (imuSampleTime_ms - lastVelPassTime_ms < velAidTimeout_ms);
#if UKF_FEATURE_OPTFLOW_FUSION
        const bool haveRecentFlowVel = (imuSampleTime_ms - prevFlowFuseTime_ms < velAidTimeout_ms);
#else
        const bool haveRecentFlowVel = false;
#endif
        const bool haveRecentBodyVel = (imuSampleTime_ms - prevBodyVelFuseTime_ms < velAidTimeout_ms);

        if (!haveRecentGpsVel && !haveRecentFlowVel && !haveRecentBodyVel) {
            // No velocity aiding available while stationary - fuse synthetic zero velocity
            // to constrain gyro bias and gravity-aligned accel bias
            fuseVelData = true;
            fusingStationaryZeroVel = true;
            velPosObs[0] = 0.0f;
            velPosObs[1] = 0.0f;
            velPosObs[2] = 0.0f;
        }
    }

    // perform fusion
    if (fuseVelData|| fuseVelVertData || fusePosData || fuseHgtData) {
        FuseVelPosNED();
        // clear the flags to prevent repeated fusion of the same data
        fuseVelData = false;
        fuseVelVertData = false;
        fuseHgtData = false;
        fusePosData = false;
    }
}

// fuse selected position, velocity and height measurements
void NavUKF_core::FuseVelPosNED()
{
    // declare variables used to control access to arrays
    bool fuseData[6] {};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    Vector6 R_OBS; // Measurement variances used for fusion
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only

    // perform sequential fusion of measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from sensors like GPS receivers; it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fuseVelVertData || fusePosData || fuseHgtData) {
        // estimate the velocity, horiz position and height measurement variances.
        // Use different errors if operating without external aiding using an assumed position or velocity of zero
        if (PV_AidingMode == AID_NONE) {
            if (fusingStationaryZeroVel) {
                // Synthetic zero velocity on ground: use 1 m/s noise (variance = 1 m^2/s^2).
                // This is tighter than _noaidHorizNoise (default 10 m/s) used when armed,
                // giving stronger velocity/bias convergence while we have high confidence
                // the vehicle is stationary. Not as tight as a real sensor since
                // the measurement is an assumption, not a physical observation.
                R_OBS[0] = sq(MIN(frontend->_noaidHorizNoise, 1.0f));
            } else if (tiltAlignComplete && motorsArmed) {
                // This is a compromise between corrections for gyro errors and reducing effect of manoeuvre accelerations on tilt estimate
                R_OBS[0] = sq(constrain_ftype(frontend->_noaidHorizNoise, 0.5f, 50.0f));
            } else {
                // Use a smaller value to give faster initial alignment
                R_OBS[0] = sq(0.5f);
            }
            R_OBS[1] = R_OBS[0];
            R_OBS[2] = R_OBS[0];
            R_OBS[3] = R_OBS[0];
            R_OBS[4] = R_OBS[0];
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];
        } else if (fusingStationaryZeroVel) {
            // Synthetic zero velocity in AID_RELATIVE or AID_ABSOLUTE mode when no
            // velocity sensor data is available (e.g. GPS configured but not yet locked,
            // or optical flow with no movement). Use 1 m/s noise for velocity — same
            // rationale as the AID_NONE case above. Position noise uses actual sensor
            // characteristics when available (extNav posErr, GPS accuracy), falling back
            // to _gpsHorizPosNoise as a conservative default since it is the only
            // position noise parameter available regardless of aiding source.
            R_OBS[0] = sq(1.0f);
            R_OBS[1] = R_OBS[0];
            R_OBS[2] = R_OBS[0];
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];
#if UKF_FEATURE_EXTERNAL_NAV
            if (extNavUsedForPos) {
                R_OBS[3] = sq(constrain_ftype(extNavDataDelayed.posErr, 0.01f, 10.0f));
            } else
#endif
            if (gpsPosAccuracy > 0.0f) {
                R_OBS[3] = sq(constrain_ftype(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
            } else {
                R_OBS[3] = sq(constrain_ftype(frontend->_gpsHorizPosNoise, 0.1f, 10.0f));
            }
            R_OBS[4] = R_OBS[3];
        } else {
#if UKF_FEATURE_EXTERNAL_NAV
            const bool extNavUsedForVel = extNavVelToFuse && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::EXTNAV, core_index);
            if (extNavUsedForVel) {
                R_OBS[2] = R_OBS[0] = sq(constrain_ftype(extNavVelDelayed.err, 0.05f, 50.0f));
            } else
#endif
            if (gpsSpdAccuracy > 0.0f) {
                // use GPS receivers reported speed accuracy if available and floor at value set by GPS velocity noise parameter
                R_OBS[0] = sq(constrain_ftype(gpsSpdAccuracy, frontend->_gpsHorizVelNoise, 50.0f));
                R_OBS[2] = sq(constrain_ftype(gpsSpdAccuracy, frontend->_gpsVertVelNoise, 50.0f));
            } else {
                // calculate additional error in GPS velocity caused by manoeuvring
                R_OBS[0] = sq(constrain_ftype(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
                R_OBS[2] = sq(constrain_ftype(frontend->_gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend->gpsDVelVarAccScale  * accNavMag);
            }
            R_OBS[1] = R_OBS[0];
            // Use GPS reported position accuracy if available and floor at value set by GPS position noise parameter
#if UKF_FEATURE_EXTERNAL_NAV
            if (extNavUsedForPos) {
                R_OBS[3] = sq(constrain_ftype(extNavDataDelayed.posErr, 0.01f, 100.0f));
            } else
#endif
            if (gpsPosAccuracy > 0.0f) {
                R_OBS[3] = sq(constrain_ftype(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
            } else {
                // calculate additional error in GPS position caused by manoeuvring
                const ftype posErr = frontend->gpsPosVarAccScale * accNavMag;
                R_OBS[3] = sq(constrain_ftype(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
            }
            R_OBS[4] = R_OBS[3];
            // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
            // For horizontal GPS velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPS performance
            // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
            ftype obs_data_chk;
#if UKF_FEATURE_EXTERNAL_NAV
            if (extNavUsedForVel) {
                obs_data_chk = sq(constrain_ftype(extNavVelDelayed.err, 0.05f, 5.0f)) + sq(frontend->extNavVelVarAccScale * accNavMag);
            } else
#endif
            {
                obs_data_chk = sq(constrain_ftype(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
            }
            R_OBS_DATA_CHECKS[0] = R_OBS_DATA_CHECKS[1] = R_OBS_DATA_CHECKS[2] = obs_data_chk;
        }
        R_OBS[5] = posDownObsNoise;
        for (uint8_t i=3; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];

        // if vertical GPS velocity data and an independent height source is being used, check to see if the GPS vertical velocity and altimeter
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        const bool fuse_gps_vz = frontend->sources.useVelZSource(AP_NavEKF_Source::SourceZ::GPS, core_index) && gpsDataDelayed.have_vz;
        if (fuse_gps_vz && fuseVelVertData && (frontend->sources.getPosZSource(core_index) != AP_NavEKF_Source::SourceZ::GPS)) {
            // calculate innovations for height and vertical GPS vel measurements
            const ftype hgtErr  = stateStruct.position.z - velPosObs[5];
            const ftype velDErr = stateStruct.velocity.z - velPosObs[2];
            // Check if they are the same sign and both more than 3-sigma out of bounds
            // Step the test threshold up in stages from 1 to 2 to 3 sigma after exiting
            // from a previous bad IMU event so that a subsequent error is caught more quickly.
            const uint32_t timeSinceLastBadIMU_ms = imuSampleTime_ms - badIMUdata_ms;
            float R_gain;
            if (timeSinceLastBadIMU_ms > (BAD_IMU_DATA_HOLD_MS * 2)) {
                R_gain = 9.0F;
            } else if  (timeSinceLastBadIMU_ms > ((BAD_IMU_DATA_HOLD_MS * 3) / 2)) {
                R_gain = 4.0F;
            } else {
                R_gain = 1.0F;
            }
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > R_gain * R_OBS[5]) && (sq(velDErr) >R_gain * R_OBS[2])) {
                badIMUdata_ms = imuSampleTime_ms;
            } else {
                goodIMUdata_ms = imuSampleTime_ms;
            }
            if (timeSinceLastBadIMU_ms < BAD_IMU_DATA_HOLD_MS) {
                badIMUdata = true;
                stateStruct.velocity.z = gpsDataDelayed.vel.z;
            } else {
                badIMUdata = false;
            }
        }

        // Test horizontal position measurements
        if (fusePosData) {
            innovVelPos[3] = stateStruct.position.x - velPosObs[3];
            innovVelPos[4] = stateStruct.position.y - velPosObs[4];
            varInnovVelPos[3] = P[7][7] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS_DATA_CHECKS[4];

            // Apply an innovation consistency threshold test
            // Don't allow test to fail if not navigating and using a constant position
            // assumption to constrain tilt errors because innovations can become large
            // due to vehicle motion.
            ftype maxPosInnov2 = sq(MAX(0.01 * (ftype)frontend->_gpsPosInnovGate, 1.0))*(varInnovVelPos[3] + varInnovVelPos[4]);

            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            bool posCheckPassed = false; // boolean true if position measurements have passed innovation consistency check
            if (posTestRatio < 1.0f || (PV_AidingMode == AID_NONE)) {
                posCheckPassed = true;
                lastGpsPosPassTime_ms = imuSampleTime_ms;
            } else if ((frontend->_gpsGlitchRadiusMax <= 0) && (PV_AidingMode != AID_NONE)) {
                // Handle the special case where the glitch radius parameter has been set to a non-positive number.
                // The innovation variance is increased to limit the state update to an amount corresponding
                // to a test ratio of 1.
                lastGpsPosPassTime_ms = imuSampleTime_ms;
                varInnovVelPos[3] *= posTestRatio;
                varInnovVelPos[4] *= posTestRatio;
                posCheckPassed = true;
                lastGpsPosPassTime_ms = imuSampleTime_ms;
            }

            // Use position data if healthy or timed out or bad IMU data
            // Always fuse data if bad IMU to prevent aliasing and clipping pulling the state estimate away
            // from the measurement un-opposed if test threshold is exceeded.
            if (posCheckPassed || posTimeout || badIMUdata) {
                // if timed out or outside the specified uncertainty radius, reset to the external sensor
                // if velocity drift is being constrained, dont reset until gps passes quality checks
                const bool posVarianceIsTooLarge = (frontend->_gpsGlitchRadiusMax > 0) && (P[8][8] + P[7][7]) > sq(ftype(frontend->_gpsGlitchRadiusMax));
                if ((posTimeout || posVarianceIsTooLarge) && (!velAiding || gpsGoodToAlign)) {
                    // reset the position to the current external sensor position
                    ResetPosition(resetDataSource::DEFAULT);

                    // Don't fuse the same data we have used to reset states.
                    fusePosData = false;

                    // Reset the position variances and corresponding covariances to a value that will pass the checks
                    zeroStatesVarCov(7, 8);
                    P[7][7] = sq(ftype(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[8][8] = P[7][7];

                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    posTestRatio = 0.0f;

                    // Reset velocity if it has timed out
                    if (velTimeout) {
                        ResetVelocity(resetDataSource::DEFAULT);

                        // Don't fuse the same data we have used to reset states.
                        fuseVelData = false;

                        // Reset the normalised innovation to avoid failing the bad fusion tests
                        velTestRatio = 0.0f;
                    }
                }
            } else {
                fusePosData = false;
            }
        }

        // Test velocity measurements
        if (fuseVelData) {
            uint8_t imax = 2;
            // Don't fuse vertical velocity observations if disabled in sources or not available
            const bool fuse_extnav_vz = frontend->sources.useVelZSource(AP_NavEKF_Source::SourceZ::EXTNAV, core_index) && useExtNavVel;
            if ((PV_AidingMode != AID_ABSOLUTE || !fuse_gps_vz) && !fuse_extnav_vz) {
                imax = 1;
            }

            // Apply an innovation consistency threshold test
            ftype innovVelSumSq = 0; // sum of squares of velocity innovations
            ftype varVelSum = 0; // sum of velocity innovation variances

            for (uint8_t i = 0; i<=imax; i++) {
                stateIndex   = i + 4;
                const float innovation = stateStruct.velocity[i] - velPosObs[i];
                innovVelSumSq += sq(innovation);
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                varVelSum += varInnovVelPos[i];
            }
            velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01 * (ftype)frontend->_gpsVelInnovGate, 1.0)));
            bool velCheckPassed = false; // boolean true if velocity measurements have passed innovation consistency checks
            if (velTestRatio < 1.0) {
                velCheckPassed = true;
                lastVelPassTime_ms = imuSampleTime_ms;
            } else if (frontend->_gpsGlitchRadiusMax <= 0) {
                // Handle the special case where the glitch radius parameter has been set to a non-positive number.
                // The innovation variance is increased to limit the state update to an amount corresponding
                // to a test ratio of 1.
                lastGpsPosPassTime_ms = imuSampleTime_ms;
                for (uint8_t i = 0; i<=imax; i++) {
                    varInnovVelPos[i] *= velTestRatio;
                }
                velCheckPassed = true;
                lastVelPassTime_ms = imuSampleTime_ms;
            }

            // Use velocity data if healthy, timed out or when IMU fault has been detected
            // Always fuse data if bad IMU to prevent aliasing and clipping pulling the state estimate away
            // from the measurement un-opposed if test threshold is exceeded.
            if (velCheckPassed || velTimeout || badIMUdata) {
                // If we are doing full aiding and velocity fusion times out, reset to the external sensor velocity
                if (PV_AidingMode == AID_ABSOLUTE && velTimeout) {
                    ResetVelocity(resetDataSource::DEFAULT);

                    // Don't fuse the same data we have used to reset states.
                    fuseVelData = false;

                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    velTestRatio = 0.0f;
                }
            } else {
                fuseVelData = false;
            }
        }

        // Test height measurements
        if (fuseHgtData) {
            // Calculate height innovations
            innovVelPos[5] = stateStruct.position.z - velPosObs[5];
            varInnovVelPos[5] = P[9][9] + R_OBS_DATA_CHECKS[5];

            // Calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01 * (ftype)frontend->_hgtInnovGate, 1.0)) * varInnovVelPos[5]);

            // When on ground we accept a larger test ratio to allow the filter to handle large switch on IMU
            // bias errors without rejecting the height sensor.
            const bool onGroundNotNavigating = (PV_AidingMode == AID_NONE) && onGround;
            const float maxTestRatio = onGroundNotNavigating ? 3.0f : 1.0f;
            bool hgtCheckPassed = false; // boolean true if height measurements have passed innovation consistency check
            if (hgtTestRatio < maxTestRatio) {
                hgtCheckPassed = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
            } else if ((frontend->_gpsGlitchRadiusMax <= 0) && !onGroundNotNavigating && (activeHgtSource == AP_NavEKF_Source::SourceZ::GPS)) {
                // Handle the special case where the glitch radius parameter has been set to a non-positive number.
                // The innovation variance is increased to limit the state update to an amount corresponding
                // to a test ratio of 1.
                lastGpsPosPassTime_ms = imuSampleTime_ms;
                varInnovVelPos[5] *= hgtTestRatio;
                hgtCheckPassed = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
            }

            // Use height data if innovation check passed or timed out or if bad IMU data
            // Always fuse data if bad IMU to prevent aliasing and clipping pulling the state estimate away
            // from the measurement un-opposed if test threshold is exceeded.
            if (hgtCheckPassed || hgtTimeout || badIMUdata) {
                // Calculate a filtered value to be used by pre-flight health checks
                // We need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
                if (onGround) {
                    ftype dtBaro = (imuSampleTime_ms - lastHgtPassTime_ms) * 1.0e-3;
                    const ftype hgtInnovFiltTC = 2.0;
                    ftype alpha = constrain_ftype(dtBaro/(dtBaro+hgtInnovFiltTC), 0.0, 1.0);
                    hgtInnovFiltState += (innovVelPos[5] - hgtInnovFiltState)*alpha;
                } else {
                    hgtInnovFiltState = 0.0f;
                }

                if (hgtTimeout) {
                    ResetHeight();

                    // Don't fuse the same data we have used to reset states.
                    fuseHgtData = false;
                }

            } else {
                fuseHgtData = false;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        if (fuseVelData) {
            fuseData[0] = true;
            fuseData[1] = true;
            if (fuseVelVertData || fusingStationaryZeroVel) {
                fuseData[2] = true;
            }
        }
        if (fusePosData) {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if (fuseHgtData) {
            fuseData[5] = true;
        }

        // fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 4 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                if (obsIndex <= 2) {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - velPosObs[obsIndex];
                    if (frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS, core_index)) {
                        R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                    }
                } else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    if (frontend->sources.getPosXYSource(core_index) == AP_NavEKF_Source::SourceXY::GPS) {
                        R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                    }
                } else if (obsIndex == 5) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    const ftype gndMaxBaroErr = MAX(frontend->_baroGndEffectDeadZone, 0.0);
                    const ftype gndBaroInnovFloor = -0.5;

                    if ((dal.get_touchdown_expected() || dal.get_takeoff_expected()) && activeHgtSource == AP_NavEKF_Source::SourceZ::BARO) {
                        // when baro positive pressure error due to ground effect is expected,
                        // floor the barometer innovation at gndBaroInnovFloor
                        // constrain the correction between 0 and gndBaroInnovFloor+gndMaxBaroErr
                        // this function looks like this:
                        //         |/
                        //---------|---------
                        //    ____/|
                        //   /     |
                        //  /      |
                        innovVelPos[5] += constrain_ftype(-innovVelPos[5]+gndBaroInnovFloor, 0.0f, gndBaroInnovFloor+gndMaxBaroErr);
                    }
                }

                uint32_t kalman_mask = (1<<10)-1; // values to calculate in Kfusion (others are set to zero)

                // inhibit delta angle bias state estimation by setting Kalman gains to zero
                if (!inhibitDelAngBiasStates) {
                    for (uint8_t i = 10; i<=12; i++) {
                        // Don't try to learn gyro bias if not aiding and the axis is
                        // less than 45 degrees from vertical because the bias is poorly observable
                        bool poorObservability = false;
                        if (PV_AidingMode == AID_NONE) {
                            const uint8_t axisIndex = i - 10;
                            if (axisIndex == 0) {
                                poorObservability = fabsF(prevTnb.a.z) > M_SQRT1_2;
                            } else if (axisIndex == 1) {
                                poorObservability = fabsF(prevTnb.b.z) > M_SQRT1_2;
                            } else {
                                poorObservability = fabsF(prevTnb.c.z) > M_SQRT1_2;
                            }
                        }
                        if (!poorObservability) {
                            kalman_mask |= (1<<i);
                        }
                    }
                }

                // Inhibit delta velocity bias state estimation by setting Kalman gains to zero
                // Don't use 'fake' horizontal measurements used to constrain attitude drift during
                // periods of non-aiding to learn bias as these can give incorrect esitmates.
                const bool horizInhibit = PV_AidingMode == AID_NONE && obsIndex != 2 && obsIndex != 5;
                if (!horizInhibit && !inhibitDelVelBiasStates && !badIMUdata) {
                    for (uint8_t i = 13; i<=15; i++) {
                        if (!dvelBiasAxisInhibit[i-13]) {
                            kalman_mask |= (1<<i);
                        }
                    }
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                if (!inhibitMagStates) {
                    kalman_mask |= (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21);
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                if (!inhibitWindStates && !treatWindStatesAsTruth) {
                    kalman_mask |= (1<<22) | (1<<23);
                }

                ut_obs.state_index = stateIndex;
                const ftype z_meas = statesArray[stateIndex] - innovVelPos[obsIndex];
                ftype innov_ut, var_ut;
                bool fault = ukfComputeUpdate(z_meas, R_OBS[obsIndex], UKFObs::State, kalman_mask, innov_ut, var_ut);
                if (!fault) {
                    varInnovVelPos[obsIndex] = var_ut;
                    fault = ukfApplyUpdate(innovVelPos[obsIndex], var_ut);
                }
                // record health status
                if (obsIndex == 0) {
                    faultStatus.bad_nvel = fault;
                } else if (obsIndex == 1) {
                    faultStatus.bad_evel = fault;
                } else if (obsIndex == 2) {
                    faultStatus.bad_dvel = fault;
                } else if (obsIndex == 3) {
                    faultStatus.bad_npos = fault;
                } else if (obsIndex == 4) {
                    faultStatus.bad_epos = fault;
                } else if (obsIndex == 5) {
                    faultStatus.bad_dpos = fault;
                }
            }
        }
    }
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// select the height measurement to be fused from the available baro, range finder and GPS sources
void NavUKF_core::selectHeightForFusion()
{
#if AP_RANGEFINDER_ENABLED
    // Read range finder data and check for new data in the buffer
    // This data is used by both height and optical flow fusion processing
    readRangeFinder();
    rangeDataToFuse = storedRange.recall(rangeDataDelayed,imuDataDelayed.time_ms);

    // correct range data for the body frame position offset relative to the IMU
    // the corrected reading is the reading that would have been taken if the sensor was
    // co-located with the IMU
    const auto *_rng = dal.rangefinder();
    if (_rng && rangeDataToFuse) {
        auto *sensor = _rng->get_backend(rangeDataDelayed.sensor_idx);
        if (sensor != nullptr) {
            Vector3F posOffsetBody = sensor->get_pos_offset().toftype() - accelPosOffset;
            if (!posOffsetBody.is_zero()) {
                Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
                rangeDataDelayed.rng += posOffsetEarth.z / prevTnb.c.z;
            }
        }
    }
#endif  // AP_RANGEFINDER_ENABLED

    // read baro height data from the sensor and check for new data in the buffer
    readBaroData();
    baroDataToFuse = storedBaro.recall(baroDataDelayed, imuDataDelayed.time_ms);

    bool rangeFinderDataIsFresh = (imuSampleTime_ms - rngValidMeaTime_ms < 500);
#if UKF_FEATURE_EXTERNAL_NAV
    const bool extNavDataIsFresh = (imuSampleTime_ms - extNavMeasTime_ms < 500);
#endif
    // select height source
    if ((frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::NONE)) {
        // user has specified no height sensor
        activeHgtSource = AP_NavEKF_Source::SourceZ::NONE;
#if AP_RANGEFINDER_ENABLED
    } else if ((frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::RANGEFINDER) && _rng && rangeFinderDataIsFresh) {
        // user has specified the range finder as a primary height source
        activeHgtSource = AP_NavEKF_Source::SourceZ::RANGEFINDER;
    } else if ((frontend->_useRngSwHgt > 0) && ((frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::BARO) || (frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::GPS)) && _rng && rangeFinderDataIsFresh) {
        // determine if we are above or below the height switch region
        const ftype rangeMaxUse = 1e-2 * (ftype)_rng->max_distance_orient(ROTATION_PITCH_270) * (ftype)frontend->_useRngSwHgt;
        bool aboveUpperSwHgt = (terrainState - stateStruct.position.z) > rangeMaxUse;
        bool belowLowerSwHgt = ((terrainState - stateStruct.position.z) < 0.7f * rangeMaxUse) && (imuSampleTime_ms - gndHgtValidTime_ms < 1000);

        // If the terrain height is consistent and we are moving slowly, then it can be
        // used as a height reference in combination with a range finder
        // apply a hysteresis to the speed check to prevent rapid switching
        bool dontTrustTerrain, trustTerrain;
        if (filterStatus.flags.horiz_vel) {
            // We can use the velocity estimate
            ftype horizSpeed = stateStruct.velocity.xy().length();
            dontTrustTerrain = (horizSpeed > frontend->_useRngSwSpd) || !terrainHgtStable;
            ftype trust_spd_trigger = MAX((frontend->_useRngSwSpd - 1.0f),(frontend->_useRngSwSpd * 0.5f));
            trustTerrain = (horizSpeed < trust_spd_trigger) && terrainHgtStable;
        } else {
            // We can't use the velocity estimate
            dontTrustTerrain = !terrainHgtStable;
            trustTerrain = terrainHgtStable;
        }

        /*
            * Switch between range finder and primary height source using height above ground and speed thresholds with
            * hysteresis to avoid rapid switching. Using range finder for height requires a consistent terrain height
            * which cannot be assumed if the vehicle is moving horizontally.
        */
        if ((aboveUpperSwHgt || dontTrustTerrain) && (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)) {
            // cannot trust terrain or range finder so stop using range finder height
            if (frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::BARO) {
                activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
            } else if (frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::GPS) {
                activeHgtSource = AP_NavEKF_Source::SourceZ::GPS;
            }
        } else if (belowLowerSwHgt && trustTerrain && (prevTnb.c.z >= 0.7f)) {
            // reliable terrain and range finder so start using range finder height
            activeHgtSource = AP_NavEKF_Source::SourceZ::RANGEFINDER;
        }
#endif
    } else if (frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::BARO) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
    } else if ((frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) < 500) && validOrigin && gpsAccuracyGood) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::GPS;
#if UKF_FEATURE_BEACON_FUSION
    } else if ((frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::BEACON) && validOrigin && rngBcn.goodToAlign) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::BEACON;
#endif
#if UKF_FEATURE_EXTERNAL_NAV
    } else if ((frontend->sources.getPosZSource(core_index) == AP_NavEKF_Source::SourceZ::EXTNAV) && extNavDataIsFresh) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::EXTNAV;
#endif
    }

    // Use Baro alt as a fallback if we lose range finder, GPS, external nav or Beacon
    bool lostRngHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER) && !rangeFinderDataIsFresh);
    bool lostGpsHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) > 2000 || !gpsAccuracyGoodForAltitude));
#if UKF_FEATURE_BEACON_FUSION
    bool lostRngBcnHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::BEACON) && ((imuSampleTime_ms - rngBcn.dataDelayed.time_ms) > 2000));
#endif
    bool fallback_to_baro =
        lostRngHgt
        || lostGpsHgt
#if UKF_FEATURE_BEACON_FUSION
        || lostRngBcnHgt
#endif
        ;
#if UKF_FEATURE_EXTERNAL_NAV
    bool lostExtNavHgt = ((activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV) && !extNavDataIsFresh);
    fallback_to_baro |= lostExtNavHgt;
#endif
    if (fallback_to_baro) {
        activeHgtSource = AP_NavEKF_Source::SourceZ::BARO;
    }

    // if there is new baro data to fuse, calculate filtered baro data required by other processes
    if (baroDataToFuse) {
        // calculate offset to baro data that enables us to switch to Baro height use during operation
        if (activeHgtSource != AP_NavEKF_Source::SourceZ::BARO) {
            calcFiltBaroOffset();
        }
        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!dal.get_takeoff_expected()) {
            const ftype gndHgtFiltTC = 0.5;
            const ftype dtBaro = frontend->hgtAvg_ms*1.0e-3;
            ftype alpha = constrain_ftype(dtBaro / (dtBaro+gndHgtFiltTC),0.0,1.0);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        }
    }

    // If we are not using GPS as the primary height sensor, correct EKF origin height so that
    // combined local NED position height and origin height remains consistent with the GPS altitude
    // This also enables the GPS height to be used as a backup height source
    if (gpsDataToFuse &&
            (((frontend->_originHgtMode & (1 << 0)) && (activeHgtSource == AP_NavEKF_Source::SourceZ::BARO)) ||
            ((frontend->_originHgtMode & (1 << 1)) && (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)))
            ) {
            correctEkfOriginHeight();
    }

    // Select the height measurement source
#if UKF_FEATURE_EXTERNAL_NAV
    if (extNavDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::EXTNAV)) {
        hgtMea = -extNavDataDelayed.pos.z;
        velPosObs[5] = -hgtMea;
        posDownObsNoise = sq(constrain_ftype(extNavDataDelayed.posErr, 0.1f, 10.0f));
        fuseHgtData = true;
    } else
#endif // UKF_FEATURE_EXTERNAL_NAV
        if (rangeDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)) {
        // using range finder data
        // correct for tilt using a flat earth model
        if (prevTnb.c.z >= 0.7) {
            // calculate height above ground
            hgtMea  = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
            // correct for terrain position relative to datum
            hgtMea -= terrainState;
            // correct sensor so that local position height adjusts to match GPS
            if (frontend->_originHgtMode & (1 << 1) && frontend->_originHgtMode & (1 << 2)) {
                // offset has to be applied to the measurement, not the NED origin
                hgtMea += (float)(ekfGpsRefHgt - 0.01 * (double)EKF_origin.alt);
            }
            velPosObs[5] = -hgtMea;
            // enable fusion
            fuseHgtData = true;
            // set the observation noise
            posDownObsNoise = sq(constrain_ftype(frontend->_rngNoise, 0.1f, 10.0f));
            // add uncertainty created by terrain gradient and vehicle tilt
            posDownObsNoise += sq(rangeDataDelayed.rng * frontend->_terrGradMax) * MAX(0.0f , (1.0f - sq(prevTnb.c.z)));
        } else {
            // disable fusion if tilted too far
            fuseHgtData = false;
        }
    } else if (gpsDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::GPS)) {
        // using GPS data
        hgtMea = gpsDataDelayed.hgt;
        velPosObs[5] = -hgtMea;
        // enable fusion
        fuseHgtData = true;
        // set the observation noise using receiver reported accuracy or the horizontal noise scaled for typical VDOP/HDOP ratio
        if (gpsHgtAccuracy > 0.0f) {
            posDownObsNoise = sq(constrain_ftype(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise, 100.0f));
        } else {
            posDownObsNoise = sq(constrain_ftype(1.5f * frontend->_gpsHorizPosNoise, 0.1f, 10.0f));
        }
    } else if (baroDataToFuse && (activeHgtSource == AP_NavEKF_Source::SourceZ::BARO)) {
        // using Baro data
        hgtMea = baroDataDelayed.hgt - baroHgtOffset;
        // correct sensor so that local position height adjusts to match GPS
        if (frontend->_originHgtMode & (1 << 0) && frontend->_originHgtMode & (1 << 2)) {
            hgtMea += (float)(ekfGpsRefHgt - 0.01 * (double)EKF_origin.alt);
        }
        // enable fusion
        fuseHgtData = true;
        // set the observation noise
        posDownObsNoise = sq(constrain_ftype(frontend->_baroAltNoise, 0.01f, 100.0f));
        // reduce weighting (increase observation noise) on baro if we are likely to be experiencing rotor wash ground interaction
        if (dal.get_takeoff_expected() || dal.get_touchdown_expected()) {
            posDownObsNoise *= frontend->gndEffectBaroScaler;
        }
        velPosObs[5] = -hgtMea;
    } else if ((activeHgtSource == AP_NavEKF_Source::SourceZ::NONE && imuSampleTime_ms - lastHgtPassTime_ms > 70)) {
        // fuse a constant height of 0 at 14 Hz
        hgtMea = 0.0f;
        fuseHgtData = true;
        velPosObs[5] = -hgtMea;
        if (onGround) {
            // use a typical vertical positoin observation noise when not flying for faster IMU delta velocity bias estimation
            posDownObsNoise = sq(2.0f);
        } else {
            // alow a larger value when flying to accomodate vertical maneouvres
            posDownObsNoise = sq(constrain_ftype(frontend->_baroAltNoise, 2.0f, 100.0f));
        }
    } else {
        fuseHgtData = false;
    }

    // detect changes in source and reset height
    if ((activeHgtSource != prevHgtSource) && fuseHgtData) {
        prevHgtSource = activeHgtSource;
        ResetPositionD(-hgtMea);
    }

    // If we haven't fused height data for a while or have bad IMU data, then declare the height data as being timed out
    // set height timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime_ms = ((useGpsVertVel || useExtNavVel) && !velTimeout) ? frontend->hgtRetryTimeMode0_ms : frontend->hgtRetryTimeMode12_ms;
    if (imuSampleTime_ms - lastHgtPassTime_ms > hgtRetryTime_ms ||
        (badIMUdata &&
        (imuSampleTime_ms - goodIMUdata_ms > BAD_IMU_DATA_TIMEOUT_MS))) {
        hgtTimeout = true;
    } else {
        hgtTimeout = false;
    }
}

#if UKF_FEATURE_BODY_ODOM
/*
 * Fuse body frame velocity measurements using an unscented transform.
*/
void NavUKF_core::FuseBodyVel()
{
    const UKFObs body_obs[3] = { UKFObs::BodyVelX, UKFObs::BodyVelY, UKFObs::BodyVelZ };
    ut_obs.pos_offset_body = bodyOdmDataDelayed.body_offset - accelPosOffset;
    const ftype R_VEL = sq(bodyOdmDataDelayed.velErr);

    for (uint8_t obsIndex = 0; obsIndex <= 2; obsIndex++) {

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

        const ftype z_meas = bodyOdmDataDelayed.vel[obsIndex];
        if (ukfComputeUpdate(z_meas, R_VEL, body_obs[obsIndex], kalman_mask,
                             innovBodyVel[obsIndex], varInnovBodyVel[obsIndex])) {
            if (obsIndex == 0) {
                faultStatus.bad_xvel = true;
            } else if (obsIndex == 1) {
                faultStatus.bad_yvel = true;
            } else {
                faultStatus.bad_zvel = true;
            }
            return;
        }
        if (obsIndex == 0) {
            faultStatus.bad_xvel = false;
        } else if (obsIndex == 1) {
            faultStatus.bad_yvel = false;
        } else {
            faultStatus.bad_zvel = false;
        }

        bodyVelTestRatio[obsIndex] = sq(innovBodyVel[obsIndex]) / (sq(5.0f) * varInnovBodyVel[obsIndex]);
        if (bodyVelTestRatio[obsIndex] < 1.0f) {
            prevBodyVelFuseTime_ms = imuSampleTime_ms;
            if (!bodyVelFusionActive) {
                bodyVelFusionActive = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UKF IMU%u fusing odometry",(unsigned)imu_index);
            }
            if (ukfApplyUpdate(innovBodyVel[obsIndex], varInnovBodyVel[obsIndex])) {
                if (obsIndex == 0) {
                    faultStatus.bad_xvel = true;
                } else if (obsIndex == 1) {
                    faultStatus.bad_yvel = true;
                } else {
                    faultStatus.bad_zvel = true;
                }
            }
        }
    }
}

#endif // UKF_FEATURE_BODY_ODOM

#if UKF_FEATURE_BODY_ODOM
// select fusion of body odometry measurements
void NavUKF_core::SelectBodyOdomFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && (dtIMUavg < 0.005f) && !bodyVelFusionDelayed) {
        bodyVelFusionDelayed = true;
        return;
    } else {
        bodyVelFusionDelayed = false;
    }

    // Check for body odometry data (aka visual position delta) at the fusion time horizon
    const bool bodyOdomDataToFuse = storedBodyOdm.recall(bodyOdmDataDelayed, imuDataDelayed.time_ms);
    if (bodyOdomDataToFuse && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::EXTNAV, core_index)) {

        // Fuse data into the main filter
        FuseBodyVel();
    }

    // Check for wheel encoder data at the fusion time horizon
    const bool wheelOdomDataToFuse = storedWheelOdm.recall(wheelOdmDataDelayed, imuDataDelayed.time_ms);
    if (wheelOdomDataToFuse && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::WHEEL_ENCODER, core_index)) {

        // check if the delta time is too small to calculate a velocity
        if (wheelOdmDataDelayed.delTime > EKF_TARGET_DT) {

            // get the forward velocity
            ftype fwdSpd = wheelOdmDataDelayed.delAng * wheelOdmDataDelayed.radius * (1.0f / wheelOdmDataDelayed.delTime);

            // get the unit vector from the projection of the X axis onto the horizontal
            Vector3F unitVec;
            unitVec.x = prevTnb.a.x;
            unitVec.y = prevTnb.a.y;
            unitVec.z = 0.0f;
            unitVec.normalize();

            // multiply by forward speed to get velocity vector measured by wheel encoders
            Vector3F velNED = unitVec * fwdSpd;

            // This is a hack to enable use of the existing body frame velocity fusion method
            // TODO write a dedicated observation model for wheel encoders
            bodyOdmDataDelayed.vel = prevTnb * velNED;
            bodyOdmDataDelayed.body_offset = wheelOdmDataDelayed.hub_offset;
            bodyOdmDataDelayed.velErr = frontend->_wencOdmVelErr.get();

            // Fuse data into the main filter
            FuseBodyVel();
        }
    }
}
#endif // UKF_FEATURE_BODY_ODOM
