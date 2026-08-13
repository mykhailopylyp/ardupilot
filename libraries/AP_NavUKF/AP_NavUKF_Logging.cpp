#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AP_NavUKF.h"
#include "AP_NavUKF_core.h"

#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_DAL/AP_DAL.h>

#pragma GCC diagnostic ignored "-Wnarrowing"

void NavUKF_core::Log_Write_UKF1(uint64_t time_us) const
{
    // Write first EKF packet
    Vector3f euler;
    Vector2p posNE;
    postype_t posD;
    Vector3f velNED;
    Vector3f gyroBias;
    float posDownDeriv;
    Location originLLH;
    getEulerAngles(euler);
    getVelNED(velNED);
    getPosNE(posNE);
    getPosD(posD);
    getGyroBias(gyroBias);
    posDownDeriv = getPosDownDerivative();
    if (!getOriginLLH(originLLH)) {
        originLLH.alt = 0;
    }
    const struct log_UKF1 pkt{
        LOG_PACKET_HEADER_INIT(LOG_UKF1_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        velN    : (float)(velNED.x), // velocity North (m/s)
        velE    : (float)(velNED.y), // velocity East (m/s)
        velD    : (float)(velNED.z), // velocity Down (m/s)
        posD_dot : (float)(posDownDeriv), // first derivative of down position
        posN    : (float)(posNE.x), // metres North
        posE    : (float)(posNE.y), // metres East
        posD    : (float)(posD), // metres Down
        gyrX    : (int16_t)(100*degrees(gyroBias.x)), // cd/sec, displayed as deg/sec due to format string
        gyrY    : (int16_t)(100*degrees(gyroBias.y)), // cd/sec, displayed as deg/sec due to format string
        gyrZ    : (int16_t)(100*degrees(gyroBias.z)), // cd/sec, displayed as deg/sec due to format string
        originHgt : originLLH.alt // WGS-84 altitude of EKF origin in cm
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void NavUKF_core::Log_Write_UKF2(uint64_t time_us) const
{
    // Write second EKF packet
    Vector3f accelBias;
    Vector3f wind;
    Vector3f magNED;
    Vector3f magXYZ;
    getAccelBias(accelBias);
    getWind(wind);
    getMagNED(magNED);
    getMagXYZ(magXYZ);
    Vector2f dragInnov;
    float betaInnov = 0;
    getSynthAirDataInnovations(dragInnov, betaInnov);
    const struct log_UKF2 pkt2{
        LOG_PACKET_HEADER_INIT(LOG_UKF2_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        accBiasX  : (int16_t)(100*accelBias.x),
        accBiasY  : (int16_t)(100*accelBias.y),
        accBiasZ  : (int16_t)(100*accelBias.z),
        windN   : (int16_t)(100*wind.x),
        windE   : (int16_t)(100*wind.y),
        magN    : (int16_t)(magNED.x),
        magE    : (int16_t)(magNED.y),
        magD    : (int16_t)(magNED.z),
        magX    : (int16_t)(magXYZ.x),
        magY    : (int16_t)(magXYZ.y),
        magZ    : (int16_t)(magXYZ.z),
        innovDragX    : dragInnov.x,
        innovDragY    : dragInnov.y,
        innovSideslip : betaInnov
    };
    AP::logger().WriteBlock(&pkt2, sizeof(pkt2));
}

void NavUKF_core::Log_Write_UKFS(uint64_t time_us) const
{
    // Write sensor selection EKF packet
    const struct log_UKFS pkt {
        LOG_PACKET_HEADER_INIT(LOG_UKFS_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        mag_index      : magSelectIndex,
        baro_index     : selected_baro,
        gps_index      : selected_gps,
        airspeed_index : getActiveAirspeed(),
        source_set     : frontend->sources.getPosVelYawSourceSet(),
        gps_good_to_align : gpsGoodToAlign,
        wait_for_gps_checks : waitingForGpsChecks,
        mag_fusion: (uint8_t) magFusionSel
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void NavUKF_core::Log_Write_UKF3(uint64_t time_us) const
{
    // Write third EKF packet
    Vector3f velInnov;
    Vector3f posInnov;
    Vector3f magInnov;
    float tasInnov = 0;
    float yawInnov = 0;
    getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    const struct log_UKF3 pkt3{
        LOG_PACKET_HEADER_INIT(LOG_UKF3_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        innovVN : (int16_t)(100*velInnov.x),
        innovVE : (int16_t)(100*velInnov.y),
        innovVD : (int16_t)(100*velInnov.z),
        innovPN : (int16_t)(100*posInnov.x),
        innovPE : (int16_t)(100*posInnov.y),
        innovPD : (int16_t)(100*posInnov.z),
        innovMX : (int16_t)(magInnov.x),
        innovMY : (int16_t)(magInnov.y),
        innovMZ : (int16_t)(magInnov.z),
        innovYaw : (int16_t)(100*degrees(yawInnov)),
        innovVT : (int16_t)(100*tasInnov),
        rerr : frontend->coreRelativeErrors[core_index],
        errorScore : frontend->coreErrorScores[core_index]
    };
    AP::logger().WriteBlock(&pkt3, sizeof(pkt3));
}

void NavUKF_core::Log_Write_UKF4(uint64_t time_us) const
{
    // Write fourth EKF packet
    float velVar = 0;
    float posVar = 0;
    float hgtVar = 0;
    Vector3f magVar;
    float tasVar = 0;
    uint16_t _faultStatus=0;
    Vector2f offset;
    const uint8_t timeoutStatus =
        posTimeout<<0 |
        velTimeout<<1 |
        hgtTimeout<<2 |
        magTimeout<<3 |
        tasTimeout<<4 |
        dragTimeout<<5;

    nav_filter_status solutionStatus {};
    getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    float tempVar = fmaxF(fmaxF(magVar.x,magVar.y),magVar.z);
    getFilterFaults(_faultStatus);
    getFilterStatus(solutionStatus);
    const struct log_UKF4 pkt4{
        LOG_PACKET_HEADER_INIT(LOG_UKF4_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        sqrtvarV : (int16_t)(100*velVar),
        sqrtvarP : (int16_t)(100*posVar),
        sqrtvarH : (int16_t)(100*hgtVar),
        sqrtvarM : (int16_t)(100*tempVar),
        sqrtvarVT : (int16_t)(100*tasVar),
        tiltErr : sqrtF(MAX(tiltErrorVariance,0.0f)),  // estimated 1-sigma tilt error in radians
        offsetNorth : offset.x,
        offsetEast : offset.y,
        faults : _faultStatus,
        timeouts : timeoutStatus,
        solution : solutionStatus.value,
        gps : gpsCheckStatus.value,
        primary : frontend->getPrimaryCoreIndex()
    };
    AP::logger().WriteBlock(&pkt4, sizeof(pkt4));
}


void NavUKF_core::Log_Write_UKF5(uint64_t time_us) const
{
    // UKF5 logging omitted to conserve LogMessages IDs
    (void)time_us;
}

void NavUKF_core::Log_Write_Quaternion(uint64_t time_us) const
{
    // log quaternion
    Quaternion quat;
    getQuaternion( quat);
    const struct log_UKFQ pktq1{
        LOG_PACKET_HEADER_INIT(LOG_UKFQ_MSG),
        time_us : time_us,
        core    : DAL_CORE(core_index),
        q1 : quat.q1,
        q2 : quat.q2,
        q3 : quat.q3,
        q4 : quat.q4
    };
    AP::logger().WriteBlock(&pktq1, sizeof(pktq1));
}

#if UKF_FEATURE_BEACON_FUSION
// logs beacon information, one beacon per call
void NavUKF_core::Log_Write_Beacon(uint64_t time_us)
{
    (void)time_us;
}
#endif  // UKF_FEATURE_BEACON_FUSION

#if UKF_FEATURE_BODY_ODOM
void NavUKF_core::Log_Write_BodyOdom(uint64_t time_us)
{
    (void)time_us;
}
#endif

void NavUKF_core::Log_Write_State_Variances(uint64_t time_us)
{
    (void)time_us;
}

void NavUKF::Log_Write()
{
    // only log if enabled
    if (activeCores() <= 0) {
        return;
    }
    if (lastLogWrite_us == imuSampleTime_us) {
        // vehicle is doubling up on logging
        return;
    }
    lastLogWrite_us = imuSampleTime_us;

    uint64_t time_us = AP::dal().micros64();

    for (uint8_t i=0; i<activeCores(); i++) {
        core[i].Log_Write(time_us);
    }

    AP::dal().start_frame(AP_DAL::FrameType::LogWriteEKF3);
}

void NavUKF_core::Log_Write(uint64_t time_us)
{
    const auto level = frontend->_log_level;
    if (level == NavUKF::LogLevel::NONE) {  // no logging from UKF_LOG_LEVEL param
        return;
    }
    Log_Write_UKF4(time_us);
    if (level == NavUKF::LogLevel::UKF4) {  // only log UKF4 scaled innovations
        return;
    }
    Log_Write_GSF(time_us);
    if (level == NavUKF::LogLevel::UKF4_GSF) {  // only log UKF4 scaled innovations and GSF, otherwise log everything
        return;
    }
    // note that several of these functions exit-early if they're not
    // attempting to log the primary core.
    Log_Write_UKF1(time_us);
    Log_Write_UKF2(time_us);
    Log_Write_UKF3(time_us);

    Log_Write_UKFS(time_us);
    Log_Write_Quaternion(time_us);


#if UKF_FEATURE_BEACON_FUSION
    // write range beacon fusion debug packet if the range value is non-zero
    Log_Write_Beacon(time_us);
#endif

#if UKF_FEATURE_BODY_ODOM
    // write debug data for body frame odometry fusion
    Log_Write_BodyOdom(time_us);
#endif

    // log state variances every 0.49s
    Log_Write_State_Variances(time_us);

    Log_Write_Timing(time_us);
}

void NavUKF_core::Log_Write_Timing(uint64_t time_us)
{
    (void)time_us;
}

void NavUKF_core::Log_Write_GSF(uint64_t time_us)
{
    (void)time_us;
}

#endif  // HAL_LOGGING_ENABLED
