#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_AHRS/AP_AHRS_config.h>

#define LOG_IDS_FROM_NAVUKF \
    LOG_UKF1_MSG, \
    LOG_UKF2_MSG, \
    LOG_UKF3_MSG, \
    LOG_UKF4_MSG, \
    LOG_UKFQ_MSG, \
    LOG_UKFS_MSG

struct PACKED log_UKF1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float velN;
    float velE;
    float velD;
    float posD_dot;
    float posN;
    float posE;
    float posD;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
    int32_t originHgt;
};

struct PACKED log_UKF2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t accBiasX;
    int16_t accBiasY;
    int16_t accBiasZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    float innovDragX;
    float innovDragY;
    float innovSideslip;
};

struct PACKED log_UKF3 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovYaw;
    int16_t innovVT;
    float rerr;
    float errorScore;
};

struct PACKED log_UKF4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    int16_t sqrtvarV;
    int16_t sqrtvarP;
    int16_t sqrtvarH;
    int16_t sqrtvarM;
    int16_t sqrtvarVT;
    float tiltErr;
    float offsetNorth;
    float offsetEast;
    uint16_t faults;
    uint8_t timeouts;
    uint32_t solution;
    uint16_t gps;
    int8_t primary;
};

struct PACKED log_UKFQ {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    float q1;
    float q2;
    float q3;
    float q4;
};

struct PACKED log_UKFS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t core;
    uint8_t mag_index;
    uint8_t baro_index;
    uint8_t gps_index;
    uint8_t airspeed_index;
    uint8_t source_set;
    uint8_t gps_good_to_align;
    uint8_t wait_for_gps_checks;
    uint8_t mag_fusion;
};

#if HAL_NAVUKF_AVAILABLE
#define LOG_STRUCTURE_FROM_NAVUKF \
    { LOG_UKF1_MSG, sizeof(log_UKF1), \
      "UKF1","QBccCfffffffccce","TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH", "s#ddhnnnnmmmkkkm", "F-BBB0000000BBBB" , true }, \
    { LOG_UKF2_MSG, sizeof(log_UKF2), \
      "UKF2","QBccccchhhhhhfff","TimeUS,C,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ,IDX,IDY,IS", "s#---nnGGGGGGoor", "F----BBCCCCCC000" , true }, \
    { LOG_UKF3_MSG, sizeof(log_UKF3), \
      "UKF3","QBcccccchhhccff","TimeUS,C,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,IVT,RErr,ErSc", "s#nnnmmmGGGd?--", "F-BBBBBBCCCBB00" , true }, \
    { LOG_UKF4_MSG, sizeof(log_UKF4), \
      "UKF4","QBcccccfffHBIHb","TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI", "s#------mm-----", "F-------??-----" , true }, \
    { LOG_UKFQ_MSG, sizeof(log_UKFQ), "UKFQ", "QBffff", "TimeUS,C,Q1,Q2,Q3,Q4", "s#????", "F-????" , true }, \
    { LOG_UKFS_MSG, sizeof(log_UKFS), \
      "UKFS","QBBBBBBBBB","TimeUS,C,MI,BI,GI,AI,SS,GPS_GTA,GPS_CHK_WAIT,MAG_FUSION", "s#--------", "F---------" , true },
#else
  #define LOG_STRUCTURE_FROM_NAVUKF
#endif
