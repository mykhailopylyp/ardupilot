/*
  feature selection for NavUKF
 */

#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Beacon/AP_Beacon_config.h>
#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow_config.h>

// define for when to include all features
#define UKF_FEATURE_ALL APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) || APM_BUILD_TYPE(APM_BUILD_Replay)

// body odomotry (which includes wheel encoding) on rover or 2M boards
#ifndef UKF_FEATURE_BODY_ODOM
#define UKF_FEATURE_BODY_ODOM UKF_FEATURE_ALL || APM_BUILD_TYPE(APM_BUILD_Rover) || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

// external navigation on 2M boards
#ifndef UKF_FEATURE_EXTERNAL_NAV
#define UKF_FEATURE_EXTERNAL_NAV UKF_FEATURE_ALL || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

// drag fusion on 2M boards
#ifndef UKF_FEATURE_DRAG_FUSION
#define UKF_FEATURE_DRAG_FUSION UKF_FEATURE_ALL || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

// Beacon Fusion if beacon data available
#ifndef UKF_FEATURE_BEACON_FUSION
#define UKF_FEATURE_BEACON_FUSION AP_BEACON_ENABLED
#endif

#ifndef UKF_FEATURE_POSITION_RESET
#define UKF_FEATURE_POSITION_RESET UKF_FEATURE_ALL || AP_AHRS_POSITION_RESET_ENABLED
#endif

// rangefinder measurements if available
#ifndef UKF_FEATURE_RANGEFINDER_MEASUREMENTS
#define UKF_FEATURE_RANGEFINDER_MEASUREMENTS AP_RANGEFINDER_ENABLED
#endif

// Flow Fusion if Flow data available
#ifndef UKF_FEATURE_OPTFLOW_FUSION
#define UKF_FEATURE_OPTFLOW_FUSION HAL_NAVUKF_AVAILABLE && AP_OPTICALFLOW_ENABLED
#endif

// Optical flow using SRTM terrain data
#ifndef UKF_FEATURE_OPTFLOW_SRTM
#define UKF_FEATURE_OPTFLOW_SRTM UKF_FEATURE_OPTFLOW_FUSION
#endif

// IMU-aided AGL Kalman filter decoupled from the main filter's vertical position state
#ifndef UKF_FEATURE_OPTFLOW_AGL_KF
#define UKF_FEATURE_OPTFLOW_AGL_KF UKF_FEATURE_OPTFLOW_FUSION
#endif
