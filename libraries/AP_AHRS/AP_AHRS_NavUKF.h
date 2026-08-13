#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  shim AP_NavUKF into AP_AHRS_Backend
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_NAVUKF_ENABLED

#include <AP_NavUKF/AP_NavUKF.h>
#include "AP_AHRS_Backend.h"

class AP_AHRS_NavUKF : public AP_AHRS_Backend {
public:

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_NavUKF);
    AP_AHRS_NavUKF() {}

    const char *shortname() const override { return "UKF"; }

    void reset_gyro_drift() override { UKF.resetGyroBias(); }

    void update() override;

    void get_results(Estimates &results) override;
    void reset() override {
        if (!started) {
            return;
        }
        started = UKF.InitialiseFilter();
    }

    bool get_origin(Location &ret) const override {
        return UKF.getOriginLLH(ret);
    }
    bool set_origin(const Location &loc) override {
        return UKF.setOriginLLH(loc);
    }

    bool            use_compass() override {
        return UKF.use_compass();
    }

    void resetHeightDatum(void) override {
        UKF.resetHeightDatum();
    }
    void request_yaw_reset() override {
        UKF.requestYawReset();
    }
    void check_lane_switch() override {
        UKF.checkLaneSwitch();
    }

    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    void get_control_limits(float &ekfGndSpdLimit, float &controlScaleXY) const override {
        return UKF.getEkfControlLimits(ekfGndSpdLimit, controlScaleXY);
    }

    // return the innovations for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override {
        return UKF.getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }

    // this is out here so parameters can be poked into it
    static NavUKF UKF;

    bool start();
    bool started;
    uint32_t start_time_ms;  // timer used to delay starting the filter

    // a counter which is incremented each time the primary core changes:
    AP_AHRS_ResetCounter<int8_t> attitude_reset_tracker;

    AP_AHRS_ResetCounter<uint16_t> yaw_reset_tracker;
    AP_AHRS_ResetCounter<uint16_t> position_NE_reset_tracker;
    AP_AHRS_ResetCounter<uint16_t> position_D_reset_tracker;
};

#endif  // AP_AHRS_NAVUKF_ENABLED
