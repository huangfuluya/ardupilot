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
  support for Pixhawk6X used as a DroneCAN External AHRS source
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

struct uavcan_equipment_ahrs_Solution;
struct uavcan_equipment_ahrs_RawIMU;

class AP_ExternalAHRS_Pixhawk6X : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_Pixhawk6X(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // DroneCAN callbacks drive this backend; update() is a no-op
    void update() override {}

    // Get model/type name
    const char* get_name() const override {
        return "Pixhawk6X";
    }

    // register DroneCAN message subscriptions
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

protected:
    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:
    // timestamps of last received messages (ms)
    uint32_t last_att_ms;
    uint32_t last_imu_ms;

    // singleton for use in static DroneCAN callbacks
    static AP_ExternalAHRS_Pixhawk6X *_singleton;

    // DroneCAN message handlers
    static void handle_solution(AP_DroneCAN *ap_dronecan, const CanardRxTransfer &transfer,
                                const uavcan_equipment_ahrs_Solution &msg);
    static void handle_rawimu(AP_DroneCAN *ap_dronecan, const CanardRxTransfer &transfer,
                               const uavcan_equipment_ahrs_RawIMU &msg);
};

#endif  // AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED
