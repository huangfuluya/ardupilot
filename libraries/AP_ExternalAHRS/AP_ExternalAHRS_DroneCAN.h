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
  support for DroneCAN connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_DRONECAN_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_ExternalAHRS_DroneCAN : public AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_DroneCAN(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled (DroneCAN does not use serial)
    int8_t get_port(void) const override { return -1; }

    // Get model/type name
    const char* get_name() const override { return "DroneCAN"; }

    // health status
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

    // filter diagnostics
    void get_filter_status(nav_filter_status &status) const override;

    // update - called from main loop, no-op as data arrives via CAN callbacks
    void update() override {}

    // number of GPS sensors
    uint8_t num_gps_sensors(void) const override { return 1; }

    // subscribe to DroneCAN messages; called from AP_DroneCAN::init()
    static bool subscribe_msgs(AP_DroneCAN *ap_dronecan);

    // message trampolines
    static void handle_gnss_fix2_trampoline(AP_DroneCAN *ap_dronecan,
                                            const CanardRxTransfer &transfer,
                                            const uavcan_equipment_gnss_Fix2 &msg);
    static void handle_ahrs_solution_trampoline(AP_DroneCAN *ap_dronecan,
                                                const CanardRxTransfer &transfer,
                                                const uavcan_equipment_ahrs_Solution &msg);
    static void handle_raw_imu_trampoline(AP_DroneCAN *ap_dronecan,
                                          const CanardRxTransfer &transfer,
                                          const uavcan_equipment_ahrs_RawIMU &msg);
    static void handle_nav_solution_trampoline(AP_DroneCAN *ap_dronecan,
                                               const CanardRxTransfer &transfer,
                                               const uavcan_navigation_GlobalNavigationSolution &msg);
    static void handle_static_pressure_trampoline(AP_DroneCAN *ap_dronecan,
                                                  const CanardRxTransfer &transfer,
                                                  const uavcan_equipment_air_data_StaticPressure &msg);
    static void handle_static_temperature_trampoline(AP_DroneCAN *ap_dronecan,
                                                     const CanardRxTransfer &transfer,
                                                     const uavcan_equipment_air_data_StaticTemperature &msg);

private:
    // message handlers
    void handle_gnss_fix2(const uavcan_equipment_gnss_Fix2 &msg);
    void handle_ahrs_solution(const uavcan_equipment_ahrs_Solution &msg);
    void handle_raw_imu(const uavcan_equipment_ahrs_RawIMU &msg);
    void handle_nav_solution(const uavcan_navigation_GlobalNavigationSolution &msg);
    void handle_static_pressure(const uavcan_equipment_air_data_StaticPressure &msg);
    void handle_static_temperature(const uavcan_equipment_air_data_StaticTemperature &msg);

    uint32_t last_nav_solution_ms;
    uint32_t last_ahrs_solution_ms;
    uint32_t last_imu_ms;
    uint32_t last_gps_ms;
    uint32_t last_baro_ms;

    float baro_pressure_pa;
    float baro_temperature_deg;
    bool have_baro_temperature;

    // singleton instance for trampoline callbacks
    static AP_ExternalAHRS_DroneCAN *_singleton;
};

#endif  // AP_EXTERNAL_AHRS_DRONECAN_ENABLED
