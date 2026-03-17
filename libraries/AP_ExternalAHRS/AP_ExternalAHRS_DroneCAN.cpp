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

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_DRONECAN_ENABLED

#include "AP_ExternalAHRS_DroneCAN.h"

#include <AP_Math/AP_Math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

// timeout for considering data stale (milliseconds)
#define DRONECAN_AHRS_TIMEOUT_MS 500

AP_ExternalAHRS_DroneCAN *AP_ExternalAHRS_DroneCAN::_singleton;

AP_ExternalAHRS_DroneCAN::AP_ExternalAHRS_DroneCAN(AP_ExternalAHRS *_frontend,
                                                   AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    _singleton = this;

    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO));

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DroneCAN ExternalAHRS initialised");
}

/*
  subscribe to DroneCAN messages needed for External AHRS
 */
bool AP_ExternalAHRS_DroneCAN::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_gnss_fix2_trampoline, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_ahrs_solution_trampoline, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_raw_imu_trampoline, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_nav_solution_trampoline, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_static_pressure_trampoline, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_static_temperature_trampoline, driver_index) != nullptr);
}

// --- trampoline functions ---

void AP_ExternalAHRS_DroneCAN::handle_gnss_fix2_trampoline(AP_DroneCAN *ap_dronecan,
                                                            const CanardRxTransfer &transfer,
                                                            const uavcan_equipment_gnss_Fix2 &msg)
{
    if (_singleton != nullptr) {
        _singleton->handle_gnss_fix2(msg);
    }
}

void AP_ExternalAHRS_DroneCAN::handle_ahrs_solution_trampoline(AP_DroneCAN *ap_dronecan,
                                                                const CanardRxTransfer &transfer,
                                                                const uavcan_equipment_ahrs_Solution &msg)
{
    if (_singleton != nullptr) {
        _singleton->handle_ahrs_solution(msg);
    }
}

void AP_ExternalAHRS_DroneCAN::handle_raw_imu_trampoline(AP_DroneCAN *ap_dronecan,
                                                          const CanardRxTransfer &transfer,
                                                          const uavcan_equipment_ahrs_RawIMU &msg)
{
    if (_singleton != nullptr) {
        _singleton->handle_raw_imu(msg);
    }
}

void AP_ExternalAHRS_DroneCAN::handle_nav_solution_trampoline(AP_DroneCAN *ap_dronecan,
                                                               const CanardRxTransfer &transfer,
                                                               const uavcan_navigation_GlobalNavigationSolution &msg)
{
    if (_singleton != nullptr) {
        _singleton->handle_nav_solution(msg);
    }
}

void AP_ExternalAHRS_DroneCAN::handle_static_pressure_trampoline(AP_DroneCAN *ap_dronecan,
                                                                  const CanardRxTransfer &transfer,
                                                                  const uavcan_equipment_air_data_StaticPressure &msg)
{
    if (_singleton != nullptr) {
        _singleton->handle_static_pressure(msg);
    }
}

void AP_ExternalAHRS_DroneCAN::handle_static_temperature_trampoline(AP_DroneCAN *ap_dronecan,
                                                                     const CanardRxTransfer &transfer,
                                                                     const uavcan_equipment_air_data_StaticTemperature &msg)
{
    if (_singleton != nullptr) {
        _singleton->handle_static_temperature(msg);
    }
}

// --- message handlers ---

/*
  handle uavcan.navigation.GlobalNavigationSolution - the primary AHRS data source.
  Provides orientation quaternion, position, velocity, angular velocity, and linear
  acceleration all in one message.
 */
void AP_ExternalAHRS_DroneCAN::handle_nav_solution(const uavcan_navigation_GlobalNavigationSolution &msg)
{
    const uint32_t now_ms = AP_HAL::millis();

    // update orientation and IMU state
    {
        WITH_SEMAPHORE(state.sem);

        // quaternion from message is XYZW; ArduPilot quaternion is WXYZ
        state.quat = Quaternion(msg.orientation_xyzw[3],
                                msg.orientation_xyzw[0],
                                msg.orientation_xyzw[1],
                                msg.orientation_xyzw[2]);
        state.have_quaternion = true;

        // angular velocity and acceleration from body frame
        state.gyro = Vector3f(msg.angular_velocity_body[0],
                              msg.angular_velocity_body[1],
                              msg.angular_velocity_body[2]);
        state.accel = Vector3f(msg.linear_acceleration_body[0],
                               msg.linear_acceleration_body[1],
                               msg.linear_acceleration_body[2]);

        // position: GlobalNavigationSolution uses degrees (float64)
        state.location = Location(int32_t(msg.latitude * 1.0e7),
                                  int32_t(msg.longitude * 1.0e7),
                                  int32_t(msg.height_msl * 100.0f),
                                  Location::AltFrame::ABSOLUTE);
        state.last_location_update_us = AP_HAL::micros();
        state.have_location = true;

        // velocity in body frame; convert to NED using quaternion
        const Vector3f vel_body(msg.linear_velocity_body[0],
                                msg.linear_velocity_body[1],
                                msg.linear_velocity_body[2]);
        state.velocity = state.quat.inverse() * vel_body;
        state.have_velocity = !is_zero(vel_body.length_squared());

        if (!state.have_origin) {
            state.origin = state.location;
            state.have_origin = true;
        }
    }

    last_nav_solution_ms = now_ms;
    last_imu_ms = now_ms;

    // pass IMU data to InertialSensor
    AP_ExternalAHRS::ins_data_message_t ins {};
    ins.accel = state.accel;
    ins.gyro = state.gyro;
    ins.temperature = AP::baro().get_temperature();
    AP::ins().handle_external(ins);

    // pass barometer data if available
    if (last_baro_ms != 0 && !isnan(msg.height_baro) && msg.height_baro > 0.0f) {
        AP_ExternalAHRS::baro_data_message_t baro {};
        baro.instance = 0;
        baro.pressure_pa = baro_pressure_pa;
        baro.temperature = have_baro_temperature ? baro_temperature_deg : AP::baro().get_temperature();
        AP::baro().handle_external(baro);
    }
}

/*
  handle uavcan.equipment.ahrs.Solution - orientation + angular velocity + acceleration.
  Used when GlobalNavigationSolution is not available.
 */
void AP_ExternalAHRS_DroneCAN::handle_ahrs_solution(const uavcan_equipment_ahrs_Solution &msg)
{
    // if we are receiving GlobalNavigationSolution, ignore the less-complete Solution msg
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_nav_solution_ms < DRONECAN_AHRS_TIMEOUT_MS) {
        return;
    }

    {
        WITH_SEMAPHORE(state.sem);

        // quaternion: XYZW → WXYZ
        state.quat = Quaternion(msg.orientation_xyzw[3],
                                msg.orientation_xyzw[0],
                                msg.orientation_xyzw[1],
                                msg.orientation_xyzw[2]);
        state.have_quaternion = true;

        state.gyro = Vector3f(msg.angular_velocity[0],
                              msg.angular_velocity[1],
                              msg.angular_velocity[2]);
        state.accel = Vector3f(msg.linear_acceleration[0],
                               msg.linear_acceleration[1],
                               msg.linear_acceleration[2]);
    }

    last_imu_ms = now_ms;
    last_ahrs_solution_ms = now_ms;

    // pass IMU data to InertialSensor
    AP_ExternalAHRS::ins_data_message_t ins {};
    ins.accel = state.accel;
    ins.gyro = state.gyro;
    ins.temperature = AP::baro().get_temperature();
    AP::ins().handle_external(ins);
}

/*
  handle uavcan.equipment.ahrs.RawIMU - raw gyro and accelerometer data.
  Used when Solution/GlobalNavigationSolution is not available.
 */
void AP_ExternalAHRS_DroneCAN::handle_raw_imu(const uavcan_equipment_ahrs_RawIMU &msg)
{
    const uint32_t now_ms = AP_HAL::millis();
    // if we are receiving higher-level messages, prefer those for IMU
    if (now_ms - last_nav_solution_ms < DRONECAN_AHRS_TIMEOUT_MS ||
        now_ms - last_ahrs_solution_ms < DRONECAN_AHRS_TIMEOUT_MS) {
        return;
    }

    {
        WITH_SEMAPHORE(state.sem);
        state.gyro = Vector3f(msg.rate_gyro_latest[0],
                              msg.rate_gyro_latest[1],
                              msg.rate_gyro_latest[2]);
        state.accel = Vector3f(msg.accelerometer_latest[0],
                               msg.accelerometer_latest[1],
                               msg.accelerometer_latest[2]);
    }

    last_imu_ms = now_ms;

    AP_ExternalAHRS::ins_data_message_t ins {};
    ins.accel = state.accel;
    ins.gyro = state.gyro;
    ins.temperature = AP::baro().get_temperature();
    AP::ins().handle_external(ins);
}

/*
  handle uavcan.equipment.gnss.Fix2 - GPS fix data
 */
void AP_ExternalAHRS_DroneCAN::handle_gnss_fix2(const uavcan_equipment_gnss_Fix2 &msg)
{
    const uint32_t now_ms = AP_HAL::millis();

    AP_ExternalAHRS::gps_data_message_t gps {};

    // map fix status
    switch (msg.status) {
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX:
        gps.fix_type = AP_GPS_FixType::NONE;
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY:
        gps.fix_type = AP_GPS_FixType::NONE;
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX:
        gps.fix_type = AP_GPS_FixType::FIX_2D;
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX:
        if (msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS) {
            gps.fix_type = AP_GPS_FixType::DGPS;
        } else if (msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK) {
            if (msg.sub_mode == UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT) {
                gps.fix_type = AP_GPS_FixType::RTK_FLOAT;
            } else if (msg.sub_mode == UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED) {
                gps.fix_type = AP_GPS_FixType::RTK_FIXED;
            } else {
                gps.fix_type = AP_GPS_FixType::FIX_3D;
            }
        } else {
            gps.fix_type = AP_GPS_FixType::FIX_3D;
        }
        break;
    default:
        gps.fix_type = AP_GPS_FixType::NO_GPS;
        break;
    }

    gps.satellites_in_view = msg.sats_used;

    // position: Fix2 uses degrees * 1e8
    gps.latitude  = int32_t(msg.latitude_deg_1e8 / 10);
    gps.longitude = int32_t(msg.longitude_deg_1e8 / 10);
    gps.msl_altitude = msg.height_msl_mm / 10;  // mm → cm

    // NED velocity in m/s
    gps.ned_vel_north = msg.ned_velocity[0];
    gps.ned_vel_east  = msg.ned_velocity[1];
    gps.ned_vel_down  = msg.ned_velocity[2];

    // accuracy from covariance diagonal (if provided)
    if (msg.covariance.len >= 3) {
        gps.horizontal_pos_accuracy = sqrtf((msg.covariance.data[0] + msg.covariance.data[1]) * 0.5f);
        gps.vertical_pos_accuracy   = sqrtf(msg.covariance.data[2]);
    }
    if (msg.covariance.len >= 6) {
        gps.horizontal_vel_accuracy = sqrtf((msg.covariance.data[3] + msg.covariance.data[4]) * 0.5f);
    }

    gps.hdop = msg.pdop;
    gps.vdop = msg.pdop;

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }

    if (gps.fix_type >= AP_GPS_FixType::FIX_3D) {
        WITH_SEMAPHORE(state.sem);
        if (!state.have_origin) {
            state.origin = Location(gps.latitude, gps.longitude,
                                    gps.msl_altitude, Location::AltFrame::ABSOLUTE);
            state.have_origin = true;
        }
        last_gps_ms = now_ms;
    }
}

/*
  handle uavcan.equipment.air_data.StaticPressure - barometric pressure
 */
void AP_ExternalAHRS_DroneCAN::handle_static_pressure(const uavcan_equipment_air_data_StaticPressure &msg)
{
    baro_pressure_pa = msg.static_pressure;

    AP_ExternalAHRS::baro_data_message_t baro {};
    baro.instance = 0;
    baro.pressure_pa = baro_pressure_pa;
    baro.temperature = have_baro_temperature ? baro_temperature_deg : AP::baro().get_temperature();
    AP::baro().handle_external(baro);

    last_baro_ms = AP_HAL::millis();
}

/*
  handle uavcan.equipment.air_data.StaticTemperature - ambient temperature
 */
void AP_ExternalAHRS_DroneCAN::handle_static_temperature(const uavcan_equipment_air_data_StaticTemperature &msg)
{
    // temperature is in Kelvin; convert to Celsius
    baro_temperature_deg = msg.static_temperature - 273.15f;
    have_baro_temperature = true;
}

// --- health / status API ---

bool AP_ExternalAHRS_DroneCAN::initialised(void) const
{
    return last_imu_ms != 0;
}

bool AP_ExternalAHRS_DroneCAN::healthy(void) const
{
    const uint32_t now_ms = AP_HAL::millis();
    return (last_imu_ms != 0) && (now_ms - last_imu_ms < DRONECAN_AHRS_TIMEOUT_MS);
}

bool AP_ExternalAHRS_DroneCAN::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "DroneCAN AHRS not healthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_DroneCAN::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (healthy()) {
        status.flags.attitude = state.have_quaternion;
        status.flags.horiz_pos_abs = state.have_location;
        status.flags.vert_pos = state.have_location;
        status.flags.horiz_vel = state.have_velocity;
        status.flags.pred_horiz_pos_abs = state.have_location;
        status.flags.using_gps = (last_gps_ms != 0);
        status.flags.gps_quality_good = (last_gps_ms != 0);
        status.flags.initalized = initialised();
    }
}

#endif  // AP_EXTERNAL_AHRS_DRONECAN_ENABLED
