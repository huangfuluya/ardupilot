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
  support for Pixhawk6X used as a DroneCAN External AHRS source.

  The Pixhawk6X running AP_Periph firmware broadcasts:
    - uavcan.equipment.ahrs.Solution   (attitude quaternion + angular velocity + linear acceleration)
    - uavcan.equipment.ahrs.RawIMU     (raw gyro + accelerometer samples)
    - uavcan.equipment.gnss.Fix2       (GPS fix - consumed by AP_GPS_DroneCAN)
    - uavcan.equipment.air_data.*      (barometer - consumed by AP_Baro_DroneCAN)
    - uavcan.equipment.ahrs.MagneticFieldStrength* (mag - consumed by AP_Compass_DroneCAN)

  This backend handles only the attitude (Solution) and raw IMU (RawIMU) messages.
  GPS, baro and compass data are handled by their respective DroneCAN backends.
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED

#include "AP_ExternalAHRS_Pixhawk6X.h"
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

// singleton instance for use in static DroneCAN callbacks
AP_ExternalAHRS_Pixhawk6X *AP_ExternalAHRS_Pixhawk6X::_singleton;

AP_ExternalAHRS_Pixhawk6X::AP_ExternalAHRS_Pixhawk6X(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    last_att_ms(0),
    last_imu_ms(0)
{
    _singleton = this;

    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Pixhawk6X ExternalAHRS initialised (DroneCAN)");
}

/*
  register DroneCAN subscriptions for Solution and RawIMU messages
*/
bool AP_ExternalAHRS_Pixhawk6X::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_solution, driver_index) != nullptr)
        && (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_rawimu, driver_index) != nullptr);
}

/*
  handle uavcan.equipment.ahrs.Solution message - provides attitude quaternion,
  angular velocity and linear acceleration from the Pixhawk6X EKF.
*/
void AP_ExternalAHRS_Pixhawk6X::handle_solution(AP_DroneCAN *ap_dronecan,
                                                const CanardRxTransfer &transfer,
                                                const uavcan_equipment_ahrs_Solution &msg)
{
    if (_singleton == nullptr) {
        return;
    }

    WITH_SEMAPHORE(_singleton->state.sem);

    // DroneCAN quaternion is XYZW; ArduPilot Quaternion is WXYZ
    _singleton->state.quat = Quaternion(msg.orientation_xyzw[3],  // w
                                        msg.orientation_xyzw[0],  // x
                                        msg.orientation_xyzw[1],  // y
                                        msg.orientation_xyzw[2]); // z
    _singleton->state.have_quaternion = true;
    _singleton->last_att_ms = AP_HAL::millis();
}

/*
  handle uavcan.equipment.ahrs.RawIMU message - provides raw gyro and
  accelerometer samples for AP_InertialSensor.
*/
void AP_ExternalAHRS_Pixhawk6X::handle_rawimu(AP_DroneCAN *ap_dronecan,
                                               const CanardRxTransfer &transfer,
                                               const uavcan_equipment_ahrs_RawIMU &msg)
{
    if (_singleton == nullptr) {
        return;
    }

    {
        WITH_SEMAPHORE(_singleton->state.sem);
        _singleton->state.accel = Vector3f(msg.accelerometer_latest[0],
                                           msg.accelerometer_latest[1],
                                           msg.accelerometer_latest[2]);
        _singleton->state.gyro  = Vector3f(msg.rate_gyro_latest[0],
                                           msg.rate_gyro_latest[1],
                                           msg.rate_gyro_latest[2]);
        _singleton->last_imu_ms = AP_HAL::millis();
    }

    AP_ExternalAHRS::ins_data_message_t ins;
    ins.accel       = _singleton->state.accel;
    ins.gyro        = _singleton->state.gyro;
    ins.temperature = -300.0f;  // temperature not available in RawIMU message

    AP::ins().handle_external(ins);
}

bool AP_ExternalAHRS_Pixhawk6X::healthy(void) const
{
    WITH_SEMAPHORE(state.sem);
    const uint32_t now_ms = AP_HAL::millis();
    return (last_att_ms != 0) && ((now_ms - last_att_ms) < 500U);
}

bool AP_ExternalAHRS_Pixhawk6X::initialised(void) const
{
    WITH_SEMAPHORE(state.sem);
    return last_att_ms != 0;
}

bool AP_ExternalAHRS_Pixhawk6X::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Pixhawk6X ExternalAHRS not healthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_Pixhawk6X::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    const uint32_t now_ms = AP_HAL::millis();
    const bool att_ok = (last_att_ms != 0) && ((now_ms - last_att_ms) < 500U);

    memset(&status, 0, sizeof(status));
    status.flags.attitude           = att_ok;
    status.flags.horiz_vel          = att_ok;
    status.flags.vert_vel           = att_ok;
    status.flags.horiz_pos_rel      = att_ok;
    status.flags.horiz_pos_abs      = att_ok;
    status.flags.vert_pos           = att_ok;
    status.flags.pred_horiz_pos_rel = att_ok;
    status.flags.pred_horiz_pos_abs = att_ok;
    status.flags.using_gps          = att_ok;
}

bool AP_ExternalAHRS_Pixhawk6X::get_variances(float &velVar, float &posVar, float &hgtVar,
                                              Vector3f &magVar, float &tasVar) const
{
    return false;
}

#endif  // AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED
