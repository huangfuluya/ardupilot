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
  implementation of Robotis Dynamixel 2.0 protocol for controlling servos
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#ifndef HAL_ZYSERVO_ENABLED
#define HAL_ZYSERVO_ENABLED 1
#endif

#if HAL_ZYSERVO_ENABLED

#define AP_ZYSERVO_MAX_NUM_SERVO NUM_SERVO_CHANNELS
#define AP_ZYSERVO_SERVO_ID_MIN (0x10)
#define AP_ZYSERVO_GCS_ID_MAX (0x3F)

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_ZYServo : public AP_CANDriver, public AP_ESC_Telem_Backend {
public:
    AP_ZYServo();
    ~AP_ZYServo();
    /* Do not allow copies */
    CLASS_NO_COPY(AP_ZYServo);

    static const struct AP_Param::GroupInfo var_info[];
    
    // return ZYServo from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_ZYServo *get_zcan(uint8_t driver_index);  // maybe it is not used

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    void update(); // do nothing, all handled in loop()
    bool is_servo_channel_active(uint8_t chan);
    // 将1000~2000的pwm范围转换为舵机的0~120度角度值
    uint16_t convert_pwm_to_angle_cd(uint16_t pwm);

    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout_us);
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout_us);
    void unpacket_and_log(const AP_HAL::CANFrame &in_frame, uint16_t expected_servo_id);
private:
    void loop();

    bool _initialized = false;
    AP_Int32 _chan_mask; // bitmask of channels to output on ZY bus
    AP_Int32 _servo_id_start;
    AP_Int32 _gcs_id_start;
    char _thread_name[16];

    uint8_t _driver_index;
    AP_HAL::CANIface* _can_iface;
    HAL_BinarySemaphore sem_handle;

    struct {
      uint8_t stage;
      float desired_angle;
      float current_angle;
      float current;
      uint16_t voltage_cv;  // I need this to compare with 0
      uint16_t status;
    } _telem[NUM_SERVO_CHANNELS];
    HAL_Semaphore _log_sem;
};

#endif  // HAL_ZYSERVO_ENABLED
