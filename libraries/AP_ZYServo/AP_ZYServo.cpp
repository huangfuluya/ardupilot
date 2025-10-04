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

  Portions of this code are based on the dynamixel_sdk code:
  https://github.com/ROBOTIS-GIT/DynamixelSDK
  which is under the following license:

* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include "AP_ZYServo.h"

#if HAL_ZYSERVO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <AP_Logger/AP_Logger.h>
extern const AP_HAL::HAL& hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "ZYServo", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

const AP_Param::GroupInfo AP_ZYServo::var_info[] = {
    // @Param: ZY_CHAN
    // @DisplayName: ZY channels
    // @Description: Bitmask defining which servo channels are to be transmitted over ZY bus
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32
    // @User: Advanced
    AP_GROUPINFO("CHAN", 1, AP_ZYServo, _chan_mask, 0xFFFF),

    AP_GROUPINFO("SID0", 2, AP_ZYServo, _servo_id_start, AP_ZYSERVO_SERVO_ID_MIN),

    AP_GROUPINFO("GID0", 3, AP_ZYServo, _gcs_id_start, AP_ZYSERVO_GCS_ID_MAX),
    AP_GROUPEND
};

// constructor
AP_ZYServo::AP_ZYServo(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ZYServo::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;
    debug_can(AP_CANManager::LOG_DEBUG, "ZYServo: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: already initialized\n\r");
        return;
    }
    snprintf(_thread_name, sizeof(_thread_name), "ZYServo_%u", driver_index);
    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ZYServo::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "ZYServo: init done\n\r");
}
AP_ZYServo::~AP_ZYServo(void)
{
    // nothing yet
}
void AP_ZYServo::update(void)
{
    // do nothing, all handled in loop()
}
bool AP_ZYServo::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&sem_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Cannot add event handle\n\r");
        return false;
    }
    return true;
}
void AP_ZYServo::loop(void)
{
    AP_HAL::CANFrame txFrame {};
    AP_HAL::CANFrame rxFrame {};
    static uint64_t last_loop_time_stamps_us = AP_HAL::micros64();

    //循环最多是100Hz,即内部的延迟不应超过10ms，即10000微秒,最多32个通道，最坏情况下每个通道用时300微秒
    while (true) {
        if (!_initialized) {
            debug_can(AP_CANManager::LOG_ERROR, "ZYServo: not initialized\n\r");
            hal.scheduler->delay_microseconds(10000);
            continue;
        }
        if (AP_HAL::micros64() - last_loop_time_stamps_us < 10000) {
            //距离上次循环时间不足10ms，等待
            hal.scheduler->delay_microseconds(10000 - (AP_HAL::micros64() - last_loop_time_stamps_us));
            continue;
        }
        last_loop_time_stamps_us = AP_HAL::micros64();
        // 获取需要发送的通道的值
        for (uint8_t chan = 1; chan <= AP_ZYSERVO_MAX_NUM_SERVO; chan++) {
            if (is_servo_channel_active(chan)) {
                uint16_t raw_pwm;
                SRV_Channels::get_output_pwm_chan(chan-1, raw_pwm);
                // 构建CAN帧
                uint16_t servo_id = _servo_id_start + uint16_t(chan-1); // 假设ID从0x10开始
                uint16_t gcs_id = _gcs_id_start - uint16_t(chan-1); // 假设GCS ID从0x3F开始递减
                // 扩展帧，先传低字节再传高字节
                txFrame.id = (uint32_t(0x1400 | gcs_id)<<16) | 0x4405 | AP_HAL::CANFrame::FlagEFF;

                uint16_t angle = convert_pwm_to_angle_cd(raw_pwm);
                txFrame.data[0] = angle & 0xFF; // 低字节
                txFrame.data[1] = (angle >> 8) & 0xFF; // 高字节
                txFrame.data[2] = 0x00; // 保留字节
                txFrame.data[3] = 0x00; // 保留字节
                txFrame.data[4] = 0x00; // 保留字节
                txFrame.data[5] = 0x00; // 保留字节
                txFrame.data[6] = 0x00; // 保留字节
                txFrame.data[7] = 0x00; // 保留字节
                txFrame.dlc = 8;
                txFrame.canfd = false;
                // 发送CAN帧
                if (write_frame(txFrame, 100)) {
                    debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Failed to send CAN frame for channel %u\n\r", chan);
                }
                if(read_frame(rxFrame,100)){
                    // 处理接收到的反馈帧
                    // 如果是舵机角度和电压的则记录下来
                    unpacket_and_log(rxFrame,servo_id);  
                }
                //返回消息中包含两帧，因此读两遍
                if(read_frame(rxFrame,100)){  
                    // 处理接收到的反馈帧
                    // 如果是舵机角度和电压的则记录下来
                    unpacket_and_log(rxFrame,servo_id);
                }
            }
        }
    }
}

void AP_ZYServo::unpacket_and_log(const AP_HAL::CANFrame &in_frame,uint16_t expected_servo_id) {
    // 解析反馈帧
    if (!(in_frame.id & AP_HAL::CANFrame::FlagEFF)) {
        // 不是扩展帧，忽略
        // gcs().send_text(MAV_SEVERITY::MAV_SEVERITY_INFO, "Not extended frame");
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Not extended frame\n\r");
        return;
    }
    uint32_t id = in_frame.id & AP_HAL::CANFrame::MaskExtID;
    uint16_t base_id = (id >> 16) & 0xFFFF;
    uint16_t func_id = id & 0xFFFF;
    if ((base_id & 0xFF00) != 0x1400 || ((func_id != 0xFC05) && func_id != 0xFC06)) {
        // 不是ZYServo的反馈帧，忽略
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Not ZYServo frame\n\r");
        return;
    }
    uint8_t servo_id = (base_id & 0x00FF);
    if (servo_id != expected_servo_id) {
        // 不是预期的舵机ID，忽略
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Unexpected servo ID\n\r");
        return;
    }
    uint8_t chan = servo_id - _servo_id_start + 1; // 计算通道号
    if (chan < 1 || chan > AP_ZYSERVO_MAX_NUM_SERVO) {
        // 通道号无效，忽略
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Invalid channel number\n\r");
        return;
    }
    if (!is_servo_channel_active(chan)) {
        // 通道未激活，忽略
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Channel not active\n\r");
        return;
    }
    if (in_frame.dlc < 4) {
        // 数据长度不足，忽略
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Frame DLC too short\n\r");
        return;
    }
    {
        WITH_SEMAPHORE(_log_sem);
    // static uint16_t last_chan = 0;
    if (func_id == 0xFC05) {
        // 提取角度和电压
        uint16_t des_angle_cd = in_frame.data[0] | (in_frame.data[1] << 8);
        uint16_t cur_angle_cd = in_frame.data[2] | (in_frame.data[3] << 8);
        uint16_t current_ca = in_frame.data[4] | (in_frame.data[5] << 8);
        uint16_t status = in_frame.data[6] | (in_frame.data[7] << 8);
        _telem[chan - 1].stage = 1; 
        _telem[chan - 1].desired_angle = des_angle_cd / 100.0f; // 转换为度
        _telem[chan - 1].current_angle = cur_angle_cd / 100.0f; // 转换为度
        _telem[chan - 1].current = current_ca / 100.0f; // 转换为安培
        _telem[chan - 1].status = status;
    } else if (func_id == 0xFC06) {
        // 提取电压
        uint16_t voltage_cv = in_frame.data[0] | (in_frame.data[1] << 8);
        _telem[chan - 1].stage = 2;
        _telem[chan - 1].voltage_cv = voltage_cv;
        // if (chan >= last_chan) {
        //     last_chan = chan;
        // }
    }

#if HAL_LOGGING_ENABLED
    static uint64_t last_log_time_us = AP_HAL::micros64();
    uint64_t delta_log_ms = (AP_HAL::micros64() - last_log_time_us) / 1000;
    // // 隔1s向地面站发送一次
    // static uint64_t last_gcs_time_us = AP_HAL::micros64();
    // if (AP_HAL::micros64() - last_gcs_time_us > 1000000) {
    //     last_gcs_time_us = AP_HAL::micros64();
    //     gcs().send_text(MAV_SEVERITY::MAV_SEVERITY_INFO,
    //         "ZYServo CH%u Dangle:%.2f Angle:%.2f Cur:%.2fV:%.2f Sta:0x%04X",
    //         chan,
    //         _telem[chan - 1].desired_angle,
    //         _telem[chan - 1].current_angle,
    //         _telem[chan - 1].current,
    //         _telem[chan - 1].voltage,
    //         _telem[chan - 1].status
    //     );
    // }
    // 记录不要超过50Hz
    // if (telem[chan - 1].stage == 2 && last_chan >= chan && delta_log_ms > 20) {
    if ((_telem[chan - 1].stage == 2) && (delta_log_ms > 20)){
        last_log_time_us = AP_HAL::micros64();

        for (uint8_t i=0; i<ARRAY_SIZE(_telem); i++) {
            if(_telem[i].voltage_cv == 0){
                // 没有收到过电压数据，说明没有收到完整数据，跳过
                continue;
            }
            // @LoggerMessage: ZY05
            // @Description: ZY05 servo data
            // @Field: TimeUS: Time since system startup
            // @Field: chan: Instance channel
            // @Field: Dang: desired angle
            // @Field: ang: reported angle
            // @Field: cur: reported current
            // @Field: vol: reported voltage
            // @Field: sta: reported status
            // @Units: s,deg,deg,A,V,bitmask
            // @ scale
            // @ data type

            // char log_name[6];
            // snprintf(log_name, sizeof(log_name), "ZY%u", chan);
            AP::logger().WriteStreaming("ZY05",
                "TimeUS,chan,Dang,Cang,cur,vol,sta,delta",
                "s#ddAv-s",
                "F000000F",
                "QBffffHQ",
                AP_HAL::micros64(),
                chan, // convert to 1 indexed to match actuator IDs and SERVOx numbering
                _telem[i].desired_angle,
                _telem[i].current_angle,
                _telem[i].current,
                _telem[i].voltage_cv / 100.0f, // convert to volts
                _telem[i].status,
                delta_log_ms*1000
            );
            _telem[i].stage = 0; // reset stage after logging
        }
    }
#endif // HAL_LOGGING_ENABLED
    }
}    

// active is true, deactivated is false
bool AP_ZYServo::is_servo_channel_active(uint8_t chan) {
    if (chan < 1 || chan > 32) {
        return false;
    }
    return (_chan_mask & (1 << (chan - 1))) != 0;
}

uint16_t AP_ZYServo::convert_pwm_to_angle_cd(uint16_t pwm) {
    // 将1000~2000的pwm范围转换为舵机的0~120度角度值
    pwm = constrain_uint16(pwm, 1000, 2000);
    uint16_t angle = (pwm - 1000) * 12;// * 120.0f / 1000.0f; // 线性映射到0~120 00  厘度
    return angle; 
}
// write frame on CAN bus
bool AP_ZYServo::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout_us)
{
    // wait for space in buffer to send command

    bool read_select = false;
    bool write_select = true;
    const uint64_t deadline_us = AP_HAL::micros64() + timeout_us;
    bool ret = _can_iface->select(read_select, write_select, &out_frame, deadline_us);
    
    if (!ret || !write_select) {
        return false;
    }

    // send frame and return success
    return (_can_iface->send(out_frame, deadline_us, AP_HAL::CANIface::AbortOnError) == 1);
}
bool AP_ZYServo::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout_us)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "ZYServo: Driver not initialized for read_frame\n\r");
        return false;
    }

    bool read_select = true;
    bool write_select = false;
    bool ret;
    const uint64_t deadline_us = AP_HAL::micros64() + timeout_us;
    ret = _can_iface->select(read_select, write_select, nullptr, deadline_us);
    if (!ret || !read_select) {
        return false;
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    return (_can_iface->receive(recv_frame, time, flags) == 1);
}
#endif  // HAL_ZYSERVO_ENABLED
