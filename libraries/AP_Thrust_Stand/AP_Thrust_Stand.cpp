/*
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "AP_Thrust_Stand.h"
#if AP_THRUST_STAND_ENABLED

#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/AP_Common.h>
#include "LogStructure.h"
#include <AP_Networking/AP_Networking.h>
#include <AP_HAL/utility/Socket.h>
#include <stdio.h>
#define THRUST_STAND_FX_REGISTER 0x0a00
#define THRUST_STAND_FY_REGISTER 0x0a02
#define THRUST_STAND_FZ_REGISTER 0x0a04
#define THRUST_STAND_MX_REGISTER 0x0a06
#define THRUST_STAND_MY_REGISTER 0x0a08
#define THRUST_STAND_MZ_REGISTER 0x0a0a


 extern const AP_HAL::HAL& hal;
 

 const AP_Param::GroupInfo AP_Thrust_Stand::var_info[] = {
     // @Param: RATE
     // @DisplayName: SBUS default output rate
     // @Description: This sets the SBUS output frame rate in Hz.
     // @Range: 25 250
     // @User: Advanced
     // @Units: Hz
     AP_GROUPINFO("RATE",  1, AP_Thrust_Stand, _rate, 1000),
     AP_GROUPINFO("MAV_DT",  2, AP_Thrust_Stand, _mavlink_dt, 50),
 
     AP_GROUPEND
 };
 
 AP_Thrust_Stand *AP_Thrust_Stand::_singleton;
 // constructor
 AP_Thrust_Stand::AP_Thrust_Stand(void)
 {
     // set defaults from the parameter table
     AP_Param::setup_object_defaults(this, var_info);
     if (_singleton != nullptr) {
         AP_HAL::panic("AP_Thrust_Stand must be singleton");
     }
     _singleton = this;
 }

 void AP_Thrust_Stand::init(void)
 {
    _setup_complete = false;

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Thrust_Stand::tick, void), "Thrust_Stand", 8192, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Thrust Stand Failed to start update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Thrust Stand initialised");
    _setup_complete = true;
 }
 void AP_Thrust_Stand::update()
 {
     if (!_setup_complete)
     {
         init();
         return;
     }
     // 在这里进行一些数据发送的任务
     static uint32_t last_mavlink_time_stamps = AP_HAL::millis();
     if (AP_HAL::millis() - last_mavlink_time_stamps > _mavlink_dt)
     {
         // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FM: Fx: %d, Fy: %d, Fz: %d, Mx: %d, My: %d, Mz: %d", _Fx, _Fy, _Fz, _Mx, _My, _Mz);
         __mavlink_debug_vect_t debug_vect;
         debug_vect.x = _Fx;
         debug_vect.y = _Fy;
         debug_vect.z = _Fz;
         debug_vect.time_usec = AP_HAL::micros64();
         char name[] = "FX_FY_FZ";
         memcpy(debug_vect.name, name, sizeof(name));
         debug_vect.name[sizeof(name) - 1] = '\0';
         mavlink_msg_debug_vect_send_struct(MAVLINK_COMM_0, &debug_vect);

         last_mavlink_time_stamps = AP_HAL::millis();
     }
 }

 void AP_Thrust_Stand::tick(void)
 {
    while (!AP::network().is_healthy())
    {
        hal.scheduler->delay(10);
    }
     AP::network().startup_wait();
     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_client: starting");
     // const char *dest = param.test_ipaddr.get_str();
     auto *sock = new SocketAPM(false);
     static uint32_t last_time_stamps = AP_HAL::millis();

     
     uint32_t dt = 0;
     while (true)
     {
         dt = (uint32_t)(round(1000 / _rate));
         dt = constrain_int32(dt, 1, 20000);
         if (AP_HAL::millis() - last_time_stamps < dt)
         {
             
             if (!_setup_complete)
             {
                 init();
                 continue;
             }
             hal.scheduler->delay(dt - (AP_HAL::millis() - last_time_stamps));
         }
         else
         {
             if (update_modbus_FM(sock))
             {
                 last_time_stamps = AP_HAL::millis();
             }
             else
             {
                 hal.scheduler->delay(2000);
             }
         }
     }
 }

 bool AP_Thrust_Stand::check_uart(void){
    if (!_setup_complete) {
        return false;
    }
    //thrust_stand_uart->begin(0);
    /*int32_t n = thrust_stand_uart->available();
    if (n == 0) {
        return false;
    }*/
    return true;
 }
 
 void AP_Thrust_Stand::log_thrust_and_torque(void) const
 {
    const struct log_F_and_M pkt{
        LOG_PACKET_HEADER_INIT(LOG_F_AND_M_MSG),
        time_us     : AP_HAL::micros64(),
        Fx        : float(_Fx/100.0),
        Fy        : float(_Fy/100.0),
        Fz        : float(_Fz/100.0),
        Mx        : float(_Mx/1000.0),
        My        : float(_My/1000.0),
        Mz        : float(_Mz/1000.0),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
 }
bool AP_Thrust_Stand::update_modbus_FM(SocketAPM* sock){
    if (sock == nullptr)
         {
             GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TCP_client: failed to create socket");
             return false;
         }
         if (!sock->is_connected())
         {
            const char *dest = AP::network().get_thrust_stand_ip_str();
             if (!sock->connect(dest, 502))
             {
                 GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FM: failed to connect");
                 return false;
             }
             if (!sock->set_blocking(true))
             {
                 GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FM: failed to set blocking");
                 return false;
             }
         }

         uint8_t buf[128]{};
         const ssize_t ret = sock->recv(buf, sizeof(buf), 10);
         if (ret > 0)
         {
             // 将接收到的数据添加到缓冲区
             if (_bufferOffset + ret >= THRUST_STAND_RECV_BUFFER_SIZE)
             {
                 _bufferOffset = 0;
             }
             else
             {
                 memcpy(_recv_buffer + _bufferOffset, buf, ret);
                 _bufferOffset += ret;
                 _recv_buffer[_bufferOffset] = '\0';
             }
         }
         int frameLength;
         char* frameEnd = findFrameEnd(_recv_buffer, &frameLength);
         if (frameEnd != nullptr) {
             if(parseFrame(frameEnd, _Fx, _Fy, _Fz, _Mx, _My, _Mz))
             {
                log_thrust_and_torque();
                 //清空缓冲区，重置偏移量
                 memset(_recv_buffer, 0, sizeof(_recv_buffer));
                 _bufferOffset = 0;
             }
             else
             {
                 GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FM: failed to parse frame");
             }
         }
         return true;
}

char* AP_Thrust_Stand::findFrameEnd(char* buffer, int* frameLength)
{
    //打印buffer
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FM: buffer: %s", buffer);
    char* endPos = strstr(buffer, "END");
    char* startPos = strstr(buffer, "RD");
    while (endPos != nullptr && startPos != nullptr) {
        if (endPos < startPos) {
            memset(endPos, 0, 3);
            // 如果END在RD之前，返回nullptr
            endPos = strstr(buffer, "END");
            startPos = strstr(buffer, "RD");
            continue;
        }
        if (endPos - startPos < 71-3) {
            memset(endPos, 0, 3);
            memset(startPos, 0, 2);
            // 如果END之前的数据长度小于70
            endPos = strstr(buffer, "END");
            startPos = strstr(buffer, "RD");
            continue;
        }
        *frameLength = (endPos - startPos) + 3;
        //打印frameLength
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "frameLength: %d", *frameLength);
        if (*frameLength >= 71) { // 确保END前有足够的数据
            return startPos; // 返回END之后的位置
        }
    }
    return nullptr;
}

bool AP_Thrust_Stand::parseFrame(char* data, int32_t &Fx, int32_t &Fy, int32_t &Fz, int32_t &Mx, int32_t &My, int32_t &Mz)
{
        //打印buffer
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FM: buffer: %s", data);
    //打印strlen(data)
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FM: buffer length: %d", (int)strlen(data));
    // 检查数据格式是否符合基本要求
    if (strlen(data) < 71 || strncmp(data + 71 - 3, "END", 3) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FM:error1");
        return false; // 数据格式不正确
    }
        // 检查头部是否为RD
    if (strncmp(data, "RD", 2) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FM:error2");
        return false; // 数据格式不正确
    }
    // 检查数据有效性标识
    if (strncmp(data + 5, "RRRRRR", 6) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FM:error3");
        return false; // 数据无效
    }
    // 提取N
    if (strncmp(data + 20, "N237", 4) != 0)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FM:error4");
        return false; // 数据无效
    }
    // 提取Fx, Fy, Fz, Mx, My, Mz
    char *ptr = data + 25;
    char *endPtr;
    Fx = (int32_t)strtol(ptr,    &endPtr, 10);
    Fy = (int32_t)strtol(ptr+7,  &endPtr, 10);
    Fz = (int32_t)strtol(ptr+14, &endPtr, 10);
    Mx = (int32_t)strtol(ptr+21, &endPtr, 10);
    My = (int32_t)strtol(ptr+28, &endPtr, 10);
    Mz = (int32_t)strtol(ptr+35, &endPtr, 10);

    return true;
}
 namespace AP {
     AP_Thrust_Stand *thrust_stand()
     {
         return AP_Thrust_Stand::get_singleton();
     }
 }  // namespace AP

 #endif  // AP_THRUST_STAND_ENABLED
 
