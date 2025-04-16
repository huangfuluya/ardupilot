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
 #include <AP_SerialManager/AP_SerialManager.h>
 #include <SRV_Channel/SRV_Channel.h>
 #include <GCS_MAVLink/GCS.h>
 #include <AP_Logger/AP_Logger.h>
 #include "LogStructure.h"
 
 extern const AP_HAL::HAL& hal;
 

 const AP_Param::GroupInfo AP_Thrust_Stand::var_info[] = {
     // @Param: RATE
     // @DisplayName: SBUS default output rate
     // @Description: This sets the SBUS output frame rate in Hz.
     // @Range: 25 250
     // @User: Advanced
     // @Units: Hz
     AP_GROUPINFO("RATE",  1, AP_Thrust_Stand, _rate, 50),
 
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
    //  // get the serial port for the thrust stand
     AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
     if (!serial_manager) {
         return;
     }
     thrust_stand_uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_ThrustStand,0);
 
     if (!thrust_stand_uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "InertialLabs ExternalAHRS no UART");
        return;
    }
    _baudrate = serial_manager->find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    _port_num = serial_manager->find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Thrust_Stand::tick, void), "Thrust_Stand", 2048, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        AP_HAL::panic("Thrust Stand Failed to start update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Thrust Stand initialised");
 }
 void AP_Thrust_Stand::update()
 {
    //do nothing, I can do anything in the tick
 }

 void AP_Thrust_Stand::tick(void)
 {
    // Open port in the thread
    thrust_stand_uart->begin(_baudrate, 1024, 512);

    /*
      we assume the user has already configured the device
     */

    _setup_complete = true;
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(250);
        }
    }
 }

 bool AP_Thrust_Stand::check_uart(void){
    if (!_setup_complete) {
        return false;
    }
    thrust_stand_uart->begin(0);
    uint32_t n = thrust_stand_uart->available();
    if (n == 0) {
        return false;
    }
    return true;
 }
 
 void AP_Thrust_Stand::log_thrust_and_torque(void) const
 {
    const struct log_F_and_M pkt{
        LOG_PACKET_HEADER_INIT(LOG_F_AND_M_MSG),
        time_us     : AP_HAL::micros64(),
        Fx        : _Fx,
        Fy        : _Fy,
        Fz        : _Fz,
        Mx        : _Mx,
        My        : _My,
        Mz        : _Mz,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
 }

 namespace AP {
     AP_Thrust_Stand *thrust_stand()
     {
         return AP_Thrust_Stand::get_singleton();
     }
 }  // namespace AP

 #endif  // AP_THRUST_STAND_ENABLED
 
 