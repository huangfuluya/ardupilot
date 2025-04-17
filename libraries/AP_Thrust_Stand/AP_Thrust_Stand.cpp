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
 #include <AP_Common/AP_Common.h>
 #include "LogStructure.h"
 
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

    modbus.begin(1, *thrust_stand_uart);

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
        Fx        : float(_Fx/1000.0),
        Fy        : float(_Fy/1000.0),
        Fz        : float(_Fz/1000.0),
        Mx        : float(_Mx/1000.0),
        My        : float(_My/1000.0),
        Mz        : float(_Mz/1000.0),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
 }
void AP_Thrust_Stand::update_modbus_FM(void){
    if(modbus.readHoldingRegisters(THRUST_STAND_FX_REGISTER, 12)==0){
        _Fx = INT32_VALUE(HIGHBYTE(modbus.getResponseBuffer(0)), LOWBYTE(modbus.getResponseBuffer(0)),
                        HIGHBYTE(modbus.getResponseBuffer(1)), LOWBYTE(modbus.getResponseBuffer(1)));
        _Fy = INT32_VALUE(HIGHBYTE(modbus.getResponseBuffer(2)), LOWBYTE(modbus.getResponseBuffer(2)),
                        HIGHBYTE(modbus.getResponseBuffer(3)), LOWBYTE(modbus.getResponseBuffer(3)));
        _Fz = INT32_VALUE(HIGHBYTE(modbus.getResponseBuffer(4)), LOWBYTE(modbus.getResponseBuffer(4)),
                        HIGHBYTE(modbus.getResponseBuffer(5)), LOWBYTE(modbus.getResponseBuffer(5)));
        _Mx = INT32_VALUE(HIGHBYTE(modbus.getResponseBuffer(6)), LOWBYTE(modbus.getResponseBuffer(6)),
                        HIGHBYTE(modbus.getResponseBuffer(7)), LOWBYTE(modbus.getResponseBuffer(7)));
        _My = INT32_VALUE(HIGHBYTE(modbus.getResponseBuffer(8)), LOWBYTE(modbus.getResponseBuffer(8)),
                        HIGHBYTE(modbus.getResponseBuffer(9)), LOWBYTE(modbus.getResponseBuffer(9)));
        _Mz = INT32_VALUE(HIGHBYTE(modbus.getResponseBuffer(10)), LOWBYTE(modbus.getResponseBuffer(10)),
                        HIGHBYTE(modbus.getResponseBuffer(11)), LOWBYTE(modbus.getResponseBuffer(11)));
    }

}
 namespace AP {
     AP_Thrust_Stand *thrust_stand()
     {
         return AP_Thrust_Stand::get_singleton();
     }
 }  // namespace AP

 #endif  // AP_THRUST_STAND_ENABLED
 
