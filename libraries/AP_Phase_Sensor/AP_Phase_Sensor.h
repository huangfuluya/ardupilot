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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#ifndef AP_Phase_Sensor_ENABLED
    #define AP_Phase_Sensor_ENABLED   1
#endif

#if AP_Phase_Sensor_ENABLED
class AP_Phase_Sensor {
public:
    //constructor
    AP_Phase_Sensor() {}

    /* Do not allow copies */
    AP_Phase_Sensor(const AP_Phase_Sensor &other) = delete;
    AP_Phase_Sensor &operator=(const AP_Phase_Sensor&) = delete;
    // destructor
    ~AP_Phase_Sensor(void){}

    void init();
    bool get_phase_deg(float &angle) const;
    float get_bias(void) const;
    bool get_relative_phase_deg(float &angle) const;
    bool get_reset_flag() const {return _reset_flag;}
    float get_save_val() const {return _save_val;}
     // tick - main call to send updates to transmitter
    void tick(void);
    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    void set_reset_flag(bool flag){_reset_flag = flag;}
private:

   bool _reset_flag = false;

    // receive_frames - sends updates down telemetry link
    void receive_frames();
    void read_uart_data(uint8_t ucData);
    bool StrExtFloat(float &val,uint8_t* Str, uint8_t len);
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    uint32_t _last_frame_ms;
    uint32_t _last_data_ms;
    float _phase_deg = 0.0f;

    AP_Int8 _enable;
    AP_Float _init_val;
    AP_Float _scale;
    AP_Float _save_val;
};
#endif
