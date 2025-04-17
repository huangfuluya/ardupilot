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
   DEVO Telemetry library
*/



#include "AP_Phase_Sensor.h"

#if AP_Phase_Sensor_ENABLED
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#define AP_SERIALMANAGER_PHASE_SENSOR_BAUD        115200
#define AP_SERIALMANAGER_PAHSE_SENSOR_BUFSIZE_RX        256
#define AP_SERIALMANAGER_PHASE_SENSOR_BUFSIZE_TX        256

#define BOARD_PHASE_SENSOR_DEFAULT 1
extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Phase_Sensor::var_info[] = {
    // @Param: TYPE
    // @DisplayName: RSSI Type
    // @Description: Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
    // @Values: 0:Disabled,1:AnalogPin,2:RCChannelPwmValue,3:ReceiverProtocol,4:PWMInputPin,5:TelemetryRadioRSSI
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 0, AP_Phase_Sensor, _enable,  BOARD_PHASE_SENSOR_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN_LOW
    // @DisplayName: RSSI pin's lowest voltage
    // @Description: 
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("SCALE", 1, AP_Phase_Sensor, _scale, 6.0f),

    // @Param: PIN_LOW
    // @DisplayName: RSSI pin's lowest voltage
    // @Description: 
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("INIT_VAL", 2, AP_Phase_Sensor, _init_val, 0.0),

    // @Param: PIN_LOW
    // @DisplayName: RSSI pin's lowest voltage
    // @Description: 
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("SAVE_VAL", 3, AP_Phase_Sensor, _save_val, 0.0),

    AP_GROUPEND
};


void AP_Phase_Sensor::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Phase_Sensor, 0))) {
        //_port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_PHASE_SENSOR_BAUD, AP_SERIALMANAGER_PAHSE_SENSOR_BUFSIZE_RX, AP_SERIALMANAGER_PHASE_SENSOR_BUFSIZE_TX);

        //hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Phase_Sensor::tick, void));
    }
}

void AP_Phase_Sensor::receive_frames() {
    uint32_t num = _port->available();
    while (num--) {
        read_uart_data(_port->read());
    }
}
bool AP_Phase_Sensor::get_phase_deg(float &angle) const
{
    if(!_port){return false;}
    if (AP_HAL::millis() - _last_data_ms > 1000) {
        return false;
    } else
    {
        angle = _phase_deg;
        return true;
    }
}
bool AP_Phase_Sensor::get_relative_phase_deg(float &angle) const
{
    float bias;
    if(!_port){return false;}
    if (AP_HAL::millis() - _last_data_ms > 1000) {
        return false;
    } 
    if(get_phase_deg(bias) && (!_reset_flag))
    {
        angle = wrap_180(bias - _init_val);
        return true;
    }
    return false;
}
float AP_Phase_Sensor::get_bias(void) const
{
    float bias;
    if(_reset_flag)
    {
        return _save_val;
    }
    if(get_phase_deg(bias) && (!_reset_flag))
    {
        return wrap_180(bias - _init_val) * _scale;
    }
    return 0.0f;
}
void AP_Phase_Sensor::tick(void)
{
    uint32_t now = AP_HAL::millis();
    static uint32_t count = 0;
    if(!_port){return;}
    if (now - _last_frame_ms > 10) {
        _last_frame_ms = now;
        receive_frames();
        
        AP::logger().WriteStreaming(
            "AANG",
            "TimeUS,Ang",
            "s-",
            "F-",
            "Qf",
            AP_HAL::micros64(),
            (double)_phase_deg);

        count++;
        if(count>1000)
        {
            count = 0;
            gcs().send_text(MAV_SEVERITY_INFO,"Angle is %0.3f",_phase_deg);
        }
    }
}
void AP_Phase_Sensor::read_uart_data(uint8_t ucData)
{ 
    static uint8_t ucDataUpdateFlag = 0;
    static uint8_t ucRx2Buffer[256];
    static uint8_t ucRx2Cnt = 0; 
    ucRx2Buffer[ucRx2Cnt++] = ucData;
    if( ucRx2Buffer[0] != 'A' )
      {
         ucRx2Cnt = 0;
         return ;
      }
    if(ucRx2Cnt < 2) return ;
    
    if((ucRx2Buffer[ucRx2Cnt - 2] == '\r') && (ucRx2Buffer[ucRx2Cnt - 1] == '\n'))  //收到帧尾
      {
         ucDataUpdateFlag = 1;
      }
    
    if(ucDataUpdateFlag)    //可以开始解包
      {
         ucDataUpdateFlag = 0;
         ucRx2Buffer[ucRx2Cnt] = 0;
         // 从字符串读取格式化输入
         float temp = 0.0f;
         if(StrExtFloat(temp,ucRx2Buffer,ucRx2Cnt))
         {
            _phase_deg = temp;      
            _last_data_ms = AP_HAL::millis();
         }
         ucRx2Cnt = 0;
      } 
}


bool AP_Phase_Sensor::StrExtFloat(float &val, uint8_t* Str, uint8_t len)
{
    float num[4] = {0.};
	//遍历深度
	int Fflag = 0;
	//数字个数
	int Fnum = 0;
	char num_start = 0, num_point = 0;
	//遍历到字符串尾部
	while (*Str != '\0')
	{
		Fflag++;
		//防止查询超过边界
		if (Fflag > len)
			break;
		//判断是不是数字
		if (*Str >= '0' && *Str <= '9')
		{
			//printf("%c",*Str);
			//判断数字存在
			num_start = 1;
			//判断是否存在小数点
			if (num_point >= 1)
			{
				num_point++;
				//当前小数部分的数值
				float fpoint = *Str - '0';
				for (int i = 1; i < num_point; i++)
				{
					fpoint = fpoint / 10.;
				}
				//加入小数部分
				num[Fnum + 1] = num[Fnum + 1] + fpoint;
			}
			else
			{
				//加入整数部分
				num[Fnum + 1] = num[Fnum + 1] * 10 + (*Str - '0');
			}
		}
		else if (*Str == '.') //判断为小数点
		{
			if (num_start == 1)//发现存在小数点
			{
				num_point = 1;
			}
		}
		else //判断为其他字符
		{
			if (num_start == 1)
			{
				Fnum++;//统计个数加一
			}
			//清空字符统计与小数点统计
			num_start = 0;
			num_point = 0;
		}
		//指针移动
		Str++;
	}
	//如果不是以字符结尾
	if (num_start == 1)
	{
		Fnum++;//统计个数加一
	}
	//放入提取到的数字个数
	num[0] = Fnum;
    if (Fnum>0)
    {
        val = num[Fnum];
        return true;
    }
    return false;
    
}

#endif
