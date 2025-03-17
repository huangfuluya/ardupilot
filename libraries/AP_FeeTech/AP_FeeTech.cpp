

#include "AP_FeeTech_config.h"
#if AP_FEETECH_ENABLED

#include "AP_FeeTech.h"
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// singleton instance
AP_FeeTech *AP_FeeTech::_singleton;

const AP_Param::GroupInfo AP_FeeTech::var_info[] = {
    // @Param: CHAN
    // @DisplayName: 从哪个通道开始，0表示从第一个通道开始
    // @Description: 
    // @Range: 25 250
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("CHAN",  1, AP_FeeTech, start_chan, 0),

    // @Param: VAL
    // @DisplayName: 舵机的初始值，指的是直接发给舵机的指令，推荐0或者2048
    // @Description: 
    // @Range: 
    // @User: Advanced
    // @Units: 
    AP_GROUPINFO("VAL",   2, AP_FeeTech, init_val,   0),

    AP_GROUPEND
};


// constructor
AP_FeeTech::AP_FeeTech(void)
{
    if (_singleton != nullptr) {
        return;
    }
    _singleton = this;
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_FeeTech::update()
{
    if (!initialised) {
        initialised = true;
        init();
    }

    if (sms_sts.pSerial == nullptr) {
        return;
    }

    // constrain output rate using sbus_frame_interval
    static uint32_t last_micros = 0;
    uint32_t now = AP_HAL::micros();
    if ((now - last_micros) <= 100.0f) {
        return;
    }

    last_micros = now;
    
    /* 取前6个通道的值，然后向前发送*/
    uint8_t nchan = 6;
    uint16_t channels[nchan] {};

    //初始化一个数组，用来存放舵机的ID值
    uint8_t ID[nchan] {};
    int16_t position[nchan] {};

    if (_trim_flag) //如果是需要trim,那么全都输出1500
    {
        for (unsigned i = 0; i < nchan; ++i) {
            channels[i] = 1500;
            // sms_sts.WritePosEx(i + start_chan + 1,init_val,0,0);
            // sms_sts.RegWritePosEx(i + start_chan + 1,init_val,100,100);
            ID[i] = i + start_chan + 1;
            position[i] = init_val;
        }
        
    }else{
        // start_chan.set_and_save(constrain_int16(start_chan, 0, 8));//对start_chan进行限制，防止越界
        for (unsigned i = 0; i < nchan; ++i) {
            SRV_Channel *c = SRV_Channels::srv_channel(i + start_chan);
            if (c == nullptr) {
                continue;
            }
            channels[i] = c->get_output_pwm(); //返回pwm值，可以使用servo output的最大最小值的设置
            //1000到2000对应正负七圈
            int16_t pos = (int16_t)((channels[i] - 1500) * 7 * 4.096f);
            // sms_sts.WritePosEx(i + start_chan + 1,pos,0,0);
            // sms_sts.RegWritePosEx(i + start_chan + 1,pos,100,100);
            
            ID[i] = i + start_chan + 1;
            position[i] = pos;
            // pos = pos +1;
            // sms_sts.WritePosEx(1,2048,0,1000);
        }
    }

    static uint16_t count = 0;
    count++;
    if (count > 6){  //默认频率为300hz，而舵机只能接收50hz的指令，所以每6次发送一次
        count = 0;
        sms_sts.SyncWritePosEx(ID, nchan, position, nullptr, nullptr);
    }
    // sms_sts.RegWriteAction(0xFE);
    // sms_sts.SyncWritePosEx(ID, nchan, position, nullptr, nullptr);
    // hal.scheduler->delay_microseconds(1e6);
    // hal.scheduler->delay(1);
    
    // for (unsigned i = 0; i < nchan; ++i) {
    //     sms_sts.WritePosEx(i + start_chan + 1,position[i],0,0);
    //     hal.scheduler->delay_microseconds(1);
    // }
    

}

void AP_FeeTech::init() 
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (!serial_manager) {
        return;
    }
    sms_sts.pSerial = serial_manager->find_serial(AP_SerialManager::SerialProtocol_FEETECH,0);
}

namespace AP {

AP_FeeTech *feetech()
{
    return AP_FeeTech::get_singleton();
}

};

#endif  // AP_FEETECH_ENABLED
