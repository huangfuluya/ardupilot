

#include "AP_FeeTech_config.h"
#if AP_FEETECH_ENABLED

#include "AP_FeeTech.h"
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_FeeTech::var_info[] = {
    // @Param: CHAN
    // @DisplayName: 从哪个通道开始，0表示从第一个通道开始
    // @Description: 
    // @Range: 25 250
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("CHAN",  1, AP_FeeTech, start_chan, 0),

    AP_GROUPEND
};


// constructor
AP_FeeTech::AP_FeeTech(void)
{
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
    // start_chan.set_and_save(constrain_int16(start_chan, 0, 8));//对start_chan进行限制，防止越界
    for (unsigned i = 0; i < nchan; ++i) {
        SRV_Channel *c = SRV_Channels::srv_channel(i + start_chan);
        if (c == nullptr) {
            continue;
        }
        channels[i] = c->get_output_pwm(); //返回pwm值，可以使用servo output的最大最小值的设置
        //1000到2000对应正负七圈
        int16_t pos = (int16_t)((channels[i] - 1500) * 7 * 4.096f);
        sms_sts.WritePosEx(i + start_chan,pos,0,0);
        // pos = pos +1;
        // sms_sts.WritePosEx(1,2048,0,1000);
    }
}

void AP_FeeTech::init() 
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (!serial_manager) {
        return;
    }
    sms_sts.pSerial = serial_manager->find_serial(AP_SerialManager::SerialProtocol_FEETECH,0);
}

#endif  // AP_FEETECH_ENABLED
