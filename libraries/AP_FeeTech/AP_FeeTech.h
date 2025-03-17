/*
 * AP_FeeTech.h
 *
 *  Created on: Aug 19, 2017
 *      Author: Mark Whitehorn
 */

#pragma once

#include "AP_FeeTech_config.h"

#if AP_FEETECH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "SCServo.h"
class AP_FeeTech {
public:
    AP_FeeTech();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_FeeTech);

    // get singleton instance
    static AP_FeeTech *get_singleton() {
        return _singleton;
    }


    static const struct AP_Param::GroupInfo var_info[];

    void update(void);

    SMS_STS sms_sts;
    
    void trim(void){
        _trim_flag = true;
    }
    void release(void){
        _trim_flag = false;
    }
private:
    static AP_FeeTech *_singleton;

    void init(void);

    AP_Int8 start_chan;
    AP_Int16 init_val;
    bool initialised;
    bool _trim_flag = true;
};

namespace AP {
    AP_FeeTech* feetech();
}

#endif  // AP_FEETECH_ENABLED
