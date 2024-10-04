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

    static const struct AP_Param::GroupInfo var_info[];

    void update(void);

    SMS_STS sms_sts;
    
private:

    void init(void);

    AP_Int8 start_chan;
    bool initialised;
};

#endif  // AP_FEETECH_ENABLED
