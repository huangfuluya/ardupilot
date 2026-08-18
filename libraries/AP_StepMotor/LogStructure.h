#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_STEPMOTOR \
    LOG_STEPMOTOR_MSG

// @LoggerMessage: STPM
// @Description: Emm_V5 closed-loop step motor telemetry
// @Field: TimeUS: Time since system startup
// @Field: CPos: Motor real-time position (deg)
// @Field: TPos: Motor target position (deg)
// @Field: CurTPos: Current target position being executed (deg)
// @Field: Vel: Motor real-time speed (RPM)
// @Field: PErr: Motor position error (deg)
struct PACKED log_StepMotor {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float cpos;
    float tpos;
    float cur_tpos;
    float vel;
    float perr;
};

#define LOG_STRUCTURE_FROM_STEPMOTOR        \
    { LOG_STEPMOTOR_MSG, sizeof(log_StepMotor), \
      "STPM",  "Qfffff", "TimeUS,CPos,TPos,CurTPos,Vel,PErr", "sdddd", "F0000", true },
