#pragma once

#include <AP_Logger/LogStructure.h>

// @LoggerMessage: RPM
// @Description: Data from RPM sensors
// @Field: TimeUS: Time since system startup
// @Field: rpm1: First sensor's data
// @Field: rpm2: Second sensor's data
struct PACKED log_F_and_M {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Fx;
    float Fy;
    float Fz;
    float Mx;
    float My;
    float Mz;
};

#define LOG_STRUCTURE_FROM_F_AND_M        \
    { LOG_F_AND_M_MSG, sizeof(log_F_and_M), \
      "FM",  "Qffffff", "TimeUS,Fx,Fy,Fz,Mx,My,Mz", "s------", "F000000" , true },
