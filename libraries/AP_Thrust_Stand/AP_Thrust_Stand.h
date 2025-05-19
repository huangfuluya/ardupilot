
#pragma once

#include "AP_Thrust_Stand_config.h"

#if AP_THRUST_STAND_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "ModbusMaster.h"
#include <AP_Networking/AP_Networking_address.h>
#include <AP_HAL/utility/Socket.h>

class AP_Thrust_Stand
{
public:
    AP_Thrust_Stand();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Thrust_Stand);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_Thrust_Stand *get_singleton() { return _singleton; }

    void update();

    // 查找完整帧的函数
    char* findFrameEnd(char* buffer, int* frameLength);
    bool parseFrame(char* data, int32_t& Fx, int32_t& Fy, int32_t& Fz, int32_t& Mx, int32_t& My, int32_t& Mz);
private:
    // 用于存储接收到的数据
    char _recv_buffer[THRUST_STAND_RECV_BUFFER_SIZE];
    uint8_t _bufferOffset = 0;

    ModbusMaster modbus;

    AP_HAL::UARTDriver *thrust_stand_uart;

    void init(void);

    void tick(void);

    void log_thrust_and_torque(void) const;

    AP_Float _rate;
    AP_Int32 _mavlink_dt;

    static AP_Thrust_Stand *_singleton;

    int32_t _Fx;
    int32_t _Fy;
    int32_t _Fz;
    int32_t _Mx;
    int32_t _My;
    int32_t _Mz;

    int8_t _port_num;
    uint32_t _baudrate;
    bool _setup_complete = false;

    bool check_uart(void);

    bool update_modbus_FM(SocketAPM* sock);
};

 namespace AP {
     AP_Thrust_Stand *thrust_stand();
 }  // namespace AP
 
 #endif  // AP_THRUST_STAND_ENABLED

 
