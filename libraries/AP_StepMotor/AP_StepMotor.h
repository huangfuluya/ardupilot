#pragma once

#ifndef AP_STEPMOTOR_ENABLED
#define AP_STEPMOTOR_ENABLED 1
#endif // AP_STEP_MOTOR_ENABLED

#if AP_STEPMOTOR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>


class AP_StepMotor
{
public:
    AP_StepMotor();
    // ~AP_StepMotor(void); // Destructor is not needed
    CLASS_NO_COPY(AP_StepMotor);
    static const struct AP_Param::GroupInfo var_info[];

    void init(void);
    void send(uint8_t chan);
    void update_thread(void);
    void update(void);
    static AP_StepMotor *get_singleton();

    
    typedef enum {
    S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
    S_RL    = 1,      /* 读取读取相电阻和相电感 */
    S_PID   = 2,      /* 读取PID参数 */
    S_VBUS  = 3,      /* 读取总线电压 */
    S_CPHA  = 5,      /* 读取相电流 */
    S_ENCL  = 7,      /* 读取经过线性化校准后的编码器值 */
    S_TPOS  = 8,      /* 读取电机目标位置角度 */
    S_VEL   = 9,      /* 读取电机实时转速 */
    S_CPOS  = 10,     /* 读取电机实时位置角度 */
    S_PERR  = 11,     /* 读取电机位置误差角度 */
    S_FLAG  = 13,     /* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 14,     /* 读取驱动参数 */
    S_State = 15,     /* 读取系统状态参数 */
    S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
    }SysParams_t;

private:
    static AP_StepMotor *_singleton;
    AP_Int8 _output_chan; // 输出通道
    
    AP_HAL::UARTDriver *_step_motor_uart;
    bool _initialised;

    void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
    void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);
    void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
    bool packet_cur_deg(uint8_t *rxCmd, uint8_t rxCount, float& deg);
    void read_bytes(void);

    AP_Int16 _dt_send;
    AP_Float _scale;
    AP_Int16 _vel_rpm;
    AP_Int16 _acc;
    AP_Int8 _divide;

    uint8_t _rxCmd[128] = {0}; // 接收命令
    uint8_t _rxCmdLen = 0; // 接收命令长度

    float _des_deg = 0.0f; // 目标角度
    float _cur_deg = 0.0f; // 当前角度
};

namespace AP {
    AP_StepMotor *stepmotor();
};


#endif // AP_STEPMOTOR_ENABLED
