#include "AP_StepMotor.h"
#if AP_STEPMOTOR_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

#include <GCS_MAVLink/GCS.h>
#define STEP_MOTOR_DEFAULT_CHAN 0
const AP_Param::GroupInfo AP_StepMotor::var_info[] = {
    // @Param: CHAN
    // @DisplayName: step motor use chan value
    // @Description: 
    // @Range: 0~14, 0 or -1 is not used
    // @User: Advanced
    // @Units: 
    AP_GROUPINFO("CHAN",  1, AP_StepMotor, _output_chan, STEP_MOTOR_DEFAULT_CHAN),

    AP_GROUPINFO("DT",     2, AP_StepMotor, _dt_send,     100), // 100ms send interval

    AP_GROUPINFO("SC",     3, AP_StepMotor, _scale,      1.0f), // scale factor for channel output

    AP_GROUPINFO("VEL",    4, AP_StepMotor, _vel_rpm,   500), // default velocity in RPM.0~5000

    AP_GROUPINFO("ACC",    5, AP_StepMotor, _acc,       0), // default acceleration (0 means direct start without ramp),0~255

    AP_GROUPEND
};


AP_StepMotor::AP_StepMotor()
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton) {
        AP_HAL::panic("Too many RSSI sensors");
    }
    _singleton = this;
}

void AP_StepMotor::init()
{   
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (!serial_manager)
    {
        return;
    }
    _step_motor_uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_StepMotor, 0);
    if (_step_motor_uart == nullptr) {
        return;
    }
    _step_motor_uart->begin(115200, 128, 128); // 初始化UART，波特率115200，接收和发送缓冲区大小128字节
    _initialised = true;

}
void AP_StepMotor::update()
{
    if (!_initialised) {
        _initialised = true;
        init();
    }
    if (_step_motor_uart == nullptr) {
        return;
    }
    if (_output_chan < 0 || _output_chan > 14) {
        return; // 无效通道
    }
    // read_bytes();
    send(_output_chan);
}
void AP_StepMotor::read_bytes()
{
    if (_step_motor_uart == nullptr) {
        return; // UART未初始化
    }
    uint8_t buf[128] = {0};
    int len = _step_motor_uart->read(buf, sizeof(buf));
    if (len > 0) {
        // 处理接收到的数据
        // 这里可以添加解析逻辑
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Received %d bytes from Step Motor UART", len);
    }
}
void AP_StepMotor::send(uint8_t chan)
{

    if (chan < 1 || chan > 14) {
        return; // invalid channel
    }
    if (!_initialised) {
        init(); // 确保初始化
        return; // 如果初始化失败，直接返回
    }

    // static uint32_t last_gcs_send_time = 0;
    static uint32_t last_step_motor_send_time = 0;

    if (AP_HAL::millis() - last_step_motor_send_time > _dt_send)
    {
        float chan_scaled = SRV_Channels::get_output_norm(SRV_Channels::channel_function(chan-1)); // 确保通道已初始化,通道编号从0开始
        uint32_t clk = (uint32_t)((chan_scaled + 1.0) * _scale * 51200);                                   // 将通道值转换为脉冲数，一圈的脉冲数为360/1.8步距角*256细分数=51200脉冲
        // 位置模式：速度1000RPM，加速度0（不使用加减速直接启动），脉冲数3200（16细分下发送3200个脉冲电机转一圈），相对运动

        Emm_V5_Pos_Control(1, 0, _vel_rpm, _acc, clk, 1, 0);
        last_step_motor_send_time = AP_HAL::millis();
    }
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void AP_StepMotor::Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  _step_motor_uart->write(cmd, 13);
//   _step_motor_uart->flush(); // 确保数据发送完成
}


/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void AP_StepMotor::Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  _step_motor_uart->write(cmd, i);
}
AP_StepMotor* AP_StepMotor::get_singleton()
{
    if (!_singleton) {
        _singleton = new AP_StepMotor();
    }
    return _singleton;
}

AP_StepMotor *AP_StepMotor::_singleton = nullptr;

namespace AP {

AP_StepMotor *stepmotor()
{
    return AP_StepMotor::get_singleton();
}

};


#endif // AP_STEPMOTOR_ENABLED
