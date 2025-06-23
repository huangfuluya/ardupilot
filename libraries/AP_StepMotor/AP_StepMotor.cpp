#include "AP_StepMotor.h"
#if AP_STEPMOTOR_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

#include <GCS_MAVLink/GCS.h>
#define STEP_MOTOR_DEFAULT_CHAN 0
extern const AP_HAL::HAL& hal;

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

    AP_GROUPINFO("DIV",    6, AP_StepMotor, _divide,    64), // default divide factor for stepper motor,16 is 1.8 degree stepper motor
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
    // 启用一个单独的线程来处理步进电机控制

    // hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_StepMotor::update_thread, void));
    if (hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_StepMotor::update_thread, void), "StepMotor", 1024, AP_HAL::Scheduler::PRIORITY_IO, 0))
    {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StepMotor thread created successfully");
        _initialised = true;
    }
}
void AP_StepMotor::update_thread(void)
{
    // static uint32_t last_gcs_send_time = 0;
    // static uint32_t last_step_motor_send_time = 0;
    while (true)
    {
        // if (AP_HAL::millis() - last_step_motor_send_time > (uint32_t)_dt_send)
        // {
            
        // }
        hal.scheduler->delay(_dt_send);
        // read_bytes();
        send(_output_chan);
        // last_step_motor_send_time = AP_HAL::millis();
    }
}
void AP_StepMotor::update()
{
    if (!_initialised) {
        // _initialised = true;
        init();
        return;
    }
    if (_step_motor_uart == nullptr) {
        return;
    }
    if (_output_chan < 1 || _output_chan > 14) {
        return; // 无效通道
    }

}
void AP_StepMotor::read_bytes(void)
{
    if (_step_motor_uart == nullptr) {
        return; // UART未初始化
    }

    //清空接收命令数组
    memset(_rxCmd, 0, sizeof(_rxCmd));
    _rxCmdLen = 0; // 重置接收命令长度
    //清空接收缓存区的内容
    while(_step_motor_uart->available() > 0) {
        _step_motor_uart->read();
    }
    // 读取电机实时位置
    Emm_V5_Read_Sys_Params(1, S_CPOS);
    hal.scheduler->delay(1);
    // 等待返回命令，命令数据缓存在数组rxCmd上，长度为rxCount
    Emm_V5_Receive_Data(_rxCmd, &_rxCmdLen);

    if (_rxCmdLen > 0) {
        // 处理接收到的命令
        //打印_rxCmd
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Recv:%d bytes", _rxCmdLen);
        if(packet_cur_deg(_rxCmd, _rxCmdLen, _cur_deg))
        {
            // 发送到GCS
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Current Position: %.2f degrees", _cur_deg);
            // mavlink_debug_vect_t debug_vect;
            // debug_vect.x = _cur_deg;
            // debug_vect.y = _des_deg;
            // debug_vect.z = 0.0f;
            // strncpy(debug_vect.name, "cur,des,nu", sizeof(debug_vect.name) - 1);
            // debug_vect.name[sizeof(debug_vect.name) - 1] = '\0'; // 确保字符串以null结尾
            // mavlink_msg_debug_vect_send_struct(
            //     mavlink_channel_t::MAVLINK_COMM_0,
            //     &debug_vect
            // );
        }
        else
        {
            //将_rxCmd按16进制打印出来

            for (uint8_t i = 0; i < _rxCmdLen; ++i) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "0x%02X ", _rxCmd[i]);
            }
        } 
    }


}
void AP_StepMotor::send(uint8_t chan)
{
    float chan_scaled = SRV_Channels::get_output_norm(SRV_Channels::channel_function(chan - 1)); // 确保通道已初始化,通道编号从0开始
    _des_deg = (chan_scaled + 1.0) * _scale * 360.0f;                                            // 将通道值转换为角度，范围-180到180度
    uint32_t clk = (uint32_t)(_des_deg * (float)(_divide) / 1.8f);                                         // 将通道值转换为脉冲数，一圈的脉冲数为360/1.8步距角*256细分数=51200脉冲
    // 位置模式：速度1000RPM，加速度0（不使用加减速直接启动），脉冲数3200（16细分下发送3200个脉冲电机转一圈），相对运动

    Emm_V5_Pos_Control(1, 0, _vel_rpm, _acc, clk, 1, 0);
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
//   hal.scheduler->delay_microseconds(5); // 等待1微秒，确保数据发送完毕
//   _step_motor_uart->flush(); // 确保数据发送完毕
//   hal.scheduler->delay_microseconds(5); // 等待1微秒，确保数据发送完毕
}

bool AP_StepMotor::packet_cur_deg(uint8_t *rxCmd, uint8_t rxCount, float &deg)
{
    uint32_t pos = 0; // 位置值
    if (rxCmd[0] == 1 && rxCmd[1] == 0x36 && rxCount == 8)
    {

        // 拼接成uint32_t类型
        pos = (uint32_t)(((uint32_t)rxCmd[3] << 24) |
                         ((uint32_t)rxCmd[4] << 16) |
                         ((uint32_t)rxCmd[5] << 8) |
                         ((uint32_t)rxCmd[6] << 0));

        // 转换成角度
        deg = (float)pos * 360.0f / 65536.0f;

        // 符号
        if (rxCmd[2])
        {
            deg = -deg;
        }
        return true; // 成功解析
    }
    return false; // 解析失败
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
//   hal.scheduler->delay_microseconds(5); // 等待1微秒，确保数据发送完毕
//   _step_motor_uart->flush(); // 确保数据发送完毕
//   hal.scheduler->delay_microseconds(5); // 等待1微秒，确保数据发送完毕
}
AP_StepMotor* AP_StepMotor::get_singleton()
{
    if (!_singleton) {
        _singleton = new AP_StepMotor();
    }
    return _singleton;
}

/**
  * @brief    接收数据
  * @param    rxCmd   : 接收到的数据缓存在该数组
  * @param    rxCount : 接收到的数据长度
  * @retval   无
  */
void AP_StepMotor::Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
    int i = 0;
    unsigned long lTime;                    // 上一时刻的时间
    unsigned long cTime;                    // 当前时刻的时间

    // 记录当前的时间
    lTime = AP_HAL::millis();

    // 开始接收数据
    while (1)
    {
        if (_step_motor_uart->available() > 0) // 串口有数据进来
        {
            if (i <= 128) // 防止数组溢出，该值需要小于数组的长度
            {
                rxCmd[i++] = _step_motor_uart->read(); // 接收数据

                lTime = AP_HAL::millis();                 // 更新上一时刻的时间
            }
        }
        else // 串口有没有数据
        {
            cTime = AP_HAL::millis();                   // 获取当前时刻的时间

            if((int)(cTime - lTime) > 0)      // 100毫秒内串口没有数据进来，就判定一帧数据接收结束
            // if (1)
            {
                *rxCount = i; // 数据长度
                break;        // 退出while(1)循环
            }
        }
    }
}

AP_StepMotor *AP_StepMotor::_singleton = nullptr;

namespace AP {

AP_StepMotor *stepmotor()
{
    return AP_StepMotor::get_singleton();
}

};


#endif // AP_STEPMOTOR_ENABLED
