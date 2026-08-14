#pragma once

#ifndef AP_STEPMOTOR_ENABLED
#define AP_STEPMOTOR_ENABLED 1
#endif // AP_STEPMOTOR_ENABLED

#if AP_STEPMOTOR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

/*
  Driver for the Emm_V5.0 (Emm42_V5.0 / ZDT_X42) closed-loop stepper
  servo board, controlled over UART with the vendor's custom protocol.

  The driver is hooked into SRV_Channels (same pattern as AP_Volz_Protocol /
  AP_RobotisServo): the normalised output (-1..1) of the servo channel
  selected by SRV_STM_CHAN is converted into a target angle and streamed to
  the board as relative position commands. update() is called from the main
  thread by SRV_Channels::push(), so no extra thread is needed.

  Setup on the flight controller:
    SERIALx_PROTOCOL = 49 (StepMotor)
    SERIALx_BAUD     = 115200 (must match the board's baud)
    SERVOn_FUNCTION  = function assigned to the followed channel
    SRV_STM_CHAN     = 1..16 channel to follow, 0 disables the driver

  Setup on the motor board (OLED menu):
    P_Serial = the matching UART mode, baud 115200
    Checksum = matches SRV_STM_CKSUM (default 0x6B)
    P_Pul    = subdivision, must match SRV_STM_DIV
*/

class AP_StepMotor
{
public:
    AP_StepMotor();
    CLASS_NO_COPY(AP_StepMotor);

    static const struct AP_Param::GroupInfo var_info[];

    // called from SRV_Channels::push() in the main loop
    void update(void);

    static AP_StepMotor *get_singleton();

    // telemetry accessors
    float get_cur_deg(void) const { return _cur_deg; }  // last reported position [deg]
    float get_des_deg(void) const { return _des_deg; }  // last commanded target [deg]
    bool  healthy(void) const;                          // recent position feedback seen

    // checksum mode, must match the board's "Checksum" menu
    typedef enum {
        CHECKSUM_6B   = 0,  // fixed 0x6B byte (board default)
        CHECKSUM_XOR  = 1,  // XOR of all previous bytes
        CHECKSUM_CRC8 = 2,  // CRC-8
    } ChecksumType;

    // system parameter read codes (Emm_V5 read commands)
    typedef enum {
        S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
        S_RL    = 1,      /* 读取相电阻和相电感 */
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
    } SysParams_t;

private:
    static AP_StepMotor *_singleton;

    // parameters
    AP_Int8  _output_chan;    // SRV_Channels channel to follow, 0=disabled
    AP_Int16 _dt_send;        // command send interval [ms]
    AP_Float _scale;          // target angle = norm * SC * 360 [deg]
    AP_Int16 _vel_rpm;        // speed for position commands [RPM]
    AP_Int8  _acc;            // acceleration gear 0-255 (0=no ramp)
    AP_Int8  _divide;         // microstep subdivision, 200*DIV pulses/rev for 1.8deg motor
    AP_Int8  _addr;           // motor bus address
    AP_Int8  _checksum_type;  // ChecksumType
    AP_Int8  _fsact;          // failsafe action: 1=stop on RC failsafe

    AP_HAL::UARTDriver *_uart;
    bool _initialised;

    // state
    float    _cur_deg;             // last reported position
    float    _des_deg;             // last commanded target
    float    _cmd_deg;             // position the motor has been commanded to
    bool     _cmd_deg_valid;       // _cmd_deg is synchronised with the motor
    bool     _feedback_valid;      // at least one position frame received
    uint32_t _last_feedback_ms;    // last position frame time
    uint32_t _last_poll_ms;        // last position poll time
    uint32_t _last_send_ms;        // last motion command time
    uint32_t _init_ms;             // init time
    bool     _stopped;             // e-stop sent, waiting for recovery
    uint8_t  _last_status;         // last command status byte
    uint32_t _last_err_msg_ms;     // rate limiting for GCS messages
    bool     _no_fb_reported;      // "no feedback" warning already sent

    // receive parser (non-blocking frame state machine)
    enum ParseState {
        PARSE_WAIT_ADDR,
        PARSE_WAIT_FN,
        PARSE_WAIT_DATA,
    };
    ParseState _parse_state;
    uint8_t    _rxbuf[12];
    uint8_t    _rxlen;   // expected total frame length
    uint8_t    _rxidx;

    // Emm_V5 protocol commands
    void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                            uint32_t clk, bool raF, bool snF);
    void Emm_V5_Speed_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
    void Emm_V5_Stop(uint8_t addr, bool snF);
    void Emm_V5_Enable(uint8_t addr, bool en);
    void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);

    // helpers
    bool    write_cmd(const uint8_t *cmd, uint8_t len);
    uint8_t checksum_byte(const uint8_t *cmd, uint8_t len) const;
    bool    checksum_ok(const uint8_t *frame, uint8_t len) const;
    void    read_incoming(void);
    uint8_t expected_frame_len(uint8_t fn) const;
    void    handle_frame(void);
    void    init(void);
};

namespace AP {
    AP_StepMotor *stepmotor();
};

#endif // AP_STEPMOTOR_ENABLED
