/*
  AP_StepMotor.cpp - driver for the Emm_V5.0 (Emm42_V5.0 / ZDT_X42)
  closed-loop stepper servo board over UART.

  Protocol reference: Emm_V5.0步进闭环驱动说明书Rev1.3 (ref_docs)

  All I/O is done non-blocking from update(), which is called by
  SRV_Channels::push() in the main thread.
 */

#include "AP_StepMotor.h"

#if AP_STEPMOTOR_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_StepMotor/LogStructure.h>

extern const AP_HAL::HAL& hal;

// serial settings: the UART is opened by AP_SerialManager (same pattern
// as AP_RobotisServo). The driver itself must not call begin() again.
// Baudrate follows SERIALx_BAUD (e.g. 57 = 57600, 115 = 115200).

// poll the board's full system state (0x43) at this period. A single
// 0x43 frame carries position/speed/error plus voltage, current and
// status flags, replacing the old 0x36 poll + 0x35/0x37/0x33/0x34
// round-robin.
#define STEP_MOTOR_POLL_MS        20
// feedback older than this is considered stale
#define STEP_MOTOR_FB_TIMEOUT_MS  3000
// a partial frame whose next byte takes longer than this is considered
// lost (dropped/garbled bytes): resynchronise the parser on the next
// address byte instead of swallowing the following frame. Must exceed
// the main loop period (a frame can be split across update() calls).
#define STEP_MOTOR_INTERBYTE_MS   50
// wait this long for position feedback before assuming position 0
#define STEP_MOTOR_SYNC_TIMEOUT_MS 1000
// telemetry log write period
#define STEP_MOTOR_LOG_MS         STEP_MOTOR_POLL_MS

const AP_Param::GroupInfo AP_StepMotor::var_info[] = {
    // @Param: CHAN
    // @DisplayName: Step motor control channel
    // @Description: The SRV_Channels output channel whose PWM (1000 to 2000) drives the step motor target angle. PWM 1500 is the zero angle, PWM 1000 or 2000 command -SC*360 or +SC*360 degrees. When the channel has no PWM output the motor returns to (or stays at) the zero position. The SERVOn_FUNCTION of that channel must be assigned to a servo function. Setting 0 disables the driver.
    // @Values: 0:Disabled,1:Chan1,2:Chan2,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("CHAN",  1, AP_StepMotor, _output_chan, 0),

    // @Param: DT
    // @DisplayName: Send interval
    // @Description: Interval between position commands sent to the motor. The servo channel output is sampled and any change in target angle is sent as a relative move at this rate.
    // @Units: ms
    // @Range: 20 1000
    // @User: Advanced
    AP_GROUPINFO("DT",    2, AP_StepMotor, _dt_send, 100),

    // @Param: SC
    // @DisplayName: Angle scale
    // @Description: Target angle scaling. Full deflection of the followed channel (PWM 1000 or 2000) commands SC*360 degrees. Set 0.5 for +/-180deg at full deflection.
    // @Units: deg
    // @Range: 0.01 50
    // @User: Standard
    AP_GROUPINFO("SC",    3, AP_StepMotor, _scale, 1.0f),

    // @Param: VEL
    // @DisplayName: Speed
    // @Description: Speed used for position commands.
    // @Units: RPM
    // @Range: 0 5000
    // @User: Standard
    AP_GROUPINFO("VEL",   4, AP_StepMotor, _vel_rpm, 500),

    // @Param: ACC
    // @DisplayName: Acceleration
    // @Description: Acceleration gear for position commands, 0-255. 0 starts directly at the configured speed without ramp. Ramp step time is (256-ACC)*50us per RPM.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("ACC",   5, AP_StepMotor, _acc, 0),

    // @Param: DIV
    // @DisplayName: Microstep subdivision
    // @Description: Subdivision configured in the board's P_Pul menu. For a 1.8 degree motor one revolution needs 200*DIV pulses (16 subdivision = 3200 pulses). Must match the board or all angles will be scaled wrongly.
    // @Range: 1 256
    // @User: Advanced
    AP_GROUPINFO("DIV",   6, AP_StepMotor, _divide, 16),

    // @Param: ADDR
    // @DisplayName: Motor address
    // @Description: Bus address of the motor, must match the board's ID menu.
    // @Range: 1 255
    // @User: Advanced
    AP_GROUPINFO("ADDR",  7, AP_StepMotor, _addr, 1),

    // @Param: CKSUM
    // @DisplayName: Checksum type
    // @Description: Checksum mode, must match the board's Checksum menu. 0x6B is the board default.
    // @Values: 0:0x6B,1:XOR,2:CRC8
    // @User: Advanced
    AP_GROUPINFO("CKSUM", 8, AP_StepMotor, _checksum_type, 0),

    // @Param: FSACT
    // @DisplayName: Failsafe action
    // @Description: Action on RC failsafe. The motor is always stopped while the hardware safety switch is in the safe position. Set 0 if you control the outputs only via MAVLink with no RC receiver connected.
    // @Values: 0:None,1:Stop motor
    // @User: Advanced
    AP_GROUPINFO("FSACT", 9, AP_StepMotor, _fsact, 1),

    AP_GROUPEND
};

AP_StepMotor::AP_StepMotor()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton) {
        AP_HAL::panic("Too many StepMotor drivers");
    }
    _singleton = this;
}

/*
  initialise the UART and enable the motor
 */
void AP_StepMotor::init(void)
{
    _initialised = true;
    _init_ms = AP_HAL::millis();

    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (serial_manager == nullptr) {
        return;
    }
    _uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_StepMotor, 0);
    if (_uart == nullptr) {
        return;
    }

    // the port is already begun by AP_SerialManager with the baudrate and
    // buffers from SERIALx_BAUD, exactly like AP_RobotisServo does it

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StepMotor: found UART, enabling motor");
    Emm_V5_Enable(_addr, true);
}

/*
  called by SRV_Channels::push() in the main loop
 */
void AP_StepMotor::update(void)
{
    if (!_initialised) {
        init();
        if (_uart == nullptr) {
            return;
        }
    }
    if (_uart == nullptr) {
        // retry once per second in case SERIALx_PROTOCOL is set late
        if (AP_HAL::millis() - _init_ms >= 1000) {
            _initialised = false;
        }
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // always drain and parse incoming frames (acknowledgements and feedback)
    read_incoming();

    // poll the board's system state (position/speed/error/flags/voltage/current)
    if ((now - _last_poll_ms) >= STEP_MOTOR_POLL_MS) {
        _last_poll_ms = now;
        Emm_V5_Read_Sys_Params(_addr, S_State);
    }

#if HAL_LOGGING_ENABLED
    // write telemetry log at a fixed rate
    if ((now - _last_log_ms) >= STEP_MOTOR_LOG_MS) {
        _last_log_ms = now;
        Log_Write();
    }
#endif

    if (_output_chan < 1 || _output_chan > NUM_SERVO_CHANNELS) {
        return; // driver disabled or invalid channel
    }

    // e-stop on RC failsafe (optional) and on hardware safety switch
    const bool safety_stop = (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED);
    const bool failsafe_stop = (_fsact != 0) && rc().in_rc_failsafe();
    if (safety_stop || failsafe_stop) {
        if (!_stopped) {
            _stopped = true;
            _cmd_deg_valid = false;
            Emm_V5_Stop(_addr, false);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "StepMotor: stopped (%s)",
                          safety_stop ? "safety switch" : "RC failsafe");
        }
        return;
    }
    _stopped = false;

    // send motion commands at the configured rate
    if ((now - _last_send_ms) < (uint16_t)_dt_send.get()) {
        return;
    }
    _last_send_ms = now;

    // target angle from the PWM of the followed servo channel.
    // PWM 1000..2000 maps linearly to -SC*360..+SC*360 deg, PWM 1500 = 0 deg.
    const SRV_Channel *ch = SRV_Channels::srv_channel(_output_chan.get() - 1);
    const uint16_t pwm = (ch != nullptr) ? ch->get_output_pwm() : 0;

    if (pwm == 0) {
        // no PWM output on the channel: hold/return to the zero position
        _des_deg = 0.0f;
    } else {
        const float norm = constrain_float(((float)pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
        _des_deg = norm * _scale * 360.0f;
    }

    // pulses per degree for a 1.8 degree motor: (360/1.8)*DIV/360
    const float steps_per_deg = _divide.get() / 1.8f;

    if (!_cmd_deg_valid) {
        // synchronise the command baseline with the reported position
        if (_feedback_valid) {
            _cmd_deg = _cur_deg;
            _abs_pulse = _cur_deg * steps_per_deg;
            _cmd_deg_valid = true;
        } else if ((now - _init_ms) > STEP_MOTOR_SYNC_TIMEOUT_MS) {
            // no feedback at all: assume the motor is at 0 so control is not blocked
            if (!_no_fb_reported) {
                _no_fb_reported = true;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "StepMotor: no position feedback, assuming 0deg");
            }
            _cmd_deg = 0.0f;
            _abs_pulse = 0.0f;
            _cmd_deg_valid = true;
        } else {
            return;
        }
    }

    // only send when the target has moved by at least one pulse since the
    // last command. Resending an identical absolute position at the DT rate
    // floods the bus (50 identical frames/s at DT=20) and has been observed
    // to make the board stop replying to read commands.
    const float delta_deg = _des_deg - _cmd_deg;
    if (fabsf(delta_deg) * steps_per_deg < 1.0f) {
        return; // target unchanged: do not resend
    }
    _abs_pulse += delta_deg * steps_per_deg;

    const uint32_t clk_abs = (uint32_t)(fabsf(_abs_pulse) + 0.5f);

    const uint8_t dir = (_abs_pulse < 0.0f) ? 1 : 0;  // 0=CW, 1=CCW
    const uint16_t vel = constrain_int16(_vel_rpm.get(), 0, 5000);
    const uint8_t acc = constrain_int16(_acc.get(), 0, 255);

    // absolute position command (raF = true)
    Emm_V5_Pos_Control(_addr, dir, vel, acc, clk_abs, true, false);
    _cmd_deg = _des_deg;
}

/*
  Emm_V5 checksum byte for the configured checksum mode.
  len is the number of bytes before the checksum byte.
 */
uint8_t AP_StepMotor::checksum_byte(const uint8_t *cmd, uint8_t len) const
{
    switch ((ChecksumType)_checksum_type.get()) {
    case CHECKSUM_XOR: {
        uint8_t x = cmd[0];
        for (uint8_t i = 1; i < len; i++) {
            x ^= cmd[i];
        }
        return x;
    }
    case CHECKSUM_CRC8: {
        static const uint8_t crc8Table[256] = {
            0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
            0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
            0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
            0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
            0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
            0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
            0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
            0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
            0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
            0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
            0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
            0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
            0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
            0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
            0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
            0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
            0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
            0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
            0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
            0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
            0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
            0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
            0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
            0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
            0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
            0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
            0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
            0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
            0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
            0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
            0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
            0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
        };
        uint8_t crc = cmd[0];
        for (uint8_t i = 1; i < len; i++) {
            crc = crc8Table[crc ^ cmd[i]];
        }
        return crc;
    }
    default:
        return 0x6B;
    }
}

bool AP_StepMotor::checksum_ok(const uint8_t *frame, uint8_t len) const
{
    return frame[len-1] == checksum_byte(frame, len-1);
}

/*
  write a command, checking that the TX buffer can take it
 */
bool AP_StepMotor::write_cmd(const uint8_t *cmd, uint8_t len)
{
    if (_uart == nullptr || _uart->txspace() < len) {
        _tx_fail++;
        return false;
    }
    _uart->write(cmd, len);
    _tx_bytes += len;
    return true;
}

/**
  * @brief    位置模式控制
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  */
void AP_StepMotor::Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[13];

    cmd[0]  = addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24);
    cmd[7]  = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);
    cmd[9]  = (uint8_t)(clk >> 0);
    cmd[10] = raF;
    cmd[11] = snF;
    cmd[12] = checksum_byte(cmd, 12);

    write_cmd(cmd, sizeof(cmd));
}

/**
  * @brief    速度模式控制（未在本驱动中使用，保留给任务脚本/后续扩展）
  * @param    addr：电机地址
  * @param    dir ：方向，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)，范围0 - 5000RPM
  * @param    acc ：加速度，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志
  */
void AP_StepMotor::Emm_V5_Speed_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[8];

    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc;
    cmd[6] = snF;
    cmd[7] = checksum_byte(cmd, 7);

    write_cmd(cmd, sizeof(cmd));
}

/**
  * @brief    立即停止（紧急刹车），速度/位置模式均有效
  */
void AP_StepMotor::Emm_V5_Stop(uint8_t addr, bool snF)
{
    uint8_t cmd[5];

    cmd[0] = addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF;
    cmd[4] = checksum_byte(cmd, 4);

    write_cmd(cmd, sizeof(cmd));
}

/**
  * @brief    电机使能控制
  */
void AP_StepMotor::Emm_V5_Enable(uint8_t addr, bool en)
{
    uint8_t cmd[6];

    cmd[0] = addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = en;
    cmd[4] = 0;      // 多机同步标志
    cmd[5] = checksum_byte(cmd, 5);

    write_cmd(cmd, sizeof(cmd));
}

/**
  * @brief    读取系统参数
  */
void AP_StepMotor::Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[4];

    cmd[i] = addr; ++i;

    switch (s) {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_TPOS_CUR : cmd[i] = 0x34; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: return;
    }

    cmd[i] = checksum_byte(cmd, i); ++i;

    write_cmd(cmd, i);
}

/*
  expected total frame length (including address, function code and
  checksum) for response frames we understand, 0 for unknown codes
 */
uint8_t AP_StepMotor::expected_frame_len(uint8_t fn) const
{
    switch (fn) {
    case 0xFD:  // position command ack / reached notification
    case 0xF3:  // enable ack
    case 0xF6:  // speed command ack
    case 0xFE:  // stop ack
        return 4;
    case 0x32:  // input pulse count, signed 32 bit
    case 0x33:  // target position, signed 32 bit
    case 0x34:  // current target position, signed 32 bit
    case 0x36:  // current position, signed 32 bit
    case 0x37:  // position error, signed 32 bit
        return 8;
    case 0x35:  // current speed, signed 16 bit
        return 6;
    case 0x43:  // system state (measured on Emm42_V5.0: 31 bytes)
        return 31;
    default:
        return 0;
    }
}

/*
  non-blocking receive parser. Frames have no length field, so the
  expected length is derived from the function code. Unknown function
  codes resynchronise on the next address byte.
 */
void AP_StepMotor::read_incoming(void)
{
    // if we are mid-frame and no further byte arrives in time the frame
    // is incomplete (bytes lost or garbled): resynchronise on the next
    // address byte so the following frame is not swallowed
    if (_parse_state != PARSE_WAIT_ADDR &&
        (AP_HAL::millis() - _last_byte_ms) > STEP_MOTOR_INTERBYTE_MS) {
        _frames_bad++;
        _parse_state = PARSE_WAIT_ADDR;
    }
    while (_uart->available() > 0) {
        const uint8_t b = _uart->read();
        _rx_bytes++;
        _last_rx_ms = AP_HAL::millis();
        _last_byte_ms = _last_rx_ms;
        switch (_parse_state) {
        case PARSE_WAIT_ADDR:
            if (b == (uint8_t)_addr.get()) {
                _rxbuf[0] = b;
                _parse_state = PARSE_WAIT_FN;
            }
            break;

        case PARSE_WAIT_FN:
            _rxbuf[1] = b;
            _rxlen = expected_frame_len(b);
            if (_rxlen == 0 || _rxlen > sizeof(_rxbuf)) {
                _parse_state = PARSE_WAIT_ADDR; // unknown frame, resync
                _frames_bad++;
            } else {
                _rxidx = 2;
                _parse_state = PARSE_WAIT_DATA;
            }
            break;

        case PARSE_WAIT_DATA:
            _rxbuf[_rxidx++] = b;
            if (_rxidx >= _rxlen) {
                if (checksum_ok(_rxbuf, _rxlen)) {
                    handle_frame();
                    _frames_ok++;
                } else {
                    _frames_bad++;
                }
                _parse_state = PARSE_WAIT_ADDR;
            }
            break;
        }
    }
}

// decode a sign-byte + uint32 angle field (value * 360/65536 deg)
static float emm_angle(const uint8_t *p)
{
    const uint32_t v = ((uint32_t)p[1] << 24) | ((uint32_t)p[2] << 16) |
                       ((uint32_t)p[3] << 8)  | ((uint32_t)p[4] << 0);
    const float deg = (float)v * 360.0f / 65536.0f;
    return p[0] ? -deg : deg;
}

/*
  handle a verified response frame
 */
void AP_StepMotor::handle_frame(void)
{
    const uint8_t fn = _rxbuf[1];

    switch (fn) {
    case 0xFD:  // command status: 0x02 ok, 0x9F reached, 0xE2 condition fail
    case 0xF3:
    case 0xF6:
    case 0xFE:
        _last_status = _rxbuf[2];
        if (_rxbuf[2] == 0xE2) {
            // motor not enabled or stall protection active
            const uint32_t now = AP_HAL::millis();
            if ((now - _last_err_msg_ms) > 1000) {
                _last_err_msg_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "StepMotor: command rejected (not enabled/stall)");
            }
        }
        break;

    case 0x36: { // motor real-time position
        const uint32_t pos = ((uint32_t)_rxbuf[3] << 24) |
                             ((uint32_t)_rxbuf[4] << 16) |
                             ((uint32_t)_rxbuf[5] << 8)  |
                             ((uint32_t)_rxbuf[6] << 0);
        float deg = (float)pos * 360.0f / 65536.0f;
        if (_rxbuf[2]) {
            deg = -deg;
        }
        _cur_deg = deg;
        _feedback_valid = true;
        _last_feedback_ms = AP_HAL::millis();
        if (!_cmd_deg_valid) {
            _cmd_deg = deg;
            _cmd_deg_valid = true;
        }
        break;
    }

    case 0x43: { // system state (31 bytes, layout measured on Emm42_V5.0)
        // [2]  u16 raw field preceding bus voltage (likely fw version)
        // [4]  u16 bus voltage [mV]
        // [6]  u16 phase current [mA]
        // [8]  u16 calibrated encoder value
        // [10] sign + u32 target position
        // [15] sign + u16 real-time velocity [RPM]
        // [18] sign + u32 real-time position
        // [23] sign + u32 position error
        // [28] ready flags: bit0 encoder, bit1 cal table, bit2 homing, bit3 home failed
        // [29] motor flags: bit0 enabled, bit1 in position, bit2 stalled, bit3 stall protection
        _sys_version = ((uint16_t)_rxbuf[2] << 8) | _rxbuf[3];
        _bus_mv      = ((uint16_t)_rxbuf[4] << 8) | _rxbuf[5];
        _phase_ma    = ((uint16_t)_rxbuf[6] << 8) | _rxbuf[7];
        _enc_cal     = ((uint16_t)_rxbuf[8] << 8) | _rxbuf[9];
        _tpos_deg    = emm_angle(&_rxbuf[10]);
        {
            const uint16_t vel = ((uint16_t)_rxbuf[16] << 8) | _rxbuf[17];
            _cur_vel_rpm = _rxbuf[15] ? -(float)vel : (float)vel;
        }
        _cur_deg     = emm_angle(&_rxbuf[18]);
        _pos_err_deg = emm_angle(&_rxbuf[23]);
        _flags_ready = _rxbuf[28];
        _flags_motor = _rxbuf[29];
        _feedback_valid = true;
        _last_feedback_ms = AP_HAL::millis();
        if (!_cmd_deg_valid) {
            _cmd_deg = _cur_deg;
            _cmd_deg_valid = true;
        }
        break;
    }

    case 0x35: { // motor real-time speed, signed 16 bit [RPM]
        const uint16_t vel = ((uint16_t)_rxbuf[3] << 8) | _rxbuf[4];
        _cur_vel_rpm = _rxbuf[2] ? -(float)vel : (float)vel;
        break;
    }

    case 0x33: // motor target position, signed 32 bit
    case 0x34: // current target position being executed, signed 32 bit
    case 0x37: { // position error, signed 32 bit
        const uint32_t raw = ((uint32_t)_rxbuf[3] << 24) |
                             ((uint32_t)_rxbuf[4] << 16) |
                             ((uint32_t)_rxbuf[5] << 8)  |
                             ((uint32_t)_rxbuf[6] << 0);
        const float deg = (float)raw * 360.0f / 65536.0f;
        if (fn == 0x33) {
            _tpos_deg = _rxbuf[2] ? -deg : deg;
        } else if (fn == 0x34) {
            _tpos_cur_deg = _rxbuf[2] ? -deg : deg;
        } else {
            _pos_err_deg = _rxbuf[2] ? -deg : deg;
        }
        break;
    }

    default:
        break;
    }
}

bool AP_StepMotor::healthy(void) const
{
    return _initialised && _feedback_valid &&
           ((AP_HAL::millis() - _last_feedback_ms) < STEP_MOTOR_FB_TIMEOUT_MS);
}

#if HAL_LOGGING_ENABLED
/*
  write a STPM telemetry packet with the last decoded feedback values
 */
void AP_StepMotor::Log_Write(void)
{
    if (!_feedback_valid) {
        return;
    }
    const struct log_StepMotor pkt{
        LOG_PACKET_HEADER_INIT(LOG_STEPMOTOR_MSG),
        time_us: AP_HAL::micros64(),
        cpos: _cur_deg,
        tpos: _tpos_deg,
        cur_tpos: _tpos_cur_deg,
        vel: _cur_vel_rpm,
        perr: _pos_err_deg
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif

AP_StepMotor *AP_StepMotor::get_singleton()
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
