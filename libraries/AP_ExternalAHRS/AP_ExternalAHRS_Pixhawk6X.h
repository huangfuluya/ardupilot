/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for Pixhawk6X used as a serial External AHRS source
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_Pixhawk6X : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_Pixhawk6X(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data
    void update() override {
        check_uart();
    }

    // Get model/type name
    const char* get_name() const override {
        return "Pixhawk6X";
    }

protected:
    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    bool setup_complete;
    uint32_t baudrate;

    // time of last valid messages
    uint32_t last_att_ms;
    uint32_t last_gps_ms;
    uint32_t last_baro_ms;
    uint32_t last_imu_ms;

    // protocol sync bytes
    static const uint8_t APAS_SYNC1 = 0xA3;
    static const uint8_t APAS_SYNC2 = 0x95;

    // message types
    enum class MsgType : uint8_t {
        ATTITUDE = 0x01,
        IMU      = 0x02,
        GPS      = 0x03,
        BARO     = 0x04,
        MAG      = 0x05,
    };

    // payload structures (packed for serial transfer)
    struct PACKED attitude_payload_t {
        float q1, q2, q3, q4;   // quaternion w,x,y,z
    };

    struct PACKED imu_payload_t {
        float accel[3];          // m/s^2
        float gyro[3];           // rad/s
        float temperature;       // degrees C
    };

    struct PACKED gps_payload_t {
        int32_t latitude;        // 1e7 degrees
        int32_t longitude;       // 1e7 degrees
        int32_t altitude;        // cm MSL
        float vel_north;         // m/s
        float vel_east;          // m/s
        float vel_down;          // m/s
        float horiz_acc;         // m
        float vert_acc;          // m
        uint8_t fix_type;
        uint8_t num_sats;
        uint16_t hdop;           // hdop * 100
    };

    struct PACKED baro_payload_t {
        float pressure;          // Pa
        float temperature;       // degrees C
    };

    struct PACKED mag_payload_t {
        float field[3];          // milliGauss
    };

    // parse buffer
    static const uint8_t MAX_PAYLOAD_LEN = 64;
    uint8_t buffer[MAX_PAYLOAD_LEN + 6];
    uint8_t buffer_ofs;

    bool check_uart();
    bool parse_byte(uint8_t b);
    bool process_message(MsgType type, const uint8_t *payload, uint8_t len);

    void update_thread();

    // buffer parse state
    enum class ParseState : uint8_t {
        SYNC1 = 0,
        SYNC2,
        TYPE,
        LEN,
        PAYLOAD,
        CRC1,
        CRC2,
    };
    ParseState parse_state;
    MsgType parse_type;
    uint8_t parse_len;
    uint8_t parse_ofs;
};

#endif  // AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED
