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

  Protocol: APAS (ArduPilot AHRS Serial)
  Frame format:
    SYNC1 (0xA3), SYNC2 (0x95), TYPE (1 byte), LEN (1 byte), PAYLOAD (LEN bytes), CRC_HI, CRC_LO
    CRC is CRC-16/CCITT over TYPE+LEN+PAYLOAD
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED

#include "AP_ExternalAHRS_Pixhawk6X.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

// constructor
AP_ExternalAHRS_Pixhawk6X::AP_ExternalAHRS_Pixhawk6X(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Pixhawk6X ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_Pixhawk6X::update_thread, void),
                                      "P6X_AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Pixhawk6X Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Pixhawk6X ExternalAHRS initialised");
}

// compute CRC16 over TYPE+LEN+PAYLOAD
static uint16_t apas_crc(uint8_t type, uint8_t len, const uint8_t *payload)
{
    uint16_t crc = 0;
    uint8_t hdr[2] = { type, len };
    crc = crc16_ccitt(hdr, 2, crc);
    crc = crc16_ccitt(payload, len, crc);
    return crc;
}

// check uart for new data
bool AP_ExternalAHRS_Pixhawk6X::check_uart()
{
    WITH_SEMAPHORE(state.sem);

    if (!setup_complete) {
        uart->begin(baudrate);
        setup_complete = true;
    }

    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    while (n--) {
        uint8_t b;
        if (uart->read(&b, 1) != 1) {
            break;
        }
        if (parse_byte(b)) {
            // message complete
        }
    }
    return true;
}

// parse one byte from the uart
bool AP_ExternalAHRS_Pixhawk6X::parse_byte(uint8_t b)
{
    switch (parse_state) {
    case ParseState::SYNC1:
        if (b == APAS_SYNC1) {
            parse_state = ParseState::SYNC2;
        }
        break;

    case ParseState::SYNC2:
        if (b == APAS_SYNC2) {
            parse_state = ParseState::TYPE;
        } else {
            parse_state = ParseState::SYNC1;
        }
        break;

    case ParseState::TYPE:
        parse_type = MsgType(b);
        buffer[0] = b; // store TYPE at buffer[0] for CRC
        parse_state = ParseState::LEN;
        break;

    case ParseState::LEN:
        parse_len = b;
        buffer[1] = b; // store LEN at buffer[1] for CRC
        parse_ofs = 0;
        if (parse_len == 0) {
            parse_state = ParseState::CRC1;
        } else if (parse_len > MAX_PAYLOAD_LEN) {
            parse_state = ParseState::SYNC1;
        } else {
            parse_state = ParseState::PAYLOAD;
        }
        break;

    case ParseState::PAYLOAD:
        buffer[2 + parse_ofs] = b;
        parse_ofs++;
        if (parse_ofs >= parse_len) {
            parse_state = ParseState::CRC1;
        }
        break;

    case ParseState::CRC1:
        buffer[2 + parse_len] = b; // store CRC1
        parse_state = ParseState::CRC2;
        break;

    case ParseState::CRC2: {
        // buffer[0]=TYPE, buffer[1]=LEN, buffer[2..2+len-1]=PAYLOAD
        // buffer[2+len]=CRC1, buffer[2+len+1]=CRC2
        const uint16_t expected_crc = apas_crc(buffer[0], buffer[1], &buffer[2]);
        const uint16_t received_crc = uint16_t(buffer[2 + parse_len]) << 8 | b;
        parse_state = ParseState::SYNC1;
        if (expected_crc == received_crc) {
            return process_message(parse_type, &buffer[2], parse_len);
        }
        break;
    }
    }
    return false;
}

// process a complete message
bool AP_ExternalAHRS_Pixhawk6X::process_message(MsgType type, const uint8_t *payload, uint8_t len)
{
    const uint32_t now_ms = AP_HAL::millis();

    switch (type) {
    case MsgType::ATTITUDE: {
        if (len < sizeof(attitude_payload_t)) {
            break;
        }
        const attitude_payload_t &att = *(const attitude_payload_t *)payload;
        state.quat = Quaternion(att.q1, att.q2, att.q3, att.q4);
        state.have_quaternion = true;
        last_att_ms = now_ms;
        return true;
    }

    case MsgType::IMU: {
        if (len < sizeof(imu_payload_t)) {
            break;
        }
        const imu_payload_t &imu = *(const imu_payload_t *)payload;
        state.accel = Vector3f(imu.accel[0], imu.accel[1], imu.accel[2]);
        state.gyro  = Vector3f(imu.gyro[0],  imu.gyro[1],  imu.gyro[2]);
        last_imu_ms = now_ms;

        {
            AP_ExternalAHRS::ins_data_message_t ins;
            ins.accel       = state.accel;
            ins.gyro        = state.gyro;
            ins.temperature = imu.temperature;
            AP::ins().handle_external(ins);
        }
        return true;
    }

    case MsgType::GPS: {
        if (len < sizeof(gps_payload_t)) {
            break;
        }
        const gps_payload_t &gps_pld = *(const gps_payload_t *)payload;
        AP_ExternalAHRS::gps_data_message_t gps {};

        gps.latitude  = gps_pld.latitude;
        gps.longitude = gps_pld.longitude;
        gps.msl_altitude = gps_pld.altitude;

        gps.ned_vel_north = gps_pld.vel_north;
        gps.ned_vel_east  = gps_pld.vel_east;
        gps.ned_vel_down  = gps_pld.vel_down;

        gps.horizontal_pos_accuracy = gps_pld.horiz_acc;
        gps.vertical_pos_accuracy   = gps_pld.vert_acc;

        gps.fix_type           = AP_GPS_FixType(gps_pld.fix_type);
        gps.satellites_in_view = gps_pld.num_sats;
        gps.hdop               = gps_pld.hdop * 0.01f;

        if (!state.have_origin && gps.fix_type >= AP_GPS_FixType::FIX_3D) {
            state.origin = Location{
                gps.latitude,
                gps.longitude,
                gps.msl_altitude,
                Location::AltFrame::ABSOLUTE};
            state.have_origin = true;
        }

        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(gps, instance);
        }

        if (gps.satellites_in_view > 3) {
            last_gps_ms = now_ms;
        }
        return true;
    }

    case MsgType::BARO: {
        if (len < sizeof(baro_payload_t)) {
            break;
        }
        const baro_payload_t &baro_pld = *(const baro_payload_t *)payload;

#if AP_BARO_EXTERNALAHRS_ENABLED
        {
            AP_ExternalAHRS::baro_data_message_t baro;
            baro.instance    = 0;
            baro.pressure_pa = baro_pld.pressure;
            baro.temperature = baro_pld.temperature;
            AP::baro().handle_external(baro);
        }
#endif
        last_baro_ms = now_ms;
        return true;
    }

    case MsgType::MAG: {
        if (len < sizeof(mag_payload_t)) {
            break;
        }
        const mag_payload_t &mag_pld = *(const mag_payload_t *)payload;

#if AP_COMPASS_EXTERNALAHRS_ENABLED
        {
            AP_ExternalAHRS::mag_data_message_t mag;
            mag.field = Vector3f(mag_pld.field[0], mag_pld.field[1], mag_pld.field[2]);
            AP::compass().handle_external(mag);
        }
#endif
        return true;
    }
    }
    return false;
}

// update thread - runs continuously
void AP_ExternalAHRS_Pixhawk6X::update_thread()
{
    while (true) {
        hal.scheduler->delay(1);
        check_uart();
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_Pixhawk6X::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
}

// check if we are healthy
bool AP_ExternalAHRS_Pixhawk6X::healthy(void) const
{
    WITH_SEMAPHORE(state.sem);
    const uint32_t now = AP_HAL::millis();
    return (now - last_att_ms) < 200;
}

// check if we are initialised
bool AP_ExternalAHRS_Pixhawk6X::initialised(void) const
{
    WITH_SEMAPHORE(state.sem);
    return last_att_ms != 0;
}

// pre-arm check
bool AP_ExternalAHRS_Pixhawk6X::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    WITH_SEMAPHORE(state.sem);
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Pixhawk6X unhealthy");
        return false;
    }
    return true;
}

// get filter status
void AP_ExternalAHRS_Pixhawk6X::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    const uint32_t now = AP_HAL::millis();
    const bool att_ok = (now - last_att_ms) < 200;
    const bool gps_ok = (now - last_gps_ms) < 1000;

    memset(&status, 0, sizeof(status));
    status.flags.attitude           = att_ok;
    status.flags.horiz_vel          = gps_ok && att_ok;
    status.flags.vert_vel           = gps_ok && att_ok;
    status.flags.horiz_pos_rel      = gps_ok && att_ok;
    status.flags.horiz_pos_abs      = gps_ok;
    status.flags.vert_pos           = (now - last_baro_ms) < 1000;
    status.flags.pred_horiz_pos_rel = gps_ok;
    status.flags.pred_horiz_pos_abs = gps_ok;
    status.flags.using_gps          = gps_ok;
}

// get variances - not available
bool AP_ExternalAHRS_Pixhawk6X::get_variances(float &velVar, float &posVar, float &hgtVar,
                                              Vector3f &magVar, float &tasVar) const
{
    return false;
}

#endif  // AP_EXTERNAL_AHRS_PIXHAWK6X_ENABLED
