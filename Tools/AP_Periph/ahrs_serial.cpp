#include "AP_Periph.h"

#if AP_PERIPH_AHRS_SERIAL_ENABLED

#include <AP_Math/crc.h>

extern const AP_HAL::HAL &hal;

/*
  APAS (ArduPilot AHRS Serial) protocol output
  Frame format:
    SYNC1 (0xA3), SYNC2 (0x95), TYPE (1 byte), LEN (1 byte), PAYLOAD (LEN bytes), CRC_HI, CRC_LO
    CRC-16/CCITT over TYPE+LEN+PAYLOAD
 */

static const uint8_t APAS_SYNC1 = 0xA3;
static const uint8_t APAS_SYNC2 = 0x95;

enum class APASMsgType : uint8_t {
    ATTITUDE = 0x01,
    IMU      = 0x02,
    GPS      = 0x03,
    BARO     = 0x04,
    MAG      = 0x05,
};

struct PACKED apas_attitude_t {
    float q1, q2, q3, q4;   // quaternion w,x,y,z
};

struct PACKED apas_imu_t {
    float accel[3];          // m/s^2
    float gyro[3];           // rad/s
    float temperature;       // degrees C
};

struct PACKED apas_gps_t {
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

struct PACKED apas_baro_t {
    float pressure;          // Pa
    float temperature;       // degrees C
};

struct PACKED apas_mag_t {
    float field[3];          // milliGauss
};

/*
  send one APAS frame on uart
 */
static void apas_send(AP_HAL::UARTDriver *uart, APASMsgType type,
                      const void *payload, uint8_t len)
{
    uint8_t hdr[2] = { uint8_t(type), len };
    uint16_t crc = crc16_ccitt(hdr, 2, 0);
    crc = crc16_ccitt((const uint8_t *)payload, len, crc);

    uart->write(APAS_SYNC1);
    uart->write(APAS_SYNC2);
    uart->write(uint8_t(type));
    uart->write(len);
    uart->write((const uint8_t *)payload, len);
    uart->write(uint8_t(crc >> 8));
    uart->write(uint8_t(crc & 0xFF));
}

/*
  AHRS serial output thread - runs at ~50 Hz
 */
void AP_Periph_FW::can_ahrs_serial_update(void)
{
    if (g.ahrs_serial_port < 0) {
        return;
    }

    AP_HAL::UARTDriver *uart = hal.serial(g.ahrs_serial_port);
    if (uart == nullptr) {
        return;
    }
    uart->begin(115200, 256, 256);

    uint32_t last_att_ms  = 0;
    uint32_t last_imu_ms  = 0;
    uint32_t last_gps_ms  = 0;
    uint32_t last_baro_ms = 0;
    uint32_t last_mag_ms  = 0;

    while (true) {
        hal.scheduler->delay(10); // 100Hz loop
        const uint32_t now_ms = AP_HAL::millis();

#if AP_AHRS_ENABLED
        // Attitude at 50Hz
        if (now_ms - last_att_ms >= 20) {
            last_att_ms = now_ms;
            Quaternion q;
            if (ahrs.get_quaternion(q)) {
                apas_attitude_t att;
                att.q1 = q.q1;
                att.q2 = q.q2;
                att.q3 = q.q3;
                att.q4 = q.q4;
                apas_send(uart, APASMsgType::ATTITUDE, &att, sizeof(att));
            }
        }
#endif

#if AP_PERIPH_IMU_ENABLED
        // IMU at 50Hz
        if (now_ms - last_imu_ms >= 20) {
            last_imu_ms = now_ms;
            if (imu.healthy()) {
                apas_imu_t pkt;
                const Vector3f &accel = imu.get_accel();
                const Vector3f &gyro  = imu.get_gyro();
                pkt.accel[0] = accel.x;
                pkt.accel[1] = accel.y;
                pkt.accel[2] = accel.z;
                pkt.gyro[0]  = gyro.x;
                pkt.gyro[1]  = gyro.y;
                pkt.gyro[2]  = gyro.z;
                pkt.temperature = imu.get_temperature(0);
                apas_send(uart, APASMsgType::IMU, &pkt, sizeof(pkt));
            }
        }
#endif

#if AP_GPS_ENABLED
        // GPS at 5Hz
        if (now_ms - last_gps_ms >= 200) {
            last_gps_ms = now_ms;
            if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
                const Location &loc = gps.location();
                const Vector3f &vel = gps.velocity();
                apas_gps_t pkt {};
                pkt.latitude  = loc.lat;
                pkt.longitude = loc.lng;
                pkt.altitude  = loc.alt;
                pkt.vel_north = vel.x;
                pkt.vel_east  = vel.y;
                pkt.vel_down  = vel.z;
                float hacc = 0, vacc = 0;
                gps.horizontal_accuracy(hacc);
                gps.vertical_accuracy(vacc);
                pkt.horiz_acc = hacc;
                pkt.vert_acc  = vacc;
                pkt.fix_type  = uint8_t(gps.status());
                pkt.num_sats  = gps.num_sats();
                pkt.hdop      = gps.get_hdop();
                apas_send(uart, APASMsgType::GPS, &pkt, sizeof(pkt));
            }
        }
#endif

#if AP_PERIPH_BARO_ENABLED
        // Baro at 10Hz
        if (now_ms - last_baro_ms >= 100) {
            last_baro_ms = now_ms;
            if (baro.healthy()) {
                apas_baro_t pkt;
                pkt.pressure    = baro.get_pressure();
                pkt.temperature = baro.get_temperature();
                apas_send(uart, APASMsgType::BARO, &pkt, sizeof(pkt));
            }
        }
#endif

        // Magnetometer at 10Hz
        if (now_ms - last_mag_ms >= 100) {
            last_mag_ms = now_ms;
            if (compass.healthy()) {
                const Vector3f &field = compass.get_field();
                apas_mag_t pkt;
                pkt.field[0] = field.x;
                pkt.field[1] = field.y;
                pkt.field[2] = field.z;
                apas_send(uart, APASMsgType::MAG, &pkt, sizeof(pkt));
            }
        }
    }
}

#endif  // AP_PERIPH_AHRS_SERIAL_ENABLED
