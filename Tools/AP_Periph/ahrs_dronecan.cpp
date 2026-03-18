#include "AP_Periph.h"

#if AP_PERIPH_AHRS_DRONECAN_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  AHRS DroneCAN output thread - publishes uavcan_equipment_ahrs_Solution at ~50 Hz
*/
void AP_Periph_FW::can_ahrs_dronecan_update(void)
{
    uint32_t last_solution_ms = 0;

    while (true) {
        hal.scheduler->delay(10);
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_solution_ms < 20) {
            continue;
        }
        last_solution_ms = now_ms;

#if AP_AHRS_ENABLED
        Quaternion q;
        if (!ahrs.get_quaternion(q)) {
            continue;
        }

        uavcan_equipment_ahrs_Solution pkt {};
        // DroneCAN uses XYZW quaternion order
        pkt.orientation_xyzw[0] = q.q2;  // x
        pkt.orientation_xyzw[1] = q.q3;  // y
        pkt.orientation_xyzw[2] = q.q4;  // z
        pkt.orientation_xyzw[3] = q.q1;  // w

        const Vector3f &gyro = ahrs.get_gyro();
        pkt.angular_velocity[0] = gyro.x;
        pkt.angular_velocity[1] = gyro.y;
        pkt.angular_velocity[2] = gyro.z;

        // Linear acceleration from IMU
#if AP_PERIPH_IMU_ENABLED
        const Vector3f &accel = imu.get_accel();
        pkt.linear_acceleration[0] = accel.x;
        pkt.linear_acceleration[1] = accel.y;
        pkt.linear_acceleration[2] = accel.z;
#endif

        uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE];
        uint16_t total_size = uavcan_equipment_ahrs_Solution_encode(&pkt, buffer, !canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_AHRS_SOLUTION_SIGNATURE,
                         UAVCAN_EQUIPMENT_AHRS_SOLUTION_ID,
                         CANARD_TRANSFER_PRIORITY_HIGH,
                         &buffer[0],
                         total_size);
#endif  // AP_AHRS_ENABLED
    }
}

#endif  // AP_PERIPH_AHRS_DRONECAN_ENABLED
