#include "AP_Periph.h"

#if AP_PERIPH_AHRS_DRONECAN_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  AHRS DroneCAN output thread.

  Broadcasts at ~50 Hz:
    - uavcan.navigation.GlobalNavigationSolution  (attitude + IMU + position + velocity, preferred)
    - uavcan.equipment.ahrs.Solution              (attitude + IMU, secondary)
    - uavcan.equipment.ahrs.RawIMU                (raw gyro + accel, fallback)
*/
void AP_Periph_FW::can_ahrs_dronecan_update(void)
{
    uint32_t last_send_ms = 0;

    while (true) {
        hal.scheduler->delay(10);
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_send_ms < 20) {
            continue;
        }
        last_send_ms = now_ms;

#if AP_AHRS_ENABLED
        Quaternion q;
        if (!ahrs.get_quaternion(q)) {
            continue;
        }

        const Vector3f &gyro  = ahrs.get_gyro();
#if AP_PERIPH_IMU_ENABLED
        const Vector3f &accel = imu.get_accel();
#endif

        // ------------------------------------------------------------------
        // 1. uavcan.navigation.GlobalNavigationSolution  (preferred)
        //    Fill position + velocity from GPS when available.
        // ------------------------------------------------------------------
        {
            uavcan_navigation_GlobalNavigationSolution gns {};
            gns.orientation_xyzw[0] = q.q2;  // x
            gns.orientation_xyzw[1] = q.q3;  // y
            gns.orientation_xyzw[2] = q.q4;  // z
            gns.orientation_xyzw[3] = q.q1;  // w

            gns.angular_velocity_body[0] = gyro.x;
            gns.angular_velocity_body[1] = gyro.y;
            gns.angular_velocity_body[2] = gyro.z;

#if AP_PERIPH_IMU_ENABLED
            gns.linear_acceleration_body[0] = accel.x;
            gns.linear_acceleration_body[1] = accel.y;
            gns.linear_acceleration_body[2] = accel.z;
#endif

#if AP_GPS_ENABLED
            if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
                const Location &loc = gps.location();
                gns.latitude         = loc.lat * 1.0e-7;
                gns.longitude        = loc.lng * 1.0e-7;
                gns.height_msl       = loc.alt * 0.01f;   // cm → m
                gns.height_ellipsoid = gns.height_msl;

                const Vector3f &vel  = gps.velocity();
                // Rotate NED velocity to body frame using quaternion earth_to_body()
                Vector3f body_vel = vel;
                q.earth_to_body(body_vel);
                gns.linear_velocity_body[0] = body_vel.x;
                gns.linear_velocity_body[1] = body_vel.y;
                gns.linear_velocity_body[2] = body_vel.z;
            } else {
                gns.latitude  = nanf("");
                gns.longitude = nanf("");
                gns.height_msl = nanf("");
                gns.height_ellipsoid = nanf("");
            }
#else
            gns.latitude  = nanf("");
            gns.longitude = nanf("");
            gns.height_msl = nanf("");
            gns.height_ellipsoid = nanf("");
#endif  // AP_GPS_ENABLED

            uint8_t buffer[UAVCAN_NAVIGATION_GLOBALNAVIGATIONSOLUTION_MAX_SIZE];
            uint16_t total_size = uavcan_navigation_GlobalNavigationSolution_encode(&gns, buffer, !canfdout());
            canard_broadcast(UAVCAN_NAVIGATION_GLOBALNAVIGATIONSOLUTION_SIGNATURE,
                             UAVCAN_NAVIGATION_GLOBALNAVIGATIONSOLUTION_ID,
                             CANARD_TRANSFER_PRIORITY_HIGH,
                             &buffer[0],
                             total_size);
        }

        // ------------------------------------------------------------------
        // 2. uavcan.equipment.ahrs.Solution  (secondary: attitude + IMU)
        // ------------------------------------------------------------------
        {
            uavcan_equipment_ahrs_Solution sol {};
            sol.orientation_xyzw[0] = q.q2;  // x
            sol.orientation_xyzw[1] = q.q3;  // y
            sol.orientation_xyzw[2] = q.q4;  // z
            sol.orientation_xyzw[3] = q.q1;  // w

            sol.angular_velocity[0] = gyro.x;
            sol.angular_velocity[1] = gyro.y;
            sol.angular_velocity[2] = gyro.z;

#if AP_PERIPH_IMU_ENABLED
            sol.linear_acceleration[0] = accel.x;
            sol.linear_acceleration[1] = accel.y;
            sol.linear_acceleration[2] = accel.z;
#endif

            uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_SOLUTION_MAX_SIZE];
            uint16_t total_size = uavcan_equipment_ahrs_Solution_encode(&sol, buffer, !canfdout());
            canard_broadcast(UAVCAN_EQUIPMENT_AHRS_SOLUTION_SIGNATURE,
                             UAVCAN_EQUIPMENT_AHRS_SOLUTION_ID,
                             CANARD_TRANSFER_PRIORITY_HIGH,
                             &buffer[0],
                             total_size);
        }

        // ------------------------------------------------------------------
        // 3. uavcan.equipment.ahrs.RawIMU  (fallback: raw gyro + accel)
        // ------------------------------------------------------------------
#if AP_PERIPH_IMU_ENABLED
        {
            uavcan_equipment_ahrs_RawIMU raw {};
            raw.rate_gyro_latest[0] = gyro.x;
            raw.rate_gyro_latest[1] = gyro.y;
            raw.rate_gyro_latest[2] = gyro.z;

            raw.accelerometer_latest[0] = accel.x;
            raw.accelerometer_latest[1] = accel.y;
            raw.accelerometer_latest[2] = accel.z;

            uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_RAWIMU_MAX_SIZE];
            uint16_t total_size = uavcan_equipment_ahrs_RawIMU_encode(&raw, buffer, !canfdout());
            canard_broadcast(UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE,
                             UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID,
                             CANARD_TRANSFER_PRIORITY_HIGH,
                             &buffer[0],
                             total_size);
        }
#endif  // AP_PERIPH_IMU_ENABLED

#endif  // AP_AHRS_ENABLED
    }
}

#endif  // AP_PERIPH_AHRS_DRONECAN_ENABLED
