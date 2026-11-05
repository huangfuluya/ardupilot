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
  simulate a payload slung from a line under a vehicle

  Also supports a payload shared by two vehicles (cooperative lift):
  MODE=1 is the master (runs the payload dynamics for two lines and owns the
  payload state), MODE=2 is the slave (sends its position/velocity to the
  master and receives its share of the tether force).  The two SITL processes
  exchange small UDP datagrams on the loopback interface.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SLUNGPAYLOAD_ENABLED

#include <AP_HAL/utility/Socket_native.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

namespace SITL {

// SlungPayloadSim handles interaction with main vehicle
class SlungPayloadSim {
public:
    friend class SlungPayload;

    // constructor
    SlungPayloadSim();

    // update the SlungPayloadSim's state using thevehicle's earth-frame position, velocity, acceleration and wind
    void update(const Vector3p& veh_pos, const Vector3f& veh_vel_ef, const Vector3f& veh_accel_ef, const Vector3f& wind_ef);

    // get earth-frame forces on the vehicle from slung payload
    // returns true on success and fills in forces_ef argument, false on failure
    bool get_forces_on_vehicle(Vector3f& forces_ef) const;

    // parameter table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8 enable;         // enable parameter
    AP_Float weight_kg;        // payload weight in kg
    AP_Float line_length;   // line length in meters
    AP_Int8 sys_id;         // mavlink system id for reporting to GCS
    AP_Float drag_coef;     // drag coefficient (spheres=0.5, cubes=1.05, barrels=0.8~1.2)

    // cooperative lift (two vehicles sharing one payload)
    AP_Int8 mode;           // 0: single vehicle, 1: dual master, 2: dual slave
    AP_Int16 peer_port;     // master binds this port, slave sends state here
    AP_Float line2_length;  // second line length (0 = same as line_length)
    AP_Float peer_ofs_n;    // peer home in this vehicle's NED frame (m)
    AP_Float peer_ofs_e;

    // send MAVLink messages to GCS
    void send_report();

    // write onboard log
    void write_log();

    // get payload location
    // returns true on success and fills in payload_loc argument, false on failure
    bool get_payload_location(Location& payload_loc) const;

    // update the slung payload's position, velocity, acceleration
    // vehicle position, velocity, acceleration and wind should be in earth-frame NED frame
    void update_payload(const Vector3p& veh_pos, const Vector3f& veh_vel_ef, const Vector3f& veh_accel_ef,
                        const Vector3f& wind_ef, float dt);

    // two-vehicle payload dynamics (master only)
    void update_payload_dual(const Vector3p& veh_pos, const Vector3f& veh_vel_ef,
                             const Vector3f& wind_ef, float dt);

    // peer link (loopback UDP between the two SITL processes)
    void open_peer_socket();
    void peer_poll();
    void peer_send_state(const Vector3p& veh_pos, const Vector3f& veh_vel_ef);
    void peer_send_force(const Vector3f& force_on_peer);
    bool is_master() const { return mode.get() == 1; }
    bool is_slave() const { return mode.get() == 2; }
    bool is_dual() const { return mode.get() != 0; }

    // returns true if the two vectors point in the same direction, false if perpendicular or opposite
    bool vectors_same_direction(const Vector3f& v1, const Vector3f& v2) const;

    // socket connection variables
    const char *target_address = "127.0.0.1";
    const uint16_t target_port = 5763;
    SocketAPM_native mav_socket { false };
    bool initialised;           // true if this class has been initialised
    uint32_t last_update_us;    // system time of last update

    // peer link socket and state
    SocketAPM_native peer_socket { true };
    bool peer_socket_ready;
    bool peer_valid;            // master: received peer state
    bool force_valid;           // slave: received force from master
    uint32_t peer_last_ms;
    Vector3f peer_pos;          // master: slave vehicle position (NED)
    Vector3f peer_vel;          // master: slave vehicle velocity (NED)
    Vector3f peer_force;        // slave: force applied to this vehicle
    float peer_tension1;        // slave: tension ratios for logging
    float peer_tension2;

    // mavlink reporting variables
    const float reporting_period_ms = 100;  // reporting period in ms
    uint32_t last_report_ms;                // system time of last MAVLink report sent to GCS
    uint32_t last_heartbeat_ms;             // system time of last MAVLink heartbeat sent to GCS
    bool mavlink_connected;                 // true if a mavlink connection has been established
    mavlink_status_t mav_status;            // reported mavlink status

    // payload variables
    bool landed = true;     // true if the payload is on the ground
    float tension_ratio;    // 0 if line is loose, 1 if completely taut
    Vector3p payload_to_veh;// distance vector (in meters in NED frame) from payload to vehicle (used for reporting purposes)
    Vector3p position_NED;  // payload's position (as an offset from EKF origin? offset from vehicle?) in meters
    Vector3f velocity_NED;  // payload velocity
    Vector3f accel_NED;     // payload's acceleration
    Vector3f veh_forces_ef; // earth-frame forces on the vehicle caused by the payload
};

}  // namespace SITL

#endif  // AP_SIM_SLUNGPAYLOAD_ENABLED
