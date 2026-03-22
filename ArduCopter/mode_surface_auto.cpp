#include "Copter.h"

#if MODE_SURFACE_ENABLED

/*
 * Surface Auto mode — mission waypoint navigation for quad+boat hybrid on water.
 *
 * The quad motors idle at GROUND_IDLE.  The boat's propulsion follows ArduCopter
 * mission NAV_WAYPOINT / NAV_LOITER_UNLIM commands:
 *   Boat throttle : SRV_Channel::k_throttle (0..100 %, SERVOx_FUNCTION = 70)
 *   Steering servo: SRV_Channel::k_steering (±4500 cdeg, SERVOx_FUNCTION = 26)
 *
 * Only MAV_CMD_NAV_WAYPOINT and MAV_CMD_NAV_LOITER_UNLIM commands are acted on.
 * All other nav commands are skipped.  The mode stops when the last waypoint is
 * reached.  A NAV_LOITER_UNLIM waypoint causes the boat to loiter (on/off
 * throttle) at that position indefinitely.
 *
 * Heading controller (P):
 *   steer = wrap_PI(bearing_to_target - current_yaw) × (4500 / π/2) × SURF_HEAD_KP
 *
 * Throttle (distance ramp):
 *   • > 3×WPNAV_RADIUS  : SURF_AUTO_SPD %
 *   • WPNAV_RADIUS .. 3× : linear ramp down to 0
 *   • ≤ WPNAV_RADIUS    : 0 (coast to rest / natural loiter)
 */

// Scaled output limit for the steering servo (centidegrees)
static constexpr float SURFACE_STEERING_MAX = 4500.0f;

// surface_auto_init - validate mission and load first waypoint
bool ModeSurfaceAuto::init(bool ignore_checks)
{
    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }

    if (!copter.mode_auto.mission.present()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "SurfAuto: no mission loaded");
        return false;
    }

    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);
    SRV_Channels::set_angle(SRV_Channel::k_steering, SURFACE_STEERING_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);

    _mission_complete = false;
    _loiter_at_target = false;

    if (!advance_to_next_wp(0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "SurfAuto: no nav waypoints in mission");
        return false;
    }

    return true;
}

// surface_auto_run - run the surface auto controller
// should be called at 100 Hz or more
void ModeSurfaceAuto::run()
{
    // Quad motors stay at ground idle — no thrust from rotors
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;
    default:
        break;
    }

    // Maintain level attitude with zero quad-motor thrust
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, g.throttle_filt);

    if (_mission_complete) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);
        return;
    }

    const float dist_m   = copter.current_loc.get_distance(_wp_target);
    const float accept_m = wp_nav->get_wp_radius_m();

    // Inside acceptance radius: advance to next waypoint (unless loitering)
    if (dist_m <= accept_m && !_loiter_at_target) {
        if (!advance_to_next_wp(_cmd_index + 1)) {
            // No more waypoints — coast to a stop
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
            SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);
            return;
        }
    }

    // --- Bearing-based navigation to current waypoint ---
    const float bearing_rad     = copter.current_loc.get_bearing(_wp_target);
    const float heading_err_rad = wrap_PI(bearing_rad - ahrs.get_yaw_rad());
    const float steer = constrain_float(
        heading_err_rad * (SURFACE_STEERING_MAX / M_PI_2) * g2.surface_head_kp,
        -SURFACE_STEERING_MAX, SURFACE_STEERING_MAX);

    // Throttle: ramp down within 3× acceptance radius, coast inside
    const float slow_m = 3.0f * accept_m;
    float thr = 0.0f;
    if (dist_m > slow_m) {
        thr = g2.surface_auto_spd;
    } else if (dist_m > accept_m) {
        thr = g2.surface_auto_spd * (dist_m - accept_m) / (slow_m - accept_m);
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, constrain_float(thr, 0.0f, 100.0f));
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steer);
}

// surface_auto_exit - neutral outputs on exit
void ModeSurfaceAuto::exit()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);
}

// advance_to_next_wp - scan the mission from start_idx for the next
// actionable nav command and update _wp_target, _cmd_index, _loiter_at_target.
// Returns true if a waypoint was found, false if mission is complete.
bool ModeSurfaceAuto::advance_to_next_wp(uint16_t start_idx)
{
    AP_Mission::Mission_Command cmd;
    uint16_t search_idx = start_idx;

    while (copter.mode_auto.mission.get_next_nav_cmd(search_idx, cmd)) {
        if (cmd.id == MAV_CMD_NAV_WAYPOINT || cmd.id == MAV_CMD_NAV_LOITER_UNLIM) {
            Location target = cmd.content.location;
            // Zero lat/lng in a mission command means "use current position"
            if (target.lat == 0 && target.lng == 0) {
                target = copter.current_loc;
            }
            _wp_target       = target;
            _cmd_index       = cmd.index;
            _loiter_at_target = (cmd.id == MAV_CMD_NAV_LOITER_UNLIM);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SurfAuto: navigate to WP %u", (unsigned)_cmd_index);
            return true;
        }
        // Skip non-position nav commands (e.g. NAV_TAKEOFF, NAV_LAND)
        search_idx = cmd.index + 1;
    }

    _mission_complete = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SurfAuto: mission complete");
    return false;
}

#endif  // MODE_SURFACE_ENABLED
