#include "Copter.h"

#if MODE_SURFACE_ENABLED

/*
 * Surface Loiter mode — GPS position hold for quad+boat hybrid vehicle on water.
 *
 * The quad motors idle at GROUND_IDLE while the boat's propulsion holds position:
 *   Boat throttle : SRV_Channel::k_throttle (0..100 %, SERVOx_FUNCTION = 70)
 *   Steering servo: SRV_Channel::k_steering (±4500 cdeg, SERVOx_FUNCTION = 26)
 *
 * On entry the loiter target is set to the current GPS position.  The pilot
 * can shift the target with roll/pitch sticks (body-frame, rotated to NE).
 *
 * Heading controller (P):
 *   steer = wrap_PI(bearing_to_target - current_yaw) × (4500 / π/2) × SURF_HEAD_KP
 *   clamped to ±4500 centidegrees.
 *
 * Throttle (distance ramp):
 *   • > 3×WPNAV_RADIUS  : full SURF_AUTO_SPD %
 *   • WPNAV_RADIUS .. 3× : linear ramp down to 0
 *   • ≤ WPNAV_RADIUS    : 0 (coast to rest inside target circle)
 */

// Scaled output limit for the steering servo (centidegrees)
static constexpr float SURFACE_STEERING_MAX = 4500.0f;
// Target-move rate when pilot applies roll/pitch sticks (m/s)
static constexpr float SURF_LOITER_MOVE_RATE_MS = 2.0f;

// surface_loiter_init - capture current GPS position as the loiter target
bool ModeSurfaceLoiter::init(bool ignore_checks)
{
    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }

    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);
    SRV_Channels::set_angle(SRV_Channel::k_steering, SURFACE_STEERING_MAX);

    // Neutral outputs on entry
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);

    _loiter_target = copter.current_loc;
    return true;
}

// surface_loiter_run - runs the surface loiter controller
// should be called at 100 Hz or more
void ModeSurfaceLoiter::run()
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

    // Allow pilot to nudge the loiter target via roll/pitch sticks
    update_loiter_target();

    // --- Bearing-based navigation to loiter target ---
    const float bearing_rad = copter.current_loc.get_bearing(_loiter_target);
    const float dist_m      = copter.current_loc.get_distance(_loiter_target);

    // Heading P-controller: ±π/2 error → ±full steer at KP = 1.0
    const float heading_err_rad = wrap_PI(bearing_rad - ahrs.get_yaw());
    const float steer = constrain_float(
        heading_err_rad * (SURFACE_STEERING_MAX / M_PI_2) * g2.surface_head_kp,
        -SURFACE_STEERING_MAX, SURFACE_STEERING_MAX);

    // Throttle: ramp down within 3× acceptance radius, coast inside
    const float accept_m = wp_nav->get_wp_radius_m();
    const float slow_m   = 3.0f * accept_m;
    float thr = 0.0f;
    if (dist_m > slow_m) {
        thr = g2.surface_auto_spd;
    } else if (dist_m > accept_m) {
        thr = g2.surface_auto_spd * (dist_m - accept_m) / (slow_m - accept_m);
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, constrain_float(thr, 0.0f, 100.0f));
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steer);
}

// surface_loiter_exit - neutral outputs on exit
void ModeSurfaceLoiter::exit()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);
}

// update_loiter_target - pilot roll/pitch sticks offset the loiter target
void ModeSurfaceLoiter::update_loiter_target()
{
    const float fwd = channel_pitch->norm_input_dz();  // forward = positive pitch
    const float rt  = channel_roll->norm_input_dz();   // right   = positive roll

    if (fabsf(fwd) < 0.05f && fabsf(rt) < 0.05f) {
        return;
    }

    const float dt  = copter.scheduler.get_loop_period_s();
    const float yaw = ahrs.get_yaw();

    // Rotate stick input from body frame into NE frame then offset the target
    const float north = (fwd * cosf(yaw) - rt * sinf(yaw)) * SURF_LOITER_MOVE_RATE_MS * dt;
    const float east  = (fwd * sinf(yaw) + rt * cosf(yaw)) * SURF_LOITER_MOVE_RATE_MS * dt;
    _loiter_target.offset(north, east);
}

#endif  // MODE_SURFACE_ENABLED
