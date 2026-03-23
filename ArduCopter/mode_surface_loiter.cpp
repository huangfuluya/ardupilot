#include "Copter.h"

#if MODE_SURFACE_ENABLED

/*
 * Surface Loiter mode — GPS position hold for quad+boat hybrid vehicle on water.
 *
 * The quad motors idle at GROUND_IDLE while the boat's propulsion holds position
 * using differential twin-engine mixing.  Both ESCs must support bidirectional /
 * reversible operation (neutral = 1500 µs):
 *   Left  motor : SRV_Channel::k_throttleLeft  (-100..+100, SERVOx_FUNCTION = 73)
 *   Right motor : SRV_Channel::k_throttleRight (-100..+100, SERVOx_FUNCTION = 74)
 *
 * On entry the loiter target is set to the current GPS position.  The pilot
 * can shift the target with roll/pitch sticks (body-frame, rotated to NE).
 *
 * Heading controller (P) → differential mixing:
 *   diff = wrap_PI(bearing - yaw) × (100 / π/2) × SURF_HEAD_KP
 *   left_motor  = throttle + diff   (clamped -100..+100; negative = reverse)
 *   right_motor = throttle - diff   (clamped -100..+100; negative = reverse)
 *
 * Throttle (distance ramp):
 *   • > 3×WPNAV_RADIUS  : full SURF_AUTO_SPD %
 *   • WPNAV_RADIUS .. 3× : linear ramp down to 0
 *   • ≤ WPNAV_RADIUS    : 0 (coast to rest inside target circle)
 */

// Maximum differential thrust correction (0..100 %)
static constexpr float SURFACE_DIFF_MAX = 100.0f;
// Target-move rate when pilot applies roll/pitch sticks (m/s)
static constexpr float SURF_LOITER_MOVE_RATE_MS = 2.0f;

// surface_loiter_init - capture current GPS position as the loiter target
bool ModeSurfaceLoiter::init(bool ignore_checks)
{
    // Refuse to enter a boat mode while flying in multirotor mode
    if (!copter.ap.land_complete && !ignore_checks) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SurfLoiter: not available while airborne");
        return false;
    }

    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }

    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  100);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 100);

    // Neutral outputs on entry; reset ramp state
    _thr_left  = 0.0f;
    _thr_right = 0.0f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0f);

    _loiter_target = copter.current_loc;
    return true;
}

// surface_loiter_run - runs the surface loiter controller
// should be called at 100 Hz or more
void ModeSurfaceLoiter::run()
{
    // While disarmed reset ramp state and skip boat output.
    // Mode::output_to_motors() already forces both channels to neutral PWM;
    // resetting _thr_left/_thr_right here ensures arming always starts from 0.
    if (!motors->armed()) {
        _thr_left  = 0.0f;
        _thr_right = 0.0f;
        return;
    }

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

    // Heading P-controller → differential correction: ±π/2 error → ±100% diff at KP=1.0
    const float heading_err_rad = wrap_PI(bearing_rad - ahrs.get_yaw_rad());
    const float diff = constrain_float(
        heading_err_rad * (SURFACE_DIFF_MAX / M_PI_2) * g2.surface_head_kp,
        -SURFACE_DIFF_MAX, SURFACE_DIFF_MAX);

    // Throttle: ramp down within 3× acceptance radius, coast inside
    const float accept_m = wp_nav->get_wp_radius_m();
    const float slow_m   = 3.0f * accept_m;
    float thr = 0.0f;
    if (dist_m > slow_m) {
        thr = g2.surface_auto_spd;
    } else if (dist_m > accept_m) {
        thr = g2.surface_auto_spd * (dist_m - accept_m) / (slow_m - accept_m);
    }

    // Target outputs before rate limiting
    const float tgt_left  = constrain_float(thr + diff, -100.0f, 100.0f);
    const float tgt_right = constrain_float(thr - diff, -100.0f, 100.0f);

    // Apply ramp rate limit (SURF_RAMP_SPD %/s); 0 = no limiting
    const float ramp = g2.surface_ramp_spd;
    if (ramp > 0.0f) {
        const float max_delta = ramp * copter.scheduler.get_loop_period_s();
        _thr_left  += constrain_float(tgt_left  - _thr_left,  -max_delta, max_delta);
        _thr_right += constrain_float(tgt_right - _thr_right, -max_delta, max_delta);
    } else {
        _thr_left  = tgt_left;
        _thr_right = tgt_right;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  _thr_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, _thr_right);
}

// surface_loiter_exit - neutral outputs on exit
void ModeSurfaceLoiter::exit()
{
    _thr_left  = 0.0f;
    _thr_right = 0.0f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0f);
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
    const float yaw = ahrs.get_yaw_rad();

    // Rotate stick input from body frame into NE frame then offset the target
    const float north = (fwd * cosf(yaw) - rt * sinf(yaw)) * SURF_LOITER_MOVE_RATE_MS * dt;
    const float east  = (fwd * sinf(yaw) + rt * cosf(yaw)) * SURF_LOITER_MOVE_RATE_MS * dt;
    _loiter_target.offset(north, east);
}

#endif  // MODE_SURFACE_ENABLED
