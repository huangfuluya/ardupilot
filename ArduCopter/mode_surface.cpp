#include "Copter.h"

#if MODE_SURFACE_ENABLED

/*
 * Surface mode — unmanned surface vessel (boat) navigation for quad+boat hybrid
 *
 * The quadrotor motors are shut down while boat propulsion is commanded
 * through differential twin-engine SRV_Channel outputs.  Both ESCs must be
 * configured for bidirectional / reversible operation (neutral = 1500 µs):
 *   Throttle stick above centre → forward thrust, below centre → reverse thrust
 *   Yaw stick                   → differential correction added/subtracted left/right
 *
 *   left_motor  = throttle + differential   (SRV_Channel::k_throttleLeft,  -100..+100)
 *   right_motor = throttle - differential   (SRV_Channel::k_throttleRight, -100..+100)
 *
 * Scaled value 0 → 1500 µs (neutral/stopped), +100 → max PWM (full forward),
 * -100 → min PWM (full reverse).
 *
 * Physical wiring / ground station setup:
 *   - Assign SERVOx_FUNCTION = 73 (ThrottleLeft)  to the port  (left)  ESC channel.
 *   - Assign SERVOx_FUNCTION = 74 (ThrottleRight) to the starboard (right) ESC channel.
 *   - Use bidirectional/reversible ESCs (neutral calibrated at 1500 µs).
 *   - Tune SURF_THR_GAIN and SURF_STEER_GAIN as needed.
 */

// Maximum differential thrust correction (0..100 %)
static constexpr float SURFACE_DIFF_MAX = 100.0f;

// surface_init - initialise surface controller
bool ModeSurface::init(bool ignore_checks)
{
    // Refuse to enter a boat mode while armed in multirotor mode
    if (motors->armed() && !ignore_checks) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Surface: disarm before switching to surface mode");
        return false;
    }

    // Set output range for each motor: ±100 (power percent)
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  100);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 100);

    // Neutral boat outputs on entry; reset ramp state
    _thr_left  = 0.0f;
    _thr_right = 0.0f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0f);
    return true;
}

// surface_run - runs the surface (boat) controller
// should be called at 100hz or more
void ModeSurface::run()
{
    // While disarmed reset ramp state and skip boat output.
    // Mode::output_to_motors() already forces both channels to neutral PWM;
    // resetting _thr_left/_thr_right here ensures arming always starts from 0.
    if (!motors->armed()) {
        _thr_left  = 0.0f;
        _thr_right = 0.0f;
        return;
    }

    // Set quad motors to shut down — no spinning in surface mode
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    // Maintain level attitude with zero thrust from quad motors.
    // This keeps the hull level and I-terms reset for a clean air transition.
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, g.throttle_filt);

    // Map throttle stick (0..1000, centre=500) to bidirectional thrust (-100..+100 %).
    // Stick above centre → forward; below centre → reverse.
    const float thr = constrain_float(
        (channel_throttle->get_control_in() - 500) * 0.2f * g2.surface_thr_gain,
        -100.0f, 100.0f);

    // Map yaw stick (-1.0..1.0) to differential correction (±100 %).
    // Positive yaw input → turn right (left motor faster, right motor slower).
    const float diff = channel_yaw->norm_input_dz() * SURFACE_DIFF_MAX * g2.surface_steer_gain;

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

// surface_exit - clean up on exit
void ModeSurface::exit()
{
    // Neutral boat outputs on exit to prevent unintended movement
    _thr_left  = 0.0f;
    _thr_right = 0.0f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0f);
}

#endif  // MODE_SURFACE_ENABLED
