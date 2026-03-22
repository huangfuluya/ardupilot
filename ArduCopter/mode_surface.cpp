#include "Copter.h"

#if MODE_SURFACE_ENABLED

/*
 * Surface mode — unmanned surface vessel (boat) navigation for quad+boat hybrid
 *
 * The quadrotor motors idle at GROUND_IDLE while boat propulsion is commanded
 * through differential twin-engine SRV_Channel outputs:
 *   Throttle stick → base forward thrust mixed to both motors
 *   Yaw stick      → differential correction added/subtracted left/right
 *
 *   left_motor  = base_throttle + differential   (SRV_Channel::k_throttleLeft,  0..100 %)
 *   right_motor = base_throttle - differential   (SRV_Channel::k_throttleRight, 0..100 %)
 *
 * The attitude controller runs at zero throttle to keep the hull level and
 * prepares the vehicle for a quick transition back to aerial flight.
 *
 * Physical wiring / ground station setup:
 *   - Assign SERVOx_FUNCTION = 73 (ThrottleLeft)  to the port  (left)  ESC channel.
 *   - Assign SERVOx_FUNCTION = 74 (ThrottleRight) to the starboard (right) ESC channel.
 *   - Tune SURF_THR_GAIN and SURF_STEER_GAIN as needed.
 */

// Maximum differential thrust correction (0..100 %)
static constexpr float SURFACE_DIFF_MAX = 100.0f;

// surface_init - initialise surface controller
bool ModeSurface::init(bool ignore_checks)
{
    // Set output range for each motor: ±100 (power percent)
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  100);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 100);

    // Neutral boat outputs on entry
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0f);
    return true;
}

// surface_run - runs the surface (boat) controller
// should be called at 100hz or more
void ModeSurface::run()
{
    // Set quad motors to ground idle — not thrusting, ready for aerial takeoff
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

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

    // Map throttle stick (get_control_in() → 0..1000) to base forward thrust (0..100 power %).
    const float thr = constrain_float(
        channel_throttle->get_control_in() * 0.1f * g2.surface_thr_gain,
        0.0f, 100.0f);

    // Map yaw stick (-1.0..1.0) to differential correction (±100 %).
    // Positive yaw input → turn right (left motor faster, right motor slower).
    const float diff = channel_yaw->norm_input_dz() * SURFACE_DIFF_MAX * g2.surface_steer_gain;

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,
                                    constrain_float(thr + diff, 0.0f, 100.0f));
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight,
                                    constrain_float(thr - diff, 0.0f, 100.0f));
}

// surface_exit - clean up on exit
void ModeSurface::exit()
{
    // Neutral boat outputs on exit to prevent unintended movement
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft,  0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0f);
}

#endif  // MODE_SURFACE_ENABLED
