#include "Copter.h"

#if MODE_SURFACE_ENABLED

/*
 * Surface mode — unmanned surface vessel (boat) navigation for quad+boat hybrid
 *
 * The quadrotor motors idle at GROUND_IDLE while boat propulsion is commanded
 * through SRV_Channel outputs:
 *   Throttle stick → boat forward thrust  (SRV_Channel::k_throttle,  0..100 power %)
 *   Yaw stick      → boat steering/rudder (SRV_Channel::k_steering, ±4500 centideg)
 *
 * The attitude controller runs at zero throttle to keep the hull level and
 * prepares the vehicle for a quick transition back to aerial flight.
 *
 * Physical wiring / ground station setup:
 *   - Assign SERVOx_FUNCTION = 70 (Throttle) to the boat ESC channel.
 *   - Assign SERVOx_FUNCTION = 26 (GroundSteering) to the rudder/steering channel.
 *   - Tune SURF_THR_GAIN and SURF_STEER_GAIN as needed.
 */

// Scaled output limit for the steering servo (centidegrees, matches SERVO_MAX in AP_MotorsUGV)
#define SURFACE_STEERING_MAX 4500.0f

// surface_init - initialise surface controller
bool ModeSurface::init(bool ignore_checks)
{
    // Set servo output angle ranges to match Rover UGV motor convention:
    //   k_throttle : -100..100  (power percent, bidirectional or unidirectional)
    //   k_steering : ±4500      (centidegrees, ±45 degrees of travel)
    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);
    SRV_Channels::set_angle(SRV_Channel::k_steering, SURFACE_STEERING_MAX);

    // Neutral boat outputs on entry
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);
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

    // Map throttle stick (get_control_in() → 0..1000) to boat forward thrust (0..100 power %).
    // Multiply by the user-adjustable surface_thr_gain to scale the output.
    const float thr_pct = (channel_throttle->get_control_in() * 0.1f) * g2.surface_thr_gain;
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,
                                    constrain_float(thr_pct, 0.0f, 100.0f));

    // Map yaw stick (-1.0..1.0) to boat steering (±4500 centidegrees = ±45 degrees).
    const float steer = channel_yaw->norm_input_dz() * SURFACE_STEERING_MAX * g2.surface_steer_gain;
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering,
                                    constrain_float(steer, -SURFACE_STEERING_MAX, SURFACE_STEERING_MAX));
}

// surface_exit - clean up on exit
void ModeSurface::exit()
{
    // Neutral boat outputs on exit to prevent unintended movement
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, 0.0f);
}

#endif  // MODE_SURFACE_ENABLED
