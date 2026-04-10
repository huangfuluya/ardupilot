#include "Copter.h"

#if MODE_SQUARE_ENABLED == ENABLED

#define SQUARE_WP_RADIUS_CM 200.0f
#define SQUARE_WP_HOLD_MS 200U

// initialise square mode
bool ModeSquare::init(bool ignore_checks)
{
    // set speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise vertical controller if needed
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // start waypoint navigation
    wp_nav->wp_and_spline_init();

    const float side_length_m = constrain_float((float)g.square_side_length, 1.0f, 200.0f);
    const float side_length_cm = side_length_m * 100.0f;
    const Vector2f start_xy = inertial_nav.get_position_xy_cm();
    const Vector2f forward {ahrs.cos_yaw(), ahrs.sin_yaw()};
    const Vector2f right {-ahrs.sin_yaw(), ahrs.cos_yaw()};

    _terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    float target_z = 0.0f;
    if (_terrain_alt) {
        if (!copter.surface_tracking.get_target_alt_cm(target_z)) {
            target_z = copter.rangefinder_state.alt_cm_filt.get();
        }
    } else {
        target_z = pos_control->is_active_z() ? pos_control->get_pos_target_z_cm() : inertial_nav.get_position_z_up_cm();
    }

    const Vector2f wp1 = start_xy + forward * side_length_cm;
    const Vector2f wp2 = wp1 + right * side_length_cm;
    const Vector2f wp3 = start_xy + right * side_length_cm;

    _waypoints[0] = Vector3f(wp1.x, wp1.y, target_z);
    _waypoints[1] = Vector3f(wp2.x, wp2.y, target_z);
    _waypoints[2] = Vector3f(wp3.x, wp3.y, target_z);
    _waypoints[3] = Vector3f(start_xy.x, start_xy.y, target_z);

    _leg_index = 0;
    _completed = false;
    _reached_wp_time_ms = 0;

    if (!wp_nav->set_wp_destination(_waypoints[_leg_index], _terrain_alt)) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Square: start side %.1fm", (double)side_length_m);
    return true;
}

// run square mode
void ModeSquare::run()
{
    // set speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const bool wpnav_ok = wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0.0f);

    if (!wpnav_ok || _completed) {
        return;
    }

    if (!wp_nav->reached_wp_destination() || (wp_nav->get_wp_distance_to_destination() > SQUARE_WP_RADIUS_CM)) {
        _reached_wp_time_ms = 0;
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (_reached_wp_time_ms == 0) {
        _reached_wp_time_ms = now;
        return;
    }

    if ((now - _reached_wp_time_ms) < SQUARE_WP_HOLD_MS) {
        return;
    }

    _reached_wp_time_ms = 0;
    _leg_index++;

    if (_leg_index >= NUM_WAYPOINTS) {
        _completed = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Square: complete");
        return;
    }

    if (!wp_nav->set_wp_destination(_waypoints[_leg_index], _terrain_alt)) {
        _completed = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "Square: set waypoint failed");
    }
}

uint32_t ModeSquare::wp_distance() const
{
    if (_completed) {
        return 0;
    }
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeSquare::wp_bearing() const
{
    if (_completed) {
        return 0;
    }
    return wp_nav->get_wp_bearing_to_destination();
}

#endif // MODE_SQUARE_ENABLED == ENABLED
