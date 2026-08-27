#include "Sub.h"

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Sub::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // sanity check angle max parameter
    const float angle_max_cd = attitude_control.lean_angle_max_cd();

    // limit max lean angle
    angle_max = constrain_float(angle_max, 1000, angle_max_cd);

    // scale roll_in, pitch_in to ATC_ANGLE_MAX parameter range
    float scaler = angle_max_cd/(float)ROLL_PITCH_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = norm(pitch_in, roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI) * atanf(cosf(pitch_in*(M_PI/18000))*tanf(roll_in*(M_PI/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}

// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Sub::get_pilot_desired_yaw_rate(int16_t stick_angle) const
{
    // convert pilot input to the desired yaw rate
    return stick_angle * g.acro_yaw_p;
}

// the roll and pitch trim buttons are momentary, they rotate the vehicle while held. The button
// state is only rebuilt when the pilot sends input, so an unheard pilot must not keep us rotating
float Sub::get_pilot_trim_rate_cds(int8_t direction) const
{
    if (failsafe.pilot_input || AP_HAL::millis() - last_trim_button_ms > PILOT_TRIM_BUTTON_TIMEOUT_MS) {
        return 0.0f;
    }
    return direction * g.pilot_trim_rate * 100.0f;
}

// returns the roll rate requested by the trim buttons, in centi-degrees per second
float Sub::get_pilot_trim_roll_rate_cds() const
{
    return get_pilot_trim_rate_cds(pilot_trim_roll_dir);
}

// returns the pitch rate requested by the trim buttons, in centi-degrees per second
float Sub::get_pilot_trim_pitch_rate_cds() const
{
    return get_pilot_trim_rate_cds(pilot_trim_pitch_dir);
}

// control_pilot_attitude - hold the pilot's attitude target, rotating it while the pilot asks for
// rotation. The target is advanced by the demanded body-frame rate rather than built from euler
// angles, so roll and pitch are not limited: the pilot may rotate through vertical, and past it,
// and the attitude reached when the demand stops is the attitude that is then held
void Sub::control_pilot_attitude(float target_yaw_rate_cds)
{
    // pick the current attitude up as the target when it has not been tracked, otherwise a mode
    // that leaves the target behind, such as manual, would snap the vehicle when it is left
    if (!attitude_hold_active) {
        attitude_control.reset_target_and_rate();
        attitude_hold_active = true;
    }

    // the roll and pitch sticks request rotation at the same rate as the trim buttons, so that a
    // mode using them behaves the same way as the buttons do
    const float roll_rate_cds = get_pilot_trim_roll_rate_cds()
                                + channel_roll->norm_input_dz() * g.pilot_trim_rate * 100.0f;
    const float pitch_rate_cds = get_pilot_trim_pitch_rate_cds()
                                 + channel_pitch->norm_input_dz() * g.pilot_trim_rate * 100.0f;

    attitude_control.input_rate_bf_roll_pitch_yaw_cds(roll_rate_cds, pitch_rate_cds, target_yaw_rate_cds);
}

// check for ekf yaw reset and adjust target heading
void Sub::check_ekf_yaw_reset()
{
    const uint16_t new_ahrs_yaw_reset_count = ahrs.get_yaw_reset_count();
    if (new_ahrs_yaw_reset_count != ahrs_yaw_reset_count) {
        attitude_control.inertial_frame_reset();
        ahrs_yaw_reset_count = new_ahrs_yaw_reset_count;
    }
}

/*************************************************************
 * yaw controllers
 *************************************************************/

// get_roi_yaw - returns heading towards location held in roi_WP
// should be called at 100hz
float Sub::get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // used to reduce update rate to 100hz

    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        roi_yaw_counter = 0;
        yaw_look_at_WP_bearing = get_bearing_cd((pos_control.get_pos_estimate_NED_m().xy() * 100.0f).tofloat(), roi_WP_neu_cm.xy());
    }

    return yaw_look_at_WP_bearing;
}

float Sub::get_look_ahead_yaw()
{
    Vector3f vel = (pos_control.get_vel_estimate_NED_ms() * 100.0f).tofloat();
    vel.z = -vel.z;
    const float speed_sq = vel.xy().length_squared();
    // Commanded Yaw to automatically look ahead.
    if (position_ok() && (speed_sq > (YAW_LOOK_AHEAD_MIN_SPEED * YAW_LOOK_AHEAD_MIN_SPEED))) {
        yaw_look_ahead_bearing = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Sub::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if (failsafe.pilot_input) {
        return 0.0f;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float mid_stick = channel_throttle->get_control_mid();
    float deadband_top = mid_stick + g.throttle_deadzone * gain;
    float deadband_bottom = mid_stick - g.throttle_deadzone * gain;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        return get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        return g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // must be in the deadband
        return 0.0f;
    }
}

// behavior is similar to Sub::get_pilot_desired_climb_rate
float Sub::get_pilot_desired_horizontal_rate(RC_Channel *channel) const
{
    if (failsafe.pilot_input) {
        return 0;
    }

    // forward and lateral sticks have center trim, unlike throttle
    auto control = channel->norm_input();

    // normalize deadzone
    auto dz = (float)g.throttle_deadzone * 2.0f / (float)(channel->get_radio_max() - channel->get_radio_min());
    auto deadband_top = dz * gain;
    auto deadband_bottom = -dz * gain;

    if (control < deadband_bottom) {
        // below the deadband
        return (float)g.pilot_speed * (control - deadband_bottom);
    } else if (control > deadband_top) {
        // above the deadband
        return (float)g.pilot_speed * (control - deadband_top);
    } else {
        // must be in the deadband
        return 0;
    }
}

// rotate vector from vehicle's perspective to North-East frame
void Sub::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}

// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
uint16_t Sub::get_pilot_speed_dn() const
{
    if (g.pilot_speed_dn == 0) {
        return abs(g.pilot_speed_up);
    }
    return abs(g.pilot_speed_dn);
}
