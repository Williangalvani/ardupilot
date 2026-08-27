#include "Sub.h"


bool ModeStabilize::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_desired_U_cm(0);

    return true;
    return true;
}

void ModeStabilize::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.attitude_hold_active = false;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    sub.control_pilot_attitude(target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out((channel_throttle->norm_input() + 1.0f) / 2.0f, false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
