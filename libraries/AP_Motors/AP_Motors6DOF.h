/// @file	AP_Motors6DOF.h
/// @brief	Motor control class for ROVs with direct control over 6DOF (or fewer) in movement

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"

/// @class      AP_MotorsMatrix
class AP_Motors6DOF : public AP_MotorsMatrix {
public:

    AP_Motors6DOF(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz) {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // Supported frame types
    typedef enum {
        SUB_FRAME_BLUEROV1,
        SUB_FRAME_VECTORED,
        SUB_FRAME_VECTORED_6DOF,
        SUB_FRAME_VECTORED_6DOF_90DEG,
        SUB_FRAME_SIMPLEROV_3,
        SUB_FRAME_SIMPLEROV_4,
        SUB_FRAME_SIMPLEROV_5,
        SUB_FRAME_CUSTOM
    } sub_frame_t;

    // Override parent
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // interpret the throttle demand as earth up rather than body up
    void set_earth_frame_throttle(bool earth_frame) { _earth_frame_throttle = earth_frame; }

    // unit vector along earth up expressed in body axes, used to spread an
    // earth frame throttle demand across the body axes
    void set_earth_up_body(const Vector3f &up_body) { _up_body = up_body; }

    // body frame vertical thrust, added after the earth frame throttle demand
    // has been distributed. range -1 ~ +1, positive is body up
    void set_throttle_body(float thrust) { _throttle_body = thrust; }

    // Override parent
    void output_min() override;

    // Map thrust input -1~1 to pwm output 1100~1900
    int16_t calc_thrust_to_pwm(float thrust_in) const;

    // Compensate thrust input -1~1 for asymmetric forward/reverse thrust
    float compensate_for_thrust_asymmetry(float thrust_in) const;

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    void set_max_throttle(float max_throttle) { _max_throttle = max_throttle; }

    // returns a vector with roll, pitch, and yaw contributions
    Vector3f get_motor_angular_factors(int motor_number);

    // returns true if motor is enabled
    bool motor_is_enabled(int motor_number);

    bool set_reversed(int motor_number, bool reversed);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    float               get_current_limit_max_throttle() override;

    //Override MotorsMatrix method
    void add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float lat_fac, uint8_t testing_order);

    void output_armed_stabilizing() override;
    void output_armed_stabilizing_vectored();
    void output_armed_stabilizing_vectored_6dof();

    // Clamp upwards thrust to the limit set by set_max_throttle()
    // Used to limit the motors output when surfaced to avoid sucking in air and wasting power
    float apply_max_throttle(float throttle_thrust);

    // cap upwards throttle, then distribute the linear demands across the body axes
    float limit_and_rotate_linear_demands(float &throttle, float &forward, float &lateral);

    // distribute the linear demands across the body axes
    void linear_demands_to_body(float &throttle, float &forward, float &lateral) const;

    // record motor saturation for the depth controller when the throttle demand is earth frame
    void note_motor_saturation(float mixed, float earth_up_demand);

    // Parameters
    AP_Int8             _motor_reverse[AP_MOTORS_MAX_NUM_MOTORS];
    AP_Float            _forwardVerticalCouplingFactor;
    // Non-positive values are treated identically to 1.0
    AP_Float            _thrust_asymmetry;

    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _lateral_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)

    float _max_throttle = 1.0f;
    // current limiting
    float _output_limited = 1.0f;
    float _batt_current_last = 0.0f;

    // unit vector along earth up expressed in body axes. Defaults to body up so
    // that an earth frame demand is inert until the vehicle supplies an attitude
    Vector3f _up_body{0.0f, 0.0f, -1.0f};

    // body frame vertical thrust demand, added after any earth frame distribution
    float _throttle_body = 0.0f;

    // the throttle demand is earth up rather than body up
    bool _earth_frame_throttle = false;
};
