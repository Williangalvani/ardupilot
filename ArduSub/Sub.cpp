/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Sub.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Sub class
 */
Sub::Sub()
    :

#if !AP_SUB_RC_ENABLED
          control_mode(Mode::Number::MANUAL),
#endif
          motors(MAIN_LOOP_RATE),
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),
          ahrs_view(ahrs, ROTATION_NONE),
          attitude_control(ahrs_view, motors),
          pos_control(ahrs_view, motors, attitude_control),
          wp_nav(ahrs_view, pos_control, attitude_control),
          loiter_nav(ahrs_view, pos_control, attitude_control),
          circle_nav(ahrs_view, pos_control),
          param_loader(var_info),
          flightmode(&mode_manual),
          auto_mode(Auto_WP),
          guided_mode(Guided_WP)
{
    failsafe.pilot_input = true;
    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one Sub");
    }
    _singleton = this;
}

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Sub, &sub, func)

/*
  scheduler table - all tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */

const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &sub.ins, update),
    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller),
    // send outputs to the motors library immediately
    FAST_TASK(motors_output),
     // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading
    FAST_TASK(check_ekf_yaw_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),
    // check if we've reached the surface or bottom
    FAST_TASK(update_surface_and_bottom_detector),
#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    FAST_TASK_CLASS(AP_Mount, &sub.camera_mount, update_fast),
#endif

    SCHED_TASK(fifty_hz_loop,         50,     75,   3),
#if AP_SUB_RC_ENABLED
    SCHED_TASK(rc_loop,              50,    130,  3),
#endif
    SCHED_TASK_CLASS(AP_GPS, &sub.gps, update, 50, 200,   6),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow,          &sub.optflow,             update,         200, 160,   9),
#endif
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    SCHED_TASK(read_rangefinder,      20,    100,  15),
    SCHED_TASK(update_altitude,       10,    100,  18),
#if AP_SUB_RC_ENABLED
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&sub.g2.rc_channels, read_aux_all, 10,  50,  18),
#endif
    SCHED_TASK(three_hz_loop,          3,     75,  21),
    SCHED_TASK(update_turn_counter,   10,     50,  24),
    SCHED_TASK(one_hz_loop,            1,    100,  33),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180,  36),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550,  39),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75,  45),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update,              50,  75,  48),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  51),
    SCHED_TASK(twentyfive_hz_logging, 25,    110,  54),
    SCHED_TASK(loop_rate_logging, LOOP_RATE, 50,   55),
    SCHED_TASK_CLASS(AP_Logger,           &sub.logger,       periodic_tasks,     400, 300,  57),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50,  60),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75,  63),
#endif
    SCHED_TASK(terrain_update,        10,    100,  72),
#if AP_STATS_ENABLED
    SCHED_TASK(stats_update,           1,    200,  76),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75,  78),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75,  81),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75,  84),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75,  87),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75,  90),
#endif

};

void Sub::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Sub::_failsafe_priorities[5];

void Sub::run_rate_controller()
{
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    motors.set_dt_s(last_loop_time_s);
    attitude_control.set_dt_s(last_loop_time_s);
    pos_control.set_dt_s(last_loop_time_s);

    //don't run rate controller in manual or motordetection modes
    if (control_mode != Mode::Number::MANUAL && control_mode != Mode::Number::MOTOR_DETECT) {
        // run low level rate controllers that only require IMU data and set loop time
        attitude_control.rate_controller_run();
    }
}

// 50 Hz tasks
void Sub::fifty_hz_loop()
{
    // check pilot input failsafe
    failsafe_pilot_input_check();

    failsafe_crash_check();

    failsafe_ekf_check();

    failsafe_sensors_check();
#if !AP_SUB_RC_ENABLED
    rc().read_input();
#endif
    g2.actuators.update_actuators();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass()
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if (AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
    }
}

#if HAL_LOGGING_ENABLED
// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        attitude_control.Write_ANG();
        attitude_control.Write_Rate(pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.D_get_accel_pid().get_pid_info());
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        motors.Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (sub.flightmode->requires_GPS() || sub.flightmode->requires_altitude())) {
        pos_control.write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        attitude_control.Write_ANG();
        attitude_control.Write_Rate(pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.D_get_accel_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}

// Full rate logging of IMU
void Sub::loop_rate_logging()
{
    if (should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}
#endif  // HAL_LOGGING_ENABLED

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    leak_detector.update();

    failsafe_leak_check();

    failsafe_internal_pressure_check();

    failsafe_internal_temperature_check();

    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AP_SERVORELAYEVENTS_ENABLED
    ServoRelayEvents.update_events();
#endif
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();
    AP_Notify::flags.flying = motors.armed();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }
#endif

    if (!motors.armed()) {
        motors.update_throttle_range();
    }

    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

#if HAL_LOGGING_ENABLED
    // log terrain data
    terrain_logging();
#endif

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    set_likely_flying(hal.util->get_soft_armed());

    attitude_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    pos_control.D_get_accel_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
}

void Sub::read_AHRS()
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    // <true> tells AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
    ahrs_view.update();
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        AP::ins().write_notch_log_messages();
#endif
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
#endif  // HAL_LOGGING_ENABLED
}

bool Sub::control_check_barometer()
{
    if (!ap.depth_sensor_present) { // can't hold depth without a depth sensor
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor is not connected.");
        return false;
    } else if (failsafe.sensor_health) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor error.");
        return false;
    }
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    distance = sub.wp_nav.get_wp_distance_to_destination_cm() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Sub::send_nav_controller_output()
    bearing = sub.wp_nav.get_wp_bearing_to_destination_cd() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Sub::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // no crosstrack error reported, see GCS_MAVLINK_Sub::send_nav_controller_output()
    xtrack_error = 0;
    return true;
}

// returns a Location to which the vehicle is currently heading, or
// would head in an autonomous mode
bool Sub::get_wp(Location &loc) const
{
    // in Guided mode, only Guided_WP uses wp_nav; the other submodes keep
    // their own targets. Guided_WP breaks out to the wp_nav return below.
    if (flightmode->in_guided_mode()) {
        switch (guided_mode) {
        case Guided_WP:
            break;
        case Guided_PosVelAccel: {
            Vector3f pos_neu_cm;
            if (!mode_guided.get_posvelaccel_target_NEU_cm(pos_neu_cm)) {
                return false;
            }
            loc = Location(pos_neu_cm, Location::AltFrame::ABOVE_ORIGIN);
            return true;
        }
        case Guided_Velocity:
        case Guided_VelAccel:
        case Guided_Angle:
            // no fixed position target to report
            return false;
        }
    }

    // for non-guided modes (Auto, etc.) fall back to wp_nav as before
    return wp_nav.is_active() && wp_nav.get_wp_destination_loc(loc);
}

#if AP_STATS_ENABLED
/*
  update AP_Stats
*/
void Sub::stats_update(void)
{
    AP::stats()->set_flying(motors.armed());
}
#endif

// get the altitude relative to the home position or the ekf origin
float Sub::get_alt_rel() const
{
    if (!ap.depth_sensor_present) {
        return 0;
    }

    // get relative position
    float posD;
    ahrs.get_relative_position_D_home(posD);
    return -posD;
}

// get the altitude above mean sea level
float Sub::get_alt_msl() const
{
    if (!ap.depth_sensor_present) {
        return 0;
    }

    Location origin;
    if (!ahrs.get_origin(origin)) {
        return 0;
    }

    // get relative position
    float posD;
    if (!ahrs.get_relative_position_D_origin_float(posD)) {
        // fall back to the barometer reading
        posD = -AP::baro().get_altitude();
    }

    // add in the ekf origin altitude
    posD -= static_cast<float>(origin.alt) * 0.01f;

    // convert down to up
    return -posD;
}

#if AP_SUB_RC_ENABLED
void Sub::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}
#endif

#if AP_EXTERNAL_BINDING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
// set target location (for use by external control and scripting)
bool Sub::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.guided_set_destination(target_loc);
}
#endif // AP_EXTERNAL_BINDING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_EXTERNAL_BINDING_ENABLED
// set target position (for use by scripting)
bool Sub::set_target_pos_NED(const Vector3f& target_pos_ned_m, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool is_terrain_alt)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos_ned_m.x * 100.0f, target_pos_ned_m.y * 100.0f, -target_pos_ned_m.z * 100.0f);

    return mode_guided.guided_set_destination(pos_neu_cm, use_yaw, yaw_deg * 100.0f, use_yaw_rate, yaw_rate_degs * 100.0f, yaw_relative, is_terrain_alt);
}

// set target position and velocity (for use by scripting)
bool Sub::set_target_posvel_NED(const Vector3f& target_pos_ned_m, const Vector3f& target_vel_ned_ms)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos_ned_m.x * 100.0f, target_pos_ned_m.y * 100.0f, -target_pos_ned_m.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel_ned_ms.x * 100.0f, target_vel_ned_ms.y * 100.0f, -target_vel_ned_ms.z * 100.0f);

    return mode_guided.guided_set_posvelaccel(pos_neu_cm, vel_neu_cms, Vector3f());
}

// set target position, velocity and acceleration (for use by scripting)
bool Sub::set_target_posvelaccel_NED(const Vector3f& target_pos_ned_m, const Vector3f& target_vel_ned_ms, const Vector3f& target_accel_ned_mss, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos_ned_m.x * 100.0f, target_pos_ned_m.y * 100.0f, -target_pos_ned_m.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel_ned_ms.x * 100.0f, target_vel_ned_ms.y * 100.0f, -target_vel_ned_ms.z * 100.0f);
    const Vector3f accel_neu_cmss(target_accel_ned_mss.x * 100.0f, target_accel_ned_mss.y * 100.0f, -target_accel_ned_mss.z * 100.0f);

    return mode_guided.guided_set_posvelaccel(pos_neu_cm, vel_neu_cms, accel_neu_cmss, use_yaw, yaw_deg * 100.0f, use_yaw_rate, yaw_rate_degs * 100.0f, yaw_relative);
}

// set target velocity (for use by scripting)
bool Sub::set_target_velocity_NED(const Vector3f& target_vel_ned_ms, bool align_yaw_to_target)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f vel_neu_cms(target_vel_ned_ms.x * 100.0f, target_vel_ned_ms.y * 100.0f, -target_vel_ned_ms.z * 100.0f);

    // optionally line up the sub with the velocity vector, otherwise hold a heading. keep the
    // heading already held by this controller, else latch the measured heading once; re-latching
    // every update would chase a still-rotating target
    float yaw_cd = ahrs.yaw_sensor;
    if ((guided_mode == Guided_Velocity) && (auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) && !yaw_rate_only) {
        yaw_cd = yaw_look_at_heading;
    }
    if (align_yaw_to_target) {
        const float speed_sq = vel_neu_cms.xy().length_squared();
        if (position_ok() && (speed_sq > (YAW_LOOK_AHEAD_MIN_SPEED * YAW_LOOK_AHEAD_MIN_SPEED))) {
            yaw_cd = degrees(atan2f(vel_neu_cms.y, vel_neu_cms.x)) * 100.0f;
        }
    }

    mode_guided.guided_set_velocity(vel_neu_cms, true, yaw_cd, false, 0.0f, false);
    return true;
}

// set target velocity and acceleration (for use by scripting)
bool Sub::set_target_velaccel_NED(const Vector3f& target_vel_ned_ms, const Vector3f& target_accel_ned_mss, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f vel_neu_cms(target_vel_ned_ms.x * 100.0f, target_vel_ned_ms.y * 100.0f, -target_vel_ned_ms.z * 100.0f);
    const Vector3f accel_neu_cmss(target_accel_ned_mss.x * 100.0f, target_accel_ned_mss.y * 100.0f, -target_accel_ned_mss.z * 100.0f);

    // without a yaw or yaw rate target hold a heading, as guided would otherwise return to a
    // stale look-at heading. keep the heading already held by this controller, else latch the
    // measured heading once; re-latching every update would chase a still-rotating target
    if (!use_yaw && !use_yaw_rate) {
        use_yaw = true;
        relative_yaw = false;
        if ((guided_mode == Guided_VelAccel) && (auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) && !yaw_rate_only) {
            yaw_deg = yaw_look_at_heading * 0.01f;
        } else {
            yaw_deg = ahrs.yaw_sensor * 0.01f;
        }
    }

    mode_guided.guided_set_velaccel(vel_neu_cms, accel_neu_cmss, use_yaw, yaw_deg * 100.0f, use_yaw_rate, yaw_rate_degs * 100.0f, relative_yaw);
    return true;
}

// set target roll pitch and yaw angles with climb rate (for use by scripting)
bool Sub::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    // optional yaw rate, in rad/s
    const Vector3f ang_vel_body { 0.0f, 0.0f, use_yaw_rate ? radians(yaw_rate_degs) : 0.0f };

    mode_guided.guided_set_angle(q, ang_vel_body, climb_rate_ms * 100.0f, false);
    return true;
}

// set target roll pitch and yaw rates with throttle (for use by scripting)
bool Sub::set_target_rate_and_throttle(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps, float throttle)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // Zero quaternion indicates rate control
    Quaternion q;
    q.zero();

    // Convert from degrees per second to radians per second
    Vector3f ang_vel_body { roll_rate_dps, pitch_rate_dps, yaw_rate_dps };
    ang_vel_body *= DEG_TO_RAD;

    // Pass to guided mode, throttle is 0 to 1 with 0.5 neutral
    mode_guided.guided_set_angle(q, ang_vel_body, throttle, true);
    return true;
}

// set target roll pitch and yaw angles and roll pitch and yaw rates with throttle (for use by scripting)
bool Sub::set_target_angle_and_rate_and_throttle(float roll_deg, float pitch_deg, float yaw_deg, float roll_rate_degs, float pitch_rate_degs, float yaw_rate_degs, float throttle)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    // Convert from degrees per second to radians per second
    Vector3f ang_vel_body { roll_rate_degs, pitch_rate_degs, yaw_rate_degs };
    ang_vel_body *= DEG_TO_RAD;

    // throttle is 0 to 1 with 0.5 neutral
    mode_guided.guided_set_angle(q, ang_vel_body, throttle, true);
    return true;
}

// set forward and lateral thrust from -1 to 1 (for use by scripting)
bool Sub::set_target_forward_lateral(float forward, float lateral)
{
    // exit if vehicle is not in a mode that accepts guided targets
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.guided_set_forward_lateral(forward, lateral);
}

// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Sub::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)Mode::Number::AUTO;
}

// lua scripts use this to retrieve the contents of the active command
bool Sub::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// lua scripts use this to indicate when they have complete the command
void Sub::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}

// get target location (for use by scripting)
bool Sub::get_target_location(Location& target_loc)
{
    return get_wp(target_loc);
}

/*
  update_target_location() acts as a wrapper for set_target_location
 */
bool Sub::update_target_location(const Location &old_loc, const Location &new_loc)
{
    /*
      by checking the caller has provided the correct old target
      location we prevent a race condition where the user changes mode
      or commands a different target in the controlling lua script
    */
    Location next_WP_loc;
    get_wp(next_WP_loc);
    if (!old_loc.same_loc_as(next_WP_loc) ||
         old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }

    return set_target_location(new_loc);
}

bool Sub::get_circle_radius(float &radius_m)
{
    radius_m = circle_nav.get_radius_m();
    return true;
}

bool Sub::set_circle_rate(float rate_dps)
{
    circle_nav.set_rate_degs(rate_dps);
    return true;
}

// set desired speed (m/s), as MAV_CMD_DO_CHANGE_SPEED does. Used for scripting.
bool Sub::set_desired_speed(float speed_ms)
{
    if (!is_positive(speed_ms)) {
        return false;
    }
    wp_nav.set_speed_NE_cms(speed_ms * 100.0f);
    return true;
}

// returns true if the EKF failsafe has triggered.  Only used by Lua scripts
bool Sub::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

// get control output (for use in scripting)
// returns true on success and control_value is set to a value in the range -1 to +1,
// except throttle which is 0 to 1 with 0.5 as no vertical thrust
bool Sub::get_control_output(AP_Vehicle::ControlOutput control_output, float &control_value)
{
    switch (control_output) {
    case AP_Vehicle::ControlOutput::Roll:
        control_value = constrain_float(motors.get_roll(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Pitch:
        control_value = constrain_float(motors.get_pitch(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Throttle:
        control_value = motors.get_throttle();
        return true;
    case AP_Vehicle::ControlOutput::Yaw:
        control_value = constrain_float(motors.get_yaw(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Lateral:
        control_value = constrain_float(motors.get_lateral(), -1.0f, 1.0f);
        return true;
    case AP_Vehicle::ControlOutput::Forward:
        control_value = constrain_float(motors.get_forward(), -1.0f, 1.0f);
        return true;
    default:
        return false;
    }
}

// Register a custom mode with given number and names
AP_Vehicle::custom_mode_state* Sub::register_custom_mode(const uint8_t num, const char* full_name, const char* short_name)
{
    const Mode::Number number = (Mode::Number)num;

    // See if this mode has been registered already, if it has return the state for it
    // This allows scripting restarts
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if (mode_guided_custom[i] == nullptr) {
            break;
        }
        if ((mode_guided_custom[i]->number() == number) &&
            (strcmp(mode_guided_custom[i]->name(), full_name) == 0) &&
            (strncmp(mode_guided_custom[i]->name4(), short_name, 4) == 0)) {
            return &mode_guided_custom[i]->state;
        }
    }

    // Number already registered to existing mode
    if (mode_from_mode_num(number) != nullptr) {
        return nullptr;
    }

    // Find free slot
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if (mode_guided_custom[i] == nullptr) {
            // Duplicate strings so were not pointing to unknown memory
            const char* full_name_copy = strdup(full_name);
            const char* short_name_copy = strndup(short_name, 4);
            if ((full_name_copy != nullptr) && (short_name_copy != nullptr)) {
                mode_guided_custom[i] = NEW_NOTHROW ModeGuidedCustom(number, full_name_copy, short_name_copy);
            }
            if (mode_guided_custom[i] == nullptr) {
                // Allocation failure
                free((void*)full_name_copy);
                free((void*)short_name_copy);
                return nullptr;
            }

            // Registration successful, notify the GCS that it should re-request the available modes
            gcs().available_modes_changed();

            return &mode_guided_custom[i]->state;
        }
    }

    // No free slots
    return nullptr;
}
#endif

Sub *Sub::_singleton = nullptr;

Sub sub;
AP_Vehicle& vehicle = sub;

AP_HAL_MAIN_CALLBACKS(&sub);
