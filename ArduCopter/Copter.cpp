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

/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to: Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Andy Piper          :Harmonic notch, In-flight FFT, Bi-directional DShot, various drivers
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel         :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland :PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: https://copter.ardupilot.org/
 *
 */

#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Copter, &copter, func)

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
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
// update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller_main),

#if AC_CUSTOMCONTROL_MULTI_ENABLED
    FAST_TASK(run_custom_controller),
#endif

    // send outputs to the motors library immediately
    FAST_TASK(motors_output_main),
     // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),

    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),
    // check if we've landed or crashed
    FAST_TASK(update_land_and_crash_detectors),
    // surface tracking update
    FAST_TASK(update_rangefinder_terrain_offset),

    SCHED_TASK(rc_loop,              250,    130,  3),
    SCHED_TASK(throttle_loop,         50,     75,  6),

    SCHED_TASK_CLASS(AP_GPS,               &copter.gps,                 update,          50, 200,   9),

    SCHED_TASK(update_batt_compass,   10,    120, 15),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all,    10,  50,  18),

    SCHED_TASK(auto_disarm_check,     10,     50,  27),
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    SCHED_TASK_CLASS(RC_Channels_Copter,   &copter.g2.rc_channels,      auto_trim_run,   10,  75,  30),
#endif

#if AP_RANGEFINDER_ENABLED
    SCHED_TASK(read_rangefinder,      20,    100,  33),
#endif

    SCHED_TASK(update_altitude,       10,    100,  42),
    SCHED_TASK(run_nav_updates,       50,    100,  45),
    SCHED_TASK(update_throttle_hover,100,     90,  48),


    SCHED_TASK(three_hz_loop,          3,     75, 57),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,  75,  60),
#endif


#if HAL_LOGGING_ENABLED
    SCHED_TASK(loop_rate_logging, LOOP_RATE,    50,  75),
#endif

    SCHED_TASK(one_hz_loop,            1,    100,  81),
    SCHED_TASK(ekf_check,             10,     75,  84),
    SCHED_TASK(check_vibration,       10,     50,  87),
    SCHED_TASK(gpsglitch_check,       10,     50,  90),
    SCHED_TASK(takeoff_check,         50,     50,  91),

    SCHED_TASK(standby_update,        100,    75,  96),
    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),


#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif

    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50, 123),

#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
#endif
#if AP_TEMPCALIBRATION_ENABLED
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100, 135),
#endif

#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,            &copter.button,              update,           5, 100, 168),
#endif
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // don't delete this, there is an equivalent (virtual) in AP_Vehicle for the non-rate loop case
    SCHED_TASK(update_dynamic_notch_at_specified_rate_main,                       LOOP_RATE, 200, 215),
#endif

};

void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];


#if AP_EXTERNAL_CONTROL_ENABLED
#if MODE_GUIDED_ENABLED
// set target location (for use by external control and scripting)
bool Copter::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}

// start takeoff to given altitude (for use by scripting)
bool Copter::start_takeoff(const float alt_m)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start_m(alt_m)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}
#endif // MODE_GUIDED_ENABLED
#endif // AP_EXTERNAL_CONTROL_ENABLED

// returns true if vehicle is landing.
bool Copter::is_landing() const
{
    return flightmode->is_landing();
}

// returns true if vehicle is taking off.
bool Copter::is_taking_off() const
{
    return flightmode->is_taking_off();
}

// bool Copter::current_mode_requires_mission() const
// {
//     return false;
// }

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_mix();

    // check auto_armed status
    update_auto_armed();


    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
    update_ekf_terrain_height_stable();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

#if HAL_LOGGING_ENABLED
// Full rate logging of attitude, rate and pid loops
// should be run at loop rate
void Copter::loop_rate_logging()
{
   if (should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
        if (!using_rate_thread) {
            Log_Write_Rate();
            Log_Write_PIDS(); // only logs if PIDS bitmask is set
        }
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    if (should_log(MASK_LOG_FTN_FAST) && !using_rate_thread) {
        AP::ins().write_notch_log_messages();
    }
#endif
    if (should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // always write AHRS attitude at 10Hz
    ahrs.Write_Attitude(attitude_control->get_att_target_euler_rad() * RAD_TO_DEG);
    // log attitude controller data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
        if (!using_rate_thread) {
            Log_Write_Rate();
        }
    }
    if (!should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
    // log at 10Hz if PIDS bitmask is selected, even if no ATT bitmask is selected; logs at looprate if ATT_FAST and PIDS bitmask set
        if (!using_rate_thread) {
            Log_Write_PIDS();
        }
    }
    // log EKF attitude data always at 10Hz unless ATTITUDE_FAST, then do it in the 25Hz loop
    if (!should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
#if AP_RSSI_ENABLED
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
#endif
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS() || !flightmode->has_manual_throttle())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {

    }

}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST))) {
        AP::ins().Write_IMU();
    }

#if HAL_GYROFFT_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        gyro_fft.write_log_messages();
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

// three_hz_loop - 3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

    // check for deadreckoning failsafe
    failsafe_deadreckon_check();

#if AP_RC_TRANSMITTER_TUNING_ENABLED
    //update transmitter based in flight tuning
    tuning();
#endif  // AP_RC_TRANSMITTER_TUNING_ENABLED

}

// ap_value calculates a 32-bit bitmask representing various pieces of
// state about the Copter.  It replaces a global variable which was
// used to track this state.
uint32_t Copter::ap_value() const
{
    uint32_t ret = 0;

    const bool *b = (const bool *)&ap;
    for (uint8_t i=0; i<sizeof(ap); i++) {
        if (b[i]) {
            ret |= 1U<<i;
        }
    }

    return ret;
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap_value());
    }
#endif

    if (!motors->armed()) {
        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());
    }

    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

    AP_Notify::flags.flying = !ap.land_complete;

    // slowly update the PID notches with the average loop rate
    if (!using_rate_thread) {
        attitude_control->set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    }
    pos_control->get_accel_U_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());

#if AC_CUSTOMCONTROL_MULTI_ENABLED
    custom_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#endif

#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // see if we should have a separate rate thread
    if (!started_rate_thread && get_fast_rate_type() != FastRateType::FAST_RATE_DISABLED) {
        if (hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&Copter::rate_controller_thread, void),
                                         "rate",
                                         1536, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
            started_rate_thread = true;
        } else {
            AP_BoardConfig::allocation_error("rate thread");
        }
    }
#endif
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing_rad = wrap_2PI(ahrs.get_yaw_rad() + radians(180.0));
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

#if HAL_LOGGING_ENABLED
    // log the simple bearing
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
#endif
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (!ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    // avoid processing bind-time RC values:
    if (!rc().has_valid_input()) {
        return;
    }
    // rotate roll, pitch input by -super simple heading (reverse of heading to home)
    rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
    pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing_rad has been updated
void Copter::update_super_simple_bearing(bool force_update)
{

    const float bearing_rad = home_bearing_rad();

    // check the bearing to home has changed by at least 5 degrees
    // todo: consider updating this continuously
    if (fabsf(wrap_PI(super_simple_last_bearing_rad - bearing_rad)) < radians(5.0)) {
        return;
    }

    super_simple_last_bearing_rad = bearing_rad;
    const float angle_rad = super_simple_last_bearing_rad + radians(180.0);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

void Copter::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in FAST_TASK.
    ahrs.update(true);
}

// read baro and log control tuning
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_FTN_FAST)) {
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
            AP::ins().write_notch_log_messages();
#endif
#if HAL_GYROFFT_ENABLED
            gyro_fft.write_log_messages();
#endif
        }
    }
#endif
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance_m();
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing_deg();
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error_m() * 0.01;
    return true;
}

// get the target earth-frame angular velocities in rad/s (Z-axis component used by some gimbals)
bool Copter::get_rate_ef_targets(Vector3f& rate_ef_targets) const
{
    // always returns zero vector if landed or disarmed
    if (copter.ap.land_complete) {
        rate_ef_targets.zero();
    } else {
        rate_ef_targets = attitude_control->get_rate_ef_targets();
    }
    return true;
}

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    :
    flight_modes(&g.flight_mode1),
    pos_variance_filt(FS_EKF_FILT_DEFAULT),
    vel_variance_filt(FS_EKF_FILT_DEFAULT),
    flightmode(&mode_stabilize),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    param_loader(var_info)
{
}

Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);
