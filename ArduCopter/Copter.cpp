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
    // send outputs to the motors library immediately
    FAST_TASK(motors_output_main),

    SCHED_TASK(auto_disarm_check,     10,     50,  27),

    SCHED_TASK(update_throttle_hover,100,     90,  48),


    SCHED_TASK(three_hz_loop,          3,     75, 57),



#if HAL_LOGGING_ENABLED
    SCHED_TASK(loop_rate_logging, LOOP_RATE,    50,  75),
#endif

    SCHED_TASK(one_hz_loop,            1,    100,  81),

    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),


#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif

#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
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


/*
  constructor for main Copter class
 */
Copter::Copter(void)
    :
    flight_modes(&g.flight_mode1),
    flightmode(&mode_stabilize),
    param_loader(var_info)
{
}

Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);
