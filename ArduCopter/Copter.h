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
#pragma once
/*
  This is the main Copter class
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

// Common dependencies
#include <AP_Common/AP_Common.h>            // Common definitions and utility routines for the ArduPilot libraries
#include <AP_Common/Location.h>             // Library having the implementation of location class         
#include <AP_Param/AP_Param.h>              // A system for managing and storing variables that are of general interest to the system.
#include <StorageManager/StorageManager.h>  // library for Management for hal.storage to allow for backwards compatible mapping of storage offsets to available storage

// Application dependencies
#include <AP_Logger/AP_Logger.h>            // ArduPilot Mega Flash Memory Library
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library
#include <AP_Motors/AP_Motors.h>            // AP Motors library
#include <Filter/Filter.h>                  // Filter library
#include <AP_Vehicle/AP_Vehicle.h>          // needed for AHRS build

// Configuration
#include "defines.h"
#include "config.h"

#define MOTOR_CLASS AP_MotorsMulticopter

#include "RC_Channel_Copter.h"         // RC Channel Library

#include "GCS_MAVLink_Copter.h"
#include "GCS_Copter.h"
#include "AP_Arming_Copter.h"
#if OSD_ENABLED || OSD_PARAM_ENABLED
 #include <AP_OSD/AP_OSD.h>
#endif

// Local modules
#include "Parameters.h"

class Copter : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Copter;
    friend class GCS_Copter;
    friend class Parameters;
    friend class ParametersG2;

    friend class AP_Arming_Copter;
 

    friend class RC_Channel_Copter;
    friend class RC_Channels_Copter;

    Copter(void);

private:

    // key aircraft parameters passed to multiple libraries
    AP_MultiCopter aparm;

    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;


    // flight modes convenience array
    AP_Int8 *flight_modes;
    const uint8_t num_flight_modes = 6;

    // Arming/Disarming management class
    AP_Arming_Copter arming;

    // GCS selection
    GCS_Copter _gcs; // avoid using this; use gcs()
    GCS_Copter &gcs() { return _gcs; }

    // ap_value calculates a 32-bit bitmask representing various pieces of
    // state about the Copter.  It replaces a global variable which was
    // used to track this state.
    uint32_t ap_value() const;

    // These variables are essentially global variables.  These should
    // be removed over time.  It is critical that the offsets of these
    // variables remain unchanged - the logging is dependent on this
    // ordering!
    struct PACKED {
        bool unused1;                        //  0
        bool unused_was_simple_mode_byte1;   //  1
        bool unused_was_simple_mode_byte2;   //  2
        bool pre_arm_rc_check;               //  3 true if rc input pre-arm checks have been completed successfully
        bool pre_arm_check;                  //  4 true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        bool auto_armed;                     //  5 stops auto missions from beginning until throttle is raised
        bool unused_log_started;             //  6
        bool land_complete;                  //  7 true if we have detected a landing
        bool new_radio_frame;                //  8 Set true if we have new PWM data to act on from the Radio
        bool unused_usb_connected;           //  9
        bool unused_receiver_present;        // 10
        bool compass_mot;                    // 11 true if we are currently performing compassmot calibration
        bool motor_test;                     // 12 true if we are currently performing the motors test
        bool initialised;                    // 13 true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
        bool land_complete_maybe;            // 14 true if we may have landed (less strict version of land_complete)
        bool throttle_zero;                  // 15 true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
        bool system_time_set_unused;         // 16 true if the system time has been set from the GPS
        bool gps_glitching;                  // 17 true if GPS glitching is affecting navigation accuracy
        bool using_interlock;                // 18 aux switch motor interlock function is in use
        bool land_repo_active;               // 19 true if the pilot is overriding the landing position
        bool motor_interlock_switch;         // 20 true if pilot is requesting motor interlock enable
        bool in_arming_delay;                // 21 true while we are armed but waiting to spin motors
        bool initialised_params;             // 22 true when the all parameters have been initialised. we cannot send parameters to the GCS until this is done
        bool unused_compass_init_location;   // 23
        bool unused2_aux_switch_rc_override_allowed; // 24
        bool armed_with_airmode_switch;      // 25 we armed using a arming switch
        bool prec_land_active;               // 26 true if precland is active
    } ap;

    AirMode air_mode; // air mode is 0 = not-configured ; 1 = disabled; 2 = enabled;
    bool force_flying; // force flying is enabled when true;

    // This is the state of the flight control system
    // There are multiple states defined such as STABILIZE, ACRO,
    Mode *flightmode;

    // Failsafe
    struct {
        uint32_t terrain_first_failure_ms;  // the first time terrain data access failed - used to calculate the duration of the failure
        uint32_t terrain_last_failure_ms;   // the most recent time terrain data access failed

        int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value

        uint8_t radio               : 1; // A status flag for the radio failsafe
        uint8_t gcs                 : 1; // A status flag for the ground station failsafe
        uint8_t ekf                 : 1; // true if ekf failsafe has occurred
        uint8_t terrain             : 1; // true if the missing terrain data failsafe has occurred
        uint8_t adsb                : 1; // true if an adsb related failsafe has occurred
        uint8_t deadreckon          : 1; // true if a dead reckoning failsafe has triggered
    } failsafe;


    // Motor Output
    MOTOR_CLASS *motors;
    const struct AP_Param::GroupInfo *motors_var_info;

#if OSD_ENABLED || OSD_PARAM_ENABLED
    AP_OSD osd;
#endif

    // System Timers
    // --------------
    // arm_time_ms - Records when vehicle was armed. Will be Zero if we are disarmed.
    uint32_t arm_time_ms;


    // last esc calibration notification update
    uint32_t esc_calibration_notify_update_ms;

    // Top-level logic
    // setup the var_info table
    AP_Param param_loader;


    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    // enum for ESC CALIBRATION
    enum ESCCalibrationModes : uint8_t {
        ESCCAL_NONE = 0,
        ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,
        ESCCAL_PASSTHROUGH_ALWAYS = 2,
        ESCCAL_AUTO = 3,
        ESCCAL_DISABLED = 9,
    };

    enum class FailsafeAction : uint8_t {
        NONE               = 0,
        LAND               = 1,
        RTL                = 2,
        SMARTRTL           = 3,
        SMARTRTL_LAND      = 4,
        TERMINATE          = 5,
        AUTO_DO_LAND_START = 6,
        BRAKE_LAND         = 7
    };

    enum class FailsafeOption {
        RC_CONTINUE_IF_AUTO             = (1<<0),   // 1
        GCS_CONTINUE_IF_AUTO            = (1<<1),   // 2
        RC_CONTINUE_IF_GUIDED           = (1<<2),   // 4
        CONTINUE_IF_LANDING             = (1<<3),   // 8
        GCS_CONTINUE_IF_PILOT_CONTROL   = (1<<4),   // 16
        RELEASE_GRIPPER                 = (1<<5),   // 32
    };


    enum class FlightOption : uint32_t {
        DISABLE_THRUST_LOSS_CHECK     = (1<<0),   // 1
        DISABLE_YAW_IMBALANCE_WARNING = (1<<1),   // 2
        RELEASE_GRIPPER_ON_THRUST_LOSS = (1<<2),  // 4
    };

    // type of fast rate attitude controller in operation
    enum class FastRateType : uint8_t {
        FAST_RATE_DISABLED            = 0,
        FAST_RATE_DYNAMIC             = 1,
        FAST_RATE_FIXED_ARMED         = 2,
        FAST_RATE_FIXED               = 3,
    };

    FastRateType get_fast_rate_type() const { return FastRateType(g2.att_enable.get()); }

    // returns true if option is enabled for this vehicle
    bool option_is_enabled(FlightOption option) const {
        return (g2.flight_options & uint32_t(option)) != 0;
    }

    static constexpr int8_t _failsafe_priorities[] = {
                                                      (int8_t)FailsafeAction::TERMINATE,
                                                      (int8_t)FailsafeAction::LAND,
                                                      (int8_t)FailsafeAction::RTL,
                                                      (int8_t)FailsafeAction::SMARTRTL_LAND,
                                                      (int8_t)FailsafeAction::SMARTRTL,
                                                      (int8_t)FailsafeAction::NONE,
                                                      -1 // the priority list must end with a sentinel of -1
                                                     };

    #define FAILSAFE_LAND_PRIORITY 1
    static_assert(_failsafe_priorities[FAILSAFE_LAND_PRIORITY] == (int8_t)FailsafeAction::LAND,
                  "FAILSAFE_LAND_PRIORITY must match the entry in _failsafe_priorities");
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");



    // AP_State.cpp
    void set_auto_armed(bool b);
    void set_failsafe_radio(bool b);
    void set_failsafe_gcs(bool b);
    void update_using_interlock();

    // Copter.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;

    bool is_landing() const override;
    bool is_taking_off() const override;
    void rc_loop();
    void throttle_loop();
    void update_batt_compass(void);
    void loop_rate_logging();
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();
    void read_AHRS(void);
    void update_altitude();
    bool get_wp_distance_m(float &distance) const override;
    bool get_wp_bearing_deg(float &bearing) const override;
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;
    bool get_rate_ef_targets(Vector3f& rate_ef_targets) const override;



    // if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    struct RateControllerRates {
        uint8_t fast_logging_rate;
        uint8_t medium_logging_rate;
        uint8_t filter_rate;
        uint8_t main_loop_rate;
    };

    LowPassFilterFloat yaw_I_filt{0.05f};
    uint32_t last_yaw_warn_ms;

    // esc_calibration.cpp
    void esc_calibration_startup_check();
    void esc_calibration_passthrough();
    void esc_calibration_auto();
    void esc_calibration_notify();
    void esc_calibration_setup();

   
#if HAL_LOGGING_ENABLED
    enum class LandDetectorLoggingFlag : uint16_t {
        LANDED               = 1U <<  0,
        LANDED_MAYBE         = 1U <<  1,
        LANDING              = 1U <<  2,
        STANDBY_ACTIVE       = 1U <<  3,
        WOW                  = 1U <<  4,
        RANGEFINDER_BELOW_2M = 1U <<  5,
        DESCENT_RATE_LOW     = 1U <<  6,
        ACCEL_STATIONARY     = 1U <<  7,
        LARGE_ANGLE_ERROR    = 1U <<  8,
        LARGE_ANGLE_REQUEST  = 1U <<  8,
        MOTOR_AT_LOWER_LIMIT = 1U <<  9,
        THROTTLE_MIX_AT_MIN  = 1U << 10,
    };
    struct {
        uint32_t last_logged_ms;
        uint32_t last_logged_count;
        uint16_t last_logged_flags;
    } land_detector;
    void Log_LDET(uint16_t logging_flags, uint32_t land_detector_count);
#endif

 
#if HAL_LOGGING_ENABLED
    // methods for AP_Vehicle:
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override;

    // Log.cpp
    void Log_Write_Control_Tuning();
    void Log_Write_Attitude();
    void Log_Write_Rate();
    void Log_Write_EKF_POS();
    void Log_Write_PIDS();
    void Log_Write_Data(LogDataID id, int32_t value);
    void Log_Write_Data(LogDataID id, uint32_t value);
    void Log_Write_Data(LogDataID id, int16_t value);
    void Log_Write_Data(LogDataID id, uint16_t value);
    void Log_Write_Data(LogDataID id, float value);
    void Log_Write_PTUN(uint8_t param, float tuning_val, float tune_min, float tune_max, float norm_in);
    void Log_Write_Guided_Position_Target(ModeGuided::SubMode submode, const Vector3f& pos_target_m, bool is_terrain_alt, const Vector3f& vel_target_ms, const Vector3f& accel_target_mss);
    void Log_Write_Guided_Attitude_Target(ModeGuided::SubMode submode, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate);
    void Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out);
    void Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Write_Rate_Thread_Dt(float dt, float dtAvg, float dtMax, float dtMin);
#endif  // HAL_LOGGING_ENABLED


    // motor_test.cpp
    void motor_test_output();
    bool mavlink_motor_control_check(const GCS_MAVLINK &gcs_chan, bool check_rc, const char* mode);
    MAV_RESULT mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, uint8_t motor_seq, uint8_t throttle_type, float throttle_value, float timeout_sec, uint8_t motor_count);
    void motor_test_stop();

    // motors.cpp
    void auto_disarm_check();
    void motors_output(bool full_push = true);
    void motors_output_main();
    void lost_vehicle_check();

    // Parameters.cpp
    void load_parameters(void) override;
    void convert_pid_parameters(void);


    // system.cpp
    void init_ardupilot() override;
    void update_auto_armed();
    bool should_log(uint32_t mask);
    const char* get_frame_string() const;
    void allocate_motors(void);

    bool started_rate_thread;
    bool using_rate_thread;

};

extern Copter copter;

using AP_HAL::millis;
using AP_HAL::micros;
