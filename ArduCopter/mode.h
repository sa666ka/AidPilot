#pragma once

#include "Copter.h"
#include <AP_Math/chirp.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h> // TODO why is this needed if Copter.h includes this
#include <AP_HAL/Semaphores.h>


class Parameters;
class ParametersG2;

class GCS_Copter;

// object shared by both Guided and Auto for takeoff.
// position controller controls vehicle but the user can control the yaw.
class _AutoTakeoff {
public:
    void run();
    void start_m(float complete_alt_m, bool is_terrain_alt);
    bool get_completion_pos_neu_m(Vector3p& pos_neu_m);

    bool complete;          // true when takeoff is complete

private:
    // altitude above-ekf-origin below which auto takeoff does not control horizontal position
    bool no_nav_active;
    float no_nav_alt_m;

    // auto takeoff variables
    float complete_alt_m;          // completion altitude expressed in m above ekf origin or above terrain (depending upon auto_takeoff_terrain_alt)
    bool is_terrain_alt;            // true if altitudes are above terrain
    Vector3p complete_pos_neu_m;   // target takeoff position as offset from ekf origin in m
};

#if AC_PAYLOAD_PLACE_ENABLED
class PayloadPlace {
public:
    void run();
    void start_descent();
    bool verify();

    enum class State : uint8_t {
        FlyToLocation,
        Descent_Start,
        Descent,
        Release,
        Releasing,
        Delay,
        Ascent_Start,
        Ascent,
        Done,
    };

    // these are set by the Mission code:
    State state = State::Descent_Start; // records state of payload place
    float descent_max_m;

private:

    uint32_t descent_established_time_ms; // milliseconds
    uint32_t place_start_time_ms; // milliseconds
    float descent_thrust_level;
    float descent_start_altitude_m;
    float descent_speed_ms;
};
#endif

class Mode {
    friend class PayloadPlace;

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
        AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        TURTLE =       28,  // Flip over after crash

        // Mode number 30 reserved for "offboard" for external/lua control.

        // Mode number 127 reserved for the "drone show mode" in the Skybrush
        // fork at https://github.com/skybrush-io/ardupilot
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    friend class _AutoTakeoff;

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // child classes should override these methods
    virtual bool init(bool ignore_checks) {
        return true;
    }
    virtual void exit() {};
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(AP_Arming::Method method) const = 0;
    virtual bool is_autopilot() const = 0;
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual bool logs_attitude() const { return false; }
    virtual bool allows_save_trim() const { return false; }
    virtual bool allows_auto_trim() const { return false; }
    virtual bool allows_autotune() const { return false; }
    virtual bool allows_flip() const { return false; }
    virtual bool crash_check_enabled() const { return true; }

    // "no pilot input" here means eg. in RC failsafe
    virtual bool allows_entry_in_rc_failsafe() const { return true; }


    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    virtual bool allows_GCS_or_SCR_arming_with_throttle_high() const { return false; }


    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    bool do_user_takeoff_U_m(float takeoff_alt_m, bool must_navigate);
    virtual bool is_taking_off() const;
    static void takeoff_stop() { takeoff.stop(); }

    virtual bool is_landing() const { return false; }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const { return false; }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc) const { return false; };
    virtual float wp_bearing_deg() const { return 0; }
    virtual float wp_distance_m() const { return 0.0f; }
    virtual float crosstrack_error_m() const { return 0.0f;}

    // functions to support MAV_CMD_DO_CHANGE_SPEED
    virtual bool set_speed_NE_ms(float speed_ne_ms) {return false;}
    virtual bool set_speed_up_ms(float speed_up_ms) {return false;}
    virtual bool set_speed_down_ms(float speed_down_ms) {return false;}

    virtual float get_alt_above_ground_m(void) const;

    // pilot input processing
    void get_pilot_desired_lean_angles_rad(float &roll_out_rad, float &pitch_out_rad, float angle_max_rad, float angle_limit_rad) const;
    float get_pilot_desired_yaw_rate_rads() const;
    Vector2f get_pilot_desired_velocity(float vel_max) const;
    float get_pilot_desired_throttle() const;

    // returns climb target_rate reduced to avoid obstacles and
    // altitude fence
    float get_avoidance_adjusted_climbrate_ms(float target_rate_ms);

    // send output to the motors, can be overridden by subclasses
    virtual void output_to_motors();

    // returns true if pilot's yaw input should be used to adjust vehicle's heading
    virtual bool use_pilot_yaw() const {return true; }

    // pause and resume a mode
    virtual bool pause() { return false; };
    virtual bool resume() { return false; };

    // handle situations where the vehicle is on the ground waiting for takeoff
    void make_safe_ground_handling(bool force_throttle_unlimited = false);

protected:

    // helper functions
    bool is_disarmed_or_landed() const;
    void zero_throttle_and_relax_ac(bool spool_up = false);
    void zero_throttle_and_hold_attitude();

    // Return stopping point as a location with above origin alt frame
    Location get_stopping_point() const;

    // functions to control normal landing.  pause_descent is true if vehicle should not descend
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horiz_and_vert_control(bool pause_descent = false) {
        land_run_horizontal_control();
        land_run_vertical_control(pause_descent);
    }

#if AC_PAYLOAD_PLACE_ENABLED
    // payload place flight behaviour:
    static PayloadPlace payload_place;
#endif

    // run normal or precision landing (if enabled)
    // pause_descent is true if vehicle should not descend
    void land_run_normal_or_precland(bool pause_descent = false);


    // return expected input throttle setting to hover:
    virtual float throttle_hover() const;

    // Alt_Hold based flight mode states used in Alt_Hold, Loiter, and Sport
    enum class AltHoldModeState {
        MotorStopped,
        Takeoff,
        Landed_Ground_Idle,
        Landed_Pre_Takeoff,
        Flying
    };
    AltHoldModeState get_alt_hold_state_U_ms(float target_climb_rate_ms);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_AHRS &ahrs;
    AC_AttitudeControl *&attitude_control;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;

    // note that we support two entirely different automatic takeoffs:

    // "user-takeoff", which is available in modes such as ALT_HOLD
    // (see has_user_takeoff method).  "user-takeoff" is a simple
    // reach-altitude-based-on-pilot-input-or-parameter routine.

    // "auto-takeoff" is used by both Guided and Auto, and is
    // basically waypoint navigation with pilot yaw permitted.

    // user-takeoff support; takeoff state is shared across all mode instances
    class _TakeOff {
    public:
        void start_m(float alt_m);
        void stop();
        void do_pilot_takeoff_ms(float& pilot_climb_rate_ms);
        bool triggered_ms(float target_climb_rate_ms) const;

        bool running() const { return _running; }
    private:
        bool _running;
        float take_off_start_alt_m;
        float take_off_complete_alt_m;
    };

    static _TakeOff takeoff;

    virtual bool do_user_takeoff_start_m(float takeoff_alt_m);

    static _AutoTakeoff auto_takeoff;

public:
    // Navigation Yaw control
    class AutoYaw {

    public:

        // Autopilot Yaw Mode enumeration
        enum class Mode {
            HOLD =             0,   // hold zero yaw rate
            LOOK_AT_NEXT_WP =  1,   // point towards next waypoint (no pilot input accepted)
            ROI =              2,   // point towards a location held in roi_ne_m (no pilot input accepted)
            FIXED =            3,   // point towards a particular angle (no pilot input accepted)
            LOOK_AHEAD =       4,   // point in the direction the copter is moving
            RESET_TO_ARMED_YAW = 5, // point towards heading at time motors were armed
            ANGLE_RATE =       6,   // turn at a specified rate from a starting angle
            RATE =             7,   // turn at a specified rate (held in auto_yaw_rate)
            CIRCLE =           8,   // use AC_Circle's provided yaw (used during Loiter-Turns commands)
            PILOT_RATE =       9,   // target rate from pilot stick
            WEATHERVANE =     10,   // yaw into wind
        };

        // mode(): current method of determining desired yaw:
        Mode mode() const { return _mode; }
        void set_mode_to_default(bool rtl);
        void set_mode(Mode new_mode);
        Mode default_mode(bool rtl) const;

        void set_rate_rad(float turn_rate_rads);

        // set_roi(...): set a "look at" location:
        void set_roi(const Location &roi_location);

        void set_fixed_yaw_rad(float angle_rad,
                               float turn_rate_rads,
                               int8_t direction,
                               bool relative_angle);

        void set_yaw_angle_and_rate_rad(float yaw_angle_rad, float yaw_rate_rads);

        void set_yaw_angle_offset_deg(const float yaw_angle_offset_deg);

        bool reached_fixed_yaw_target();

        AC_AttitudeControl::HeadingCommand get_heading();

    private:

        // yaw_rad(): main product of AutoYaw; the heading:
        float yaw_rad();

        // rate_rads(): desired yaw rate in radians/second:
        float rate_rads();

        // Returns the yaw angle (in radians) representing the direction of horizontal motion.
        float look_ahead_yaw_rad();

        float roi_yaw_rad() const;

        // auto flight mode's yaw mode
        Mode _mode = Mode::LOOK_AT_NEXT_WP;
        Mode _last_mode;

        // Yaw will point at this location if mode is set to Mode::ROI
        Vector3f roi_ne_m;

        // yaw used for YAW_FIXED yaw_mode
        float _fixed_yaw_offset_rad;

        // Radians/s we should turn
        float _fixed_yaw_slewrate_rads;

        // time of the last yaw update
        uint32_t _last_update_ms;

        // heading when in yaw_look_ahead_yaw
        float _look_ahead_yaw_rad;

        // turn heading (rad) and rate (rads) when auto_yaw_mode is set to AUTO_YAW_RATE
        float _yaw_angle_rad;
        float _yaw_rate_rads;
        float _pilot_yaw_rate_rads;
    };
    static AutoYaw auto_yaw;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    float get_pilot_desired_climb_rate_ms() const;
    float get_non_takeoff_throttle() const;
    void update_simple_mode();
    bool set_mode(Mode::Number mode, ModeReason reason);
    void set_land_complete(bool b);
    GCS_Copter &gcs() const;
    float get_pilot_speed_up_ms() const;
    float get_pilot_speed_dn_ms() const;
    float get_pilot_accel_U_mss() const;
    // end pass-through functions
};

class ModeAltHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ALT_HOLD; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }
    bool allows_auto_trim() const override { return true; }
    bool allows_save_trim() const override { return true; }

protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};


class ModeGuided : public Mode {

public:
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::GUIDED; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }


    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    // Sets guided's angular target submode: Using a rotation quaternion, angular velocity, and climbrate or thrust (depends on user option)
    // attitude_quat: IF zero: ang_vel (angular velocity) must be provided even if all zeroes
    //                IF non-zero: attitude_control is performed using both the attitude quaternion and angular velocity
    // ang_vel: angular velocity (rad/s)
    // climb_rate_ms_or_thrust: represents either the climb_rate (m/s) or thrust scaled from [0, 1], unitless
    // use_thrust: IF true: climb_rate_ms_or_thrust represents thrust
    //             IF false: climb_rate_ms_or_thrust represents climb_rate (m/s)
    void set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel, float climb_rate_ms_or_thrust, bool use_thrust);

    bool set_pos_NEU_m(const Vector3f& pos_neu_m, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool is_terrain_alt = false);
    bool set_destination(const Location& dest_loc, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);
    bool get_wp(Location &loc) const override;
    void set_accel_NEU_mss(const Vector3f& accel_neu_mss, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    void set_vel_NEU_ms(const Vector3f& vel_neu_ms, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    void set_vel_accel_NEU_m(const Vector3f& vel_neu_ms, const Vector3f& accel_neu_mss, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    bool set_pos_vel_NEU_m(const Vector3f& pos_neu_m, const Vector3f& vel_neu_ms, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);
    bool set_pos_vel_accel_NEU_m(const Vector3f& pos_neu_m, const Vector3f& vel_neu_ms, const Vector3f& accel_neu_mss, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);

    // get position, velocity and acceleration targets
    const Vector3p& get_target_pos_NEU_m() const;
    const Vector3f& get_target_vel_NEU_ms() const;
    const Vector3f& get_target_accel_NEU_mss() const;

    // returns true if GUIDED_OPTIONS param suggests SET_ATTITUDE_TARGET's "thrust" field should be interpreted as thrust instead of climb rate
    bool set_attitude_target_provides_thrust() const;
    bool stabilizing_pos_NE() const;
    bool stabilizing_vel_NE() const;
    bool use_wpnav_for_position_control() const;

    void limit_clear();
    void limit_init_time_and_pos();
    void limit_set(uint32_t timeout_ms, float alt_min_m, float alt_max_m, float horiz_max_m);
    bool limit_check();

    bool is_taking_off() const override;
    
    bool set_speed_NE_ms(float speed_ne_ms) override;
    bool set_speed_up_ms(float speed_up_ms) override;
    bool set_speed_down_ms(float speed_down_ms) override;

    // initialises position controller to implement take-off
    // takeoff_alt_m is interpreted as alt-above-home (in m) or alt-above-terrain if a rangefinder is available
    bool do_user_takeoff_start_m(float takeoff_alt_m) override;

    enum class SubMode {
        TakeOff,
        WP,
        Pos,
        PosVelAccel,
        VelAccel,
        Accel,
        Angle,
    };

    SubMode submode() const { return guided_mode; }

    void angle_control_start();
    void angle_control_run();

    // return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rate control
    uint32_t get_timeout_ms() const;

    bool use_pilot_yaw() const override;

    // pause continue in guided mode
    bool pause() override;
    bool resume() override;

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override;

private:

    // enum for GUID_OPTIONS parameter
    enum class Option : uint32_t {
        AllowArmingFromTX   = (1U << 0),
        // this bit is still available, pilot yaw was mapped to bit 2 for symmetry with auto
        IgnorePilotYaw      = (1U << 2),
        SetAttitudeTarget_ThrustAsThrust = (1U << 3),
        DoNotStabilizePositionXY = (1U << 4),
        DoNotStabilizeVelocityXY = (1U << 5),
        WPNavUsedForPosControl = (1U << 6),
        AllowWeatherVaning = (1U << 7)
    };

    // returns true if the Guided-mode-option is set (see GUID_OPTIONS)
    bool option_is_enabled(Option option) const;

    // wp controller
    void wp_control_start();
    void wp_control_run();

    void pva_control_start();
    void pos_control_start();
    void accel_control_start();
    void velaccel_control_start();
    void posvelaccel_control_start();
    void takeoff_run();
    void pos_control_run();
    void accel_control_run();
    void velaccel_control_run();
    void pause_control_run();
    void posvelaccel_control_run();
    void set_yaw_state_rad(bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_angle);

    // controls which controller is run (pos or vel):
    static SubMode guided_mode;
    static bool send_notification;     // used to send one time notification to ground station
    static bool takeoff_complete;      // true once takeoff has completed (used to trigger retracting of landing gear)

    // guided mode is paused or not
    static bool _paused;
};

#if AP_SCRIPTING_ENABLED
// Mode which behaves as guided with custom mode number and name
class ModeGuidedCustom : public ModeGuided {
public:
    // constructor registers custom number and names
    ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name);

    bool init(bool ignore_checks) override;

    Number mode_number() const override { return number; }

    const char *name() const override { return full_name; }
    const char *name4() const override { return short_name; }

    // State object which can be edited by scripting
    AP_Vehicle::custom_mode_state state;

private:
    const Number number;
    const char* full_name;
    const char* short_name;
};
#endif

class ModeGuidedNoGPS : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;
    Number mode_number() const override { return Number::GUIDED_NOGPS; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "GUIDED_NOGPS"; }
    const char *name4() const override { return "GNGP"; }

private:

};


class ModeLand : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::LAND; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool is_landing() const override { return true; };


    void do_not_use_GPS();

    // returns true if LAND mode is trying to control X/Y position
    bool controlling_position() const { return control_position; }

    void set_land_pause(bool new_value) { land_pause = new_value; }

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

    void gps_run();
    void nogps_run();

    bool control_position; // true if we are using an external reference to control position

    uint32_t land_start_time;
    bool land_pause;
};

/*
class ModeLoiter : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::LOITER; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool allows_autotune() const override { return true; }
    bool allows_auto_trim() const override { return true; }


protected:

    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return pos_control->crosstrack_error_m();}

};


class ModePosHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::POSHOLD; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool allows_autotune() const override { return true; }
    bool allows_auto_trim() const override { return true; }

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "PHLD"; }

private:

    void update_pilot_lean_angle_rad(float &lean_angle_filtered_rad, float &lean_angle_raw_rad);
    float mix_controls(float mix_ratio, float first_control, float second_control);
    void update_brake_angle_from_velocity(float &brake_angle_rad, float velocity_ms);
    void init_wind_comp_estimate();
    void update_wind_comp_estimate();
    void get_wind_comp_lean_angles_rad(float &roll_angle_rad, float &pitch_angle_rad);
    void roll_controller_to_pilot_override();
    void pitch_controller_to_pilot_override();

    enum class RPMode {
        PILOT_OVERRIDE=0,            // pilot is controlling this axis (i.e. roll or pitch)
        BRAKE,                       // this axis is braking towards zero
        BRAKE_READY_TO_LOITER,       // this axis has completed braking and is ready to enter loiter mode (both modes must be this value before moving to next stage)
        BRAKE_TO_LOITER,             // both vehicle's axis (roll and pitch) are transitioning from braking to loiter mode (braking and loiter controls are mixed)
        LOITER,                      // both vehicle axis are holding position
        CONTROLLER_TO_PILOT_OVERRIDE // pilot has input controls on this axis and this axis is transitioning to pilot override (other axis will transition to brake if no pilot input)
    };

    RPMode roll_mode;
    RPMode pitch_mode;

    // pilot input related variables
    float pilot_roll_rad;   // filtered roll lean angle commanded by the pilot. Slowly returns to zero when stick is released
    float pilot_pitch_rad;  // filtered pitch lean angle commanded by the pilot. Slowly returns to zero when stick is released


    // braking related variables
    struct {
        bool  time_updated_roll;                    // true if braking timeout (roll) has been re-estimated
        bool  time_updated_pitch;                   // true if braking timeout (pitch) has been re-estimated
        float gain;                                 // braking gain converting velocity (m/s) -> lean angle (rad)
        float roll_rad;                             // braking roll angle (rad)
        float pitch_rad;                            // braking pitch angle (rad)
        uint32_t start_time_roll_ms;                // time (ms) when braking on roll axis begins
        uint32_t start_time_pitch_ms;               // time (ms) when braking on pitch axis begins
        float angle_max_roll_rad;                   // peak roll angle (rad) during braking, used to detect vehicle flattening
        float angle_max_pitch_rad;                  // peak pitch angle (rad) during braking, used to detect vehicle flattening
        uint32_t loiter_transition_start_time_ms;   // time (ms) when transition from brake to loiter started
    } brake;

    // loiter transition timing (ms)
    uint32_t controller_to_pilot_start_time_roll_ms;    // time (ms) when transition from controller to pilot roll input began
    uint32_t controller_to_pilot_start_time_pitch_ms;   // time (ms) when transition from controller to pilot pitch input began

    // cached controller outputs for mix during transition (radians)
    float controller_final_roll_rad;    // final roll output (rad) from controller before transition to pilot input
    float controller_final_pitch_rad;   // final pitch output (rad) from controller before transition to pilot input

    // wind compensation related variables
    Vector2f wind_comp_ne_mss;              // earth-frame accel estimate (N,E), m/s^2, low-pass filtered
    float wind_comp_roll_rad;           // roll angle (rad) to counter wind
    float wind_comp_pitch_rad;          // pitch angle (rad) to counter wind
    uint32_t wind_comp_start_time_ms;   // time (ms) when wind compensation updates are started

    // final outputs (radians)
    float roll_rad;     // final roll angle sent to attitude controller
    float pitch_rad;    // final pitch angle sent to attitude controller
};


class ModeRTL : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::RTL; }

    bool init(bool ignore_checks) override;
    void run() override {
        return run(true);
    }
    void run(bool disarm_on_land);

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }


    // for reporting to GCS
    bool get_wp(Location &loc) const override;

    bool use_pilot_yaw() const override;

    bool set_speed_NE_ms(float speed_ne_ms) override;
    bool set_speed_up_ms(float speed_up_ms) override;
    bool set_speed_down_ms(float speed_down_ms) override;

    // RTL states
    enum class SubMode : uint8_t {
        STARTING,
        INITIAL_CLIMB,
        RETURN_HOME,
        LOITER_AT_HOME,
        FINAL_DESCENT,
        LAND
    };
    SubMode state() { return _state; }

    // this should probably not be exposed
    bool state_complete() const { return _state_complete; }

    virtual bool is_landing() const override;

    void restart_without_terrain();

    // enum for RTL_ALT_TYPE parameter
    enum class RTLAltType : int8_t {
        RELATIVE = 0,
        TERRAIN = 1
    };
    ModeRTL::RTLAltType get_alt_type() const;

protected:

    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // for reporting to GCS
    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return wp_nav->crosstrack_error_m();}

    void descent_start();
    void descent_run();
    void land_start();
    void land_run(bool disarm_on_land);

    void set_descent_target_alt(uint32_t alt) { rtl_path.descent_target.alt = alt; }

private:

    void climb_start();
    void return_start();
    void climb_return_run();
    void loiterathome_start();
    void loiterathome_run();
    void build_path();
    void compute_return_target();

    SubMode _state = SubMode::INITIAL_CLIMB;  // records state of rtl (initial climb, returning home, etc)
    bool _state_complete = false; // set to true if the current state is completed

    struct {
        // NEU w/ Z element alt-above-ekf-origin unless use_terrain is true in which case Z element is alt-above-terrain
        Location origin_point;
        Location climb_target;
        Location return_target;
        Location descent_target;
        bool land;
    } rtl_path;

    // return target alt type
    enum class ReturnTargetAltType {
        RELATIVE = 0,
        RANGEFINDER = 1,
        TERRAINDATABASE = 2
    };

    // Loiter timer - Records how long we have been in loiter
    uint32_t _loiter_start_time;

    bool terrain_following_allowed;

    // enum for RTL_OPTIONS parameter
    enum class Options : int32_t {
        // First pair of bits are still available, pilot yaw was mapped to bit 2 for symmetry with auto
        IgnorePilotYaw    = (1U << 2),
    };

};


class ModeSmartRTL : public ModeRTL {

public:
    // inherit constructor
    using ModeRTL::Mode;
    Number mode_number() const override { return Number::SMART_RTL; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

    void save_position();
    void exit() override;

    bool is_landing() const override;
    bool use_pilot_yaw() const override;

    // Safe RTL states
    enum class SubMode : uint8_t {
        WAIT_FOR_PATH_CLEANUP,
        PATH_FOLLOW,
        PRELAND_POSITION,
        DESCEND,
        LAND
    };

protected:

    const char *name() const override { return "SMARTRTL"; }
    const char *name4() const override { return "SRTL"; }

    // for reporting to GCS
    bool get_wp(Location &loc) const override;
    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return wp_nav->crosstrack_error_m();}

private:

    void wait_cleanup_run();
    void path_follow_run();
    void pre_land_position_run();
    void land();
    SubMode smart_rtl_state = SubMode::PATH_FOLLOW;

    // keep track of how long we have failed to get another return
    // point while following our path home.  If we take too long we
    // may choose to land the vehicle.
    uint32_t path_follow_last_pop_fail_ms;

    // backup last popped point so that it can be restored to the path
    // if vehicle exits SmartRTL mode before reaching home. invalid if zero
    Vector3f dest_NED_backup;
};


class ModeSport : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::SPORT; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

protected:

    const char *name() const override { return "SPORT"; }
    const char *name4() const override { return "SPRT"; }

private:

};
*/

class ModeStabilize : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::STABILIZE; }

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool allows_save_trim() const override { return true; }
    bool allows_auto_trim() const override { return true; }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

private:

};
