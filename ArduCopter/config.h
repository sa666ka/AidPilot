//
#pragma once


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// Default and automatic configuration details.
//
#include "defines.h"


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduCopter
#endif

#ifndef ARMING_DELAY_SEC
    # define ARMING_DELAY_SEC 2.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// FRAME_CONFIG
//
#ifndef FRAME_CONFIG
 # define FRAME_CONFIG   MULTICOPTER_FRAME
#endif

// AUTO Mode
#ifndef WP_YAW_BEHAVIOR_DEFAULT
 # define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
#endif

#ifndef YAW_LOOK_AHEAD_MIN_SPEED_MS
 # define YAW_LOOK_AHEAD_MIN_SPEED_MS 1     // minimum ground speed in m/s required before copter is aimed at ground course
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
// default RC speed in Hz
#ifndef RC_FAST_SPEED
   #   define RC_FAST_SPEED 490
#endif

//////////////////////////////////////////////////////////////////////////////
// Rangefinder
//

#ifndef RANGEFINDER_FILT_DEFAULT
 # define RANGEFINDER_FILT_DEFAULT 0.5f     // filter for rangefinder distance
#endif

#ifndef SURFACE_TRACKING_TIMEOUT_MS
 # define SURFACE_TRACKING_TIMEOUT_MS  1000 // surface tracking target alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

// prearm GPS hdop check
#ifndef GPS_HDOP_GOOD_DEFAULT
 # define GPS_HDOP_GOOD_DEFAULT         140     // minimum hdop that represents a good position.  used during pre-arm checks if fence is enabled
#endif

// missing terrain data failsafe
#ifndef FS_TERRAIN_TIMEOUT_MS
 #define FS_TERRAIN_TIMEOUT_MS          5000     // 5 seconds of missing terrain data will trigger failsafe (RTL)
#endif

// pre-arm baro vs inertial nav max alt disparity
#ifndef PREARM_MAX_ALT_DISPARITY_M
 # define PREARM_MAX_ALT_DISPARITY_M    1.0      // barometer and inertial nav altitude must be within this many meters
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF Failsafe
#ifndef FS_EKF_ACTION_DEFAULT
 # define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_LAND  // EKF failsafe triggers land by default
#endif
#ifndef FS_EKF_THRESHOLD_DEFAULT
 # define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

#ifndef EKF_ORIGIN_MAX_ALT_KM
 # define EKF_ORIGIN_MAX_ALT_KM         50   // EKF origin and home must be within 50km vertically
#endif

#ifndef FS_EKF_FILT_DEFAULT
# define FS_EKF_FILT_DEFAULT     5.0f    // frequency cutoff of EKF variance filters
#endif

//////////////////////////////////////////////////////////////////////////////
// Nav-Guided - allows external nav computer to control vehicle
#ifndef AC_NAV_GUIDED
 # define AC_NAV_GUIDED    1
#endif


//////////////////////////////////////////////////////////////////////////////
// Guided mode - control vehicle's position or angles from GCS
#ifndef MODE_GUIDED_ENABLED
# define MODE_GUIDED_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// GuidedNoGPS mode - control vehicle's angles from GCS
#ifndef MODE_GUIDED_NOGPS_ENABLED
# define MODE_GUIDED_NOGPS_ENABLED 1
#endif


//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
//

#ifndef FLIGHT_MODE_1
 # define FLIGHT_MODE_1                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_2
 # define FLIGHT_MODE_2                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_3
 # define FLIGHT_MODE_3                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_4
 # define FLIGHT_MODE_4                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_5
 # define FLIGHT_MODE_5                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_6
 # define FLIGHT_MODE_6                  Mode::Number::STABILIZE
#endif


//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//
#ifndef FS_THR_VALUE_DEFAULT
 # define FS_THR_VALUE_DEFAULT             975
#endif

//////////////////////////////////////////////////////////////////////////////
// Takeoff
//
#ifndef PILOT_TKOFF_ALT_DEFAULT
 # define PILOT_TKOFF_ALT_DEFAULT           0     // default final alt above home for pilot initiated takeoff
#endif


//////////////////////////////////////////////////////////////////////////////
// Landing
//
#ifndef LAND_SPEED
 # define LAND_SPEED    50          // the descent speed for the final stage of landing in cm/s
#endif
#ifndef LAND_REPOSITION_DEFAULT
 # define LAND_REPOSITION_DEFAULT   1   // by default the pilot can override roll/pitch during landing
#endif
#ifndef LAND_WITH_DELAY_MS
 # define LAND_WITH_DELAY_MS        4000    // default delay (in milliseconds) when a land-with-delay is triggered during a failsafe event
#endif
#ifndef LAND_CANCEL_TRIGGER_THR
 # define LAND_CANCEL_TRIGGER_THR   700     // land is cancelled by input throttle above 700
#endif
#ifndef LAND_RANGEFINDER_MIN_ALT_M
#define LAND_RANGEFINDER_MIN_ALT_M  2.0
#endif

//////////////////////////////////////////////////////////////////////////////
// Landing Detector
//
#ifndef LAND_DETECTOR_TRIGGER_SEC
 # define LAND_DETECTOR_TRIGGER_SEC         1.0f    // number of seconds to detect a landing
#endif
#ifndef LAND_AIRMODE_DETECTOR_TRIGGER_SEC
 # define LAND_AIRMODE_DETECTOR_TRIGGER_SEC 3.0f    // number of seconds to detect a landing in air mode
#endif
#ifndef LAND_DETECTOR_MAYBE_TRIGGER_SEC
 # define LAND_DETECTOR_MAYBE_TRIGGER_SEC   0.2f    // number of seconds that means we might be landed (used to reset horizontal position targets to prevent tipping over)
#endif
#ifndef LAND_DETECTOR_ACCEL_LPF_CUTOFF
# define LAND_DETECTOR_ACCEL_LPF_CUTOFF     1.0f    // frequency cutoff of land detector accelerometer filter
#endif
#ifndef LAND_DETECTOR_ACCEL_MAX
# define LAND_DETECTOR_ACCEL_MAX            1.0f    // vehicle acceleration must be under 1m/s/s
#endif
#ifndef LAND_DETECTOR_VEL_Z_MAX
# define LAND_DETECTOR_VEL_Z_MAX              1.0f    // vehicle vertical velocity must be under 1m/s
#endif

//////////////////////////////////////////////////////////////////////////////
// Flight mode definitions
//

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#ifndef ROLL_PITCH_YAW_INPUT_MAX
 # define ROLL_PITCH_YAW_INPUT_MAX      4500        // roll, pitch and yaw input range
#endif
#ifndef DEFAULT_ANGLE_MAX
 # define DEFAULT_ANGLE_MAX         3000            // ANGLE_MAX parameters default value
#endif

//////////////////////////////////////////////////////////////////////////////
// Pilot control defaults
//

#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif

// default maximum vertical velocity and acceleration the pilot may request
#ifndef PILOT_VELZ_MAX
 # define PILOT_VELZ_MAX    250     // maximum vertical velocity in cm/s
#endif
#ifndef PILOT_ACCEL_Z_DEFAULT
 # define PILOT_ACCEL_Z_DEFAULT 250 // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

#ifndef PILOT_Y_RATE_DEFAULT
 # define PILOT_Y_RATE_DEFAULT  202.5   // yaw rotation rate parameter default in deg/s for all mode except ACRO
#endif
#ifndef PILOT_Y_EXPO_DEFAULT
 # define PILOT_Y_EXPO_DEFAULT  0.0     // yaw expo parameter default for all mode except ACRO
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  10
#endif


//////////////////////////////////////////////////////////////////////////////
// Logging control
//

// Default logging bitmask
#ifndef DEFAULT_LOG_BITMASK
 # define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_PID | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif

//////////////////////////////////////////////////////////////////////////////
// Fence, Rally and Terrain and AC_Avoidance defaults
//


#if MODE_GUIDED_NOGPS_ENABLED && !MODE_GUIDED_ENABLED
  #error ModeGuided-NoGPS requires ModeGuided which is disabled
#endif

//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

#ifndef AP_COPTER_AHRS_AUTO_TRIM_ENABLED
# define AP_COPTER_AHRS_AUTO_TRIM_ENABLED 1
#endif

#ifndef CH_MODE_DEFAULT
 # define CH_MODE_DEFAULT   5
#endif


#ifndef HAL_FRAME_TYPE_DEFAULT
#define HAL_FRAME_TYPE_DEFAULT AP_Motors::MOTOR_FRAME_TYPE_X
#endif

#ifndef AC_CUSTOMCONTROL_MULTI_ENABLED
#define AC_CUSTOMCONTROL_MULTI_ENABLED FRAME_CONFIG == MULTICOPTER_FRAME && AP_CUSTOMCONTROL_ENABLED
#endif

#ifndef AC_PAYLOAD_PLACE_ENABLED
#define AC_PAYLOAD_PLACE_ENABLED 1
#endif
