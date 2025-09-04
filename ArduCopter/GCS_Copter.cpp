#include "GCS_Copter.h"

#include "Copter.h"

const char* GCS_Copter::frame_string() const
{
    if (copter.motors == nullptr) {
        return "MultiCopter";
    }
    return copter.motors->get_frame_string();
}


void GCS_Copter::update_vehicle_sensor_status_flags(void)
{
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    // Update position controller flags
    if (copter.pos_control != nullptr) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

        // XY position controller
        if (copter.pos_control->is_active_NE()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        }

        // Z altitude controller
        if (copter.pos_control->is_active_U()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        }
    }



#if AP_RANGEFINDER_ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
    if (copter.rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif



    control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    // only mark propulsion healthy if all of the motors are producing
    // nominal thrust
    if (!copter.motors->get_thrust_boost()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    }
}
