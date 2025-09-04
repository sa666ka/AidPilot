#include "Copter.h"

#include "RC_Channel_Copter.h"


// defining these two macros and including the RC_Channels_VarInfo header defines the parameter information common to all vehicle types
#define RC_CHANNELS_SUBCLASS RC_Channels_Copter
#define RC_CHANNEL_SUBCLASS RC_Channel_Copter

#include <RC_Channel/RC_Channels_VarInfo.h>

int8_t RC_Channels_Copter::flight_mode_channel_number() const
{
    return copter.g.flight_mode_chan.get();
}

void RC_Channel_Copter::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > copter.num_flight_modes) {
        // should not have been called
        return;
    }

    if (!copter.set_mode((Mode::Number)copter.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        return;
    }
}

bool RC_Channels_Copter::in_rc_failsafe() const
{
    return copter.failsafe.radio;
}

bool RC_Channels_Copter::has_valid_input() const
{
    if (in_rc_failsafe()) {
        return false;
    }
    if (copter.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}

// returns true if throttle arming checks should be run
bool RC_Channels_Copter::arming_check_throttle() const {
    if ((copter.g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0) {
        // center sprung throttle configured, dont run AP_Arming check
        // Copter already checks this case in its own arming checks
        return false;
    }
    return RC_Channels::arming_check_throttle();
}

RC_Channel * RC_Channels_Copter::get_arming_channel(void) const
{
    return copter.channel_yaw;
}

// init_aux_switch_function - initialize aux functions
void RC_Channel_Copter::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::ALTHOLD:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::AUTOTUNE_MODE:
    case AUX_FUNC::AUTOTUNE_TEST_GAINS:
    case AUX_FUNC::BRAKE:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::DRIFT:
    case AUX_FUNC::FLIP:
    case AUX_FUNC::FLOWHOLD:
    case AUX_FUNC::FOLLOW:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::LAND:
    case AUX_FUNC::LOITER:

    case AUX_FUNC::POSHOLD:
    case AUX_FUNC::RESETTOARMEDYAW:
    case AUX_FUNC::RTL:
    case AUX_FUNC::SAVE_TRIM:
    case AUX_FUNC::SAVE_WP:
    case AUX_FUNC::SMART_RTL:
    case AUX_FUNC::STABILIZE:
    case AUX_FUNC::THROW:
    case AUX_FUNC::USER_FUNC1:
    case AUX_FUNC::USER_FUNC2:
    case AUX_FUNC::USER_FUNC3:

    case AUX_FUNC::ZIGZAG:
    case AUX_FUNC::ZIGZAG_Auto:
    case AUX_FUNC::ZIGZAG_SaveWP:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::AUTO_RTL:
    case AUX_FUNC::TURTLE:
    case AUX_FUNC::SIMPLE_HEADING_RESET:
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::TURBINE_START:
    case AUX_FUNC::FLIGHTMODE_PAUSE:
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    case AUX_FUNC::AHRS_AUTO_TRIM:
#endif
        break;
    case AUX_FUNC::ACRO_TRAINER:
    case AUX_FUNC::ATTCON_ACCEL_LIM:
    case AUX_FUNC::ATTCON_FEEDFWD:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::MOTOR_INTERLOCK:

    case AUX_FUNC::PRECISION_LOITER:
#if AP_RANGEFINDER_ENABLED
    case AUX_FUNC::RANGEFINDER:
#endif
    case AUX_FUNC::SIMPLE_MODE:
    case AUX_FUNC::STANDBY:
    case AUX_FUNC::SUPERSIMPLE_MODE:
    case AUX_FUNC::SURFACE_TRACKING:

    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::FORCEFLYING:
    case AUX_FUNC::CUSTOM_CONTROLLER:
    case AUX_FUNC::WEATHER_VANE_ENABLE:
#if AP_RC_TRANSMITTER_TUNING_ENABLED
    case AUX_FUNC::TRANSMITTER_TUNING:
    case AUX_FUNC::TRANSMITTER_TUNING2:
        run_aux_function(ch_option, ch_flag, AuxFuncTrigger::Source::INIT, ch_in);
        break;
#endif  // AP_RC_TRANSMITTER_TUNING_ENABLED
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_function_change_mode - change mode based on an aux switch
// being moved
void RC_Channel_Copter::do_aux_function_change_mode(const Mode::Number mode,
                                                    const AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        copter.set_mode(mode, ModeReason::AUX_FUNCTION);
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (copter.flightmode->mode_number() == mode) {
            rc().reset_mode_switch();
        }
    }
}

// do_aux_function - implement the function invoked by auxiliary switches
bool RC_Channel_Copter::do_aux_function(const AuxFuncTrigger &trigger)
{
    const AUX_FUNC &ch_option = trigger.func;
    const AuxSwitchPos &ch_flag = trigger.pos;

    switch(ch_option) {
        case AUX_FUNC::FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.set_mode(Mode::Number::FLIP, ModeReason::AUX_FUNCTION);
            }
            break;


        case AUX_FUNC::SAVE_TRIM:
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                (copter.flightmode->allows_save_trim()) &&
                (copter.channel_throttle->get_control_in() == 0)) {
                copter.g2.rc_channels.save_trim();
            }
            break;


#if AP_RANGEFINDER_ENABLED
        case AUX_FUNC::RANGEFINDER:
            // enable or disable the rangefinder
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
            break;
#endif // AP_RANGEFINDER_ENABLED


        case AUX_FUNC::LAND:
            do_aux_function_change_mode(Mode::Number::LAND, ch_flag);
            break;

        case AUX_FUNC::GUIDED:
            do_aux_function_change_mode(Mode::Number::GUIDED, ch_flag);
            break;

        case AUX_FUNC::LOITER:
            do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
            break;

        case AUX_FUNC::FOLLOW:
            do_aux_function_change_mode(Mode::Number::FOLLOW, ch_flag);
            break;

        case AUX_FUNC::ATTCON_FEEDFWD:
            // enable or disable feed forward
            copter.attitude_control->bf_feedforward(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            copter.attitude_control->accel_limiting(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::MOTOR_INTERLOCK:

            copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
            break;


        case AUX_FUNC::STABILIZE:
            do_aux_function_change_mode(Mode::Number::STABILIZE, ch_flag);
            break;


        case AUX_FUNC::ALTHOLD:
            do_aux_function_change_mode(Mode::Number::ALT_HOLD, ch_flag);
            break;


        case AUX_FUNC::STANDBY: {
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.standby_active = true;
                    LOGGER_WRITE_EVENT(LogEvent::STANDBY_ENABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Enabled");
                    break;
                default:
                    copter.standby_active = false;
                    LOGGER_WRITE_EVENT(LogEvent::STANDBY_DISABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Disabled");
                    break;
                }
            break;
        }

#if AP_RANGEFINDER_ENABLED
        case AUX_FUNC::SURFACE_TRACKING:
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::GROUND);
                break;
            case AuxSwitchPos::MIDDLE:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::NONE);
                break;
            case AuxSwitchPos::HIGH:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::CEILING);
                break;
            }
            break;
#endif

        case AUX_FUNC::FLIGHTMODE_PAUSE:
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    if (!copter.flightmode->pause()) {
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Flight Mode Pause failed");
                    }
                    break;
                case AuxSwitchPos::MIDDLE:
                    break;
                case AuxSwitchPos::LOW:
                    copter.flightmode->resume();
                    break;
            }
            break;

        case AUX_FUNC::AIRMODE:
            do_aux_function_change_air_mode(ch_flag);
            break;

        case AUX_FUNC::FORCEFLYING:
            do_aux_function_change_force_flying(ch_flag);
            break;


#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
        case AUX_FUNC::AHRS_AUTO_TRIM:
            copter.g2.rc_channels.do_aux_function_ahrs_auto_trim(ch_flag);
            break;
#endif  // AP_COPTER_AHRS_AUTO_TRIM_ENABLED

        case AUX_FUNC::SIMPLE_HEADING_RESET:
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.init_simple_bearing();
                gcs().send_text(MAV_SEVERITY_INFO, "Simple heading reset");
            }
            break;

        case AUX_FUNC::ARMDISARM_AIRMODE:
            RC_Channel::do_aux_function_armdisarm(ch_flag);
            if (copter.arming.is_armed()) {
                copter.ap.armed_with_airmode_switch = true;
            }
            break;

#if AP_RC_TRANSMITTER_TUNING_ENABLED
    case AUX_FUNC::TRANSMITTER_TUNING:
    case AUX_FUNC::TRANSMITTER_TUNING2:
        // do nothing, used in tuning.cpp for transmitter based tuning
        break;
#endif  // AP_RC_TRANSMITTER_TUNING_ENABLED

    default:
        return RC_Channel::do_aux_function(trigger);
    }
    return true;
}

// change air-mode status
void RC_Channel_Copter::do_aux_function_change_air_mode(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.air_mode = AirMode::AIRMODE_ENABLED;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.air_mode = AirMode::AIRMODE_DISABLED;
        break;
    }
}

// change force flying status
void RC_Channel_Copter::do_aux_function_change_force_flying(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.force_flying = true;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.force_flying = false;
        break;
    }
}

// note that this is a method on the RC_Channels object, not the
// individual channel
// save_trim - adds roll and pitch trims from the radio to ahrs
void RC_Channels_Copter::save_trim()
{
    float roll_trim = 0;
    float pitch_trim = 0;
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    if (auto_trim.running) {
        auto_trim.running = false;
    } else {
#endif
    // save roll and pitch trim
    roll_trim = cd_to_rad((float)get_roll_channel().get_control_in());
    pitch_trim = cd_to_rad((float)get_pitch_channel().get_control_in());
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED    
    }
#endif
    AP::ahrs().add_trim(roll_trim, pitch_trim);
    LOGGER_WRITE_EVENT(LogEvent::SAVE_TRIM);
    gcs().send_text(MAV_SEVERITY_INFO, "Trim saved");
}

#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
// start/stop ahrs auto trim
void RC_Channels_Copter::do_aux_function_ahrs_auto_trim(const RC_Channel::AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case RC_Channel::AuxSwitchPos::HIGH:
        if (!copter.flightmode->allows_auto_trim()) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim not allowed in this mode");
            break;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim running");
        // flash the leds
        AP_Notify::flags.save_trim = true;
        auto_trim.running = true;
        break;
    case RC_Channel::AuxSwitchPos::MIDDLE:
        break;
    case RC_Channel::AuxSwitchPos::LOW:
        if (auto_trim.running) {
            AP_Notify::flags.save_trim = false;
            save_trim();
        }
        break;
    }
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
void RC_Channels_Copter::auto_trim_cancel()
{
    auto_trim.running = false;
    AP_Notify::flags.save_trim = false;
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim cancelled");
    // restore original trims
}

void RC_Channels_Copter::auto_trim_run()
{
        if (!auto_trim.running) {
            return;
        }

        // only trim in certain modes:
        if (!copter.flightmode->allows_auto_trim()) {
            auto_trim_cancel();
            return;
        }

        // must be started and stopped mid-air:
        if (copter.ap.land_complete_maybe) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Must be flying to use AUTOTRIM");
            auto_trim_cancel();
            return;
        }
        // calculate roll trim adjustment, divisor set subjectively to give same "feel" as previous RC input method
        float roll_trim_adjustment_rad = copter.attitude_control->get_att_target_euler_rad().x / 20.0f;

        // calculate pitch trim adjustment, divisor set subjectively to give same "feel" as previous RC input method
        float pitch_trim_adjustment_rad = copter.attitude_control->get_att_target_euler_rad().y / 20.0f;

        // add trim to ahrs object, but do not save to permanent storage:
        AP::ahrs().add_trim(roll_trim_adjustment_rad, pitch_trim_adjustment_rad, false);
}

#endif  // AP_COPTER_AHRS_AUTO_TRIM_ENABLED
