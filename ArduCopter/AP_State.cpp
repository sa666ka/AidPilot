#include "Copter.h"

// ---------------------------------------------
void Copter::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        LOGGER_WRITE_EVENT(LogEvent::AUTO_ARMED);
    }
}


// ---------------------------------------------
void Copter::set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}


// ---------------------------------------------
void Copter::set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gcs = b;
}

// ---------------------------------------------

void Copter::update_using_interlock()
{
    // check if we are using motor interlock control on an aux switch or are in throw mode
    // which uses the interlock to stop motors while the copter is being thrown
    ap.using_interlock = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) != nullptr;

}
