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

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_VEHICLE_ENABLED
#define AP_VEHICLE_ENABLED 1
#endif

#if AP_VEHICLE_ENABLED

/*
  this header holds a parameter structure for each vehicle type for
  parameters needed by multiple libraries
 */

#include "ModeReason.h" // reasons can't be defined in this header due to circular loops


#include <AP_BoardConfig/AP_BoardConfig.h>     // board configuration library
#include <AP_EFI/AP_EFI.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>                    // Notify library
#include <AP_Param/AP_Param.h>
#include <AP_RSSI/AP_RSSI.h>                        // RSSI Library
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library
#include <AP_Networking/AP_Networking.h>
#include <SITL/SITL.h>
#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <Filter/LowPassFilter.h>
#include <Filter/AP_Filter.h>
#include <AP_Stats/AP_Stats.h>              // statistics library
#include <AP_DDS/AP_DDS_config.h>


class AP_DDS_Client;

class AP_Vehicle : public AP_HAL::HAL::Callbacks {

public:

    AP_Vehicle() {
        if (_singleton) {
            AP_HAL::panic("Too many Vehicles");
        }
        AP_Param::setup_object_defaults(this, var_info);
        _singleton = this;
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Vehicle);

    static AP_Vehicle *get_singleton();

    // setup() is called once during vehicle startup to initialise the
    // vehicle object and the objects it contains.  The
    // AP_HAL_MAIN_CALLBACKS pragma creates a main(...) function
    // referencing an object containing setup() and loop() functions.
    // A vehicle is not expected to override setup(), but
    // subclass-specific initialisation can be done in init_ardupilot
    // which is called from setup().
    void setup(void) override final;

    // HAL::Callbacks implementation.
    void loop() override final;

    // set_mode *must* set control_mode_reason
    virtual bool set_mode(const uint8_t new_mode, const ModeReason reason) = 0;
    virtual uint8_t get_mode() const = 0;

    ModeReason get_control_mode_reason() const {
        return control_mode_reason;
    }

    virtual bool current_mode_requires_mission() const { return false; }

    // perform any notifications required to indicate a mode change
    // failed due to a bad mode number being supplied.  This can
    // happen for many reasons - bad mavlink packet and bad mode
    // parameters for example.
    void notify_no_such_mode(uint8_t mode_number);

#if AP_SCHEDULER_ENABLED
    void get_common_scheduler_tasks(const AP_Scheduler::Task*& tasks, uint8_t& num_tasks);
    // implementations *MUST* fill in all passed-in fields or we get
    // Valgrind errors
    virtual void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) = 0;
#endif

    /*
      set the "likely flying" flag. This is not guaranteed to be
      accurate, but is the vehicle codes best guess as to the whether
      the vehicle is currently flying
    */
    void set_likely_flying(bool b) {
        if (b && !likely_flying) {
            _last_flying_ms = AP_HAL::millis();
        }
        likely_flying = b;
    }

    /*
      get the likely flying status. Returns true if the vehicle code
      thinks we are flying at the moment. Not guaranteed to be
      accurate
    */
    bool get_likely_flying(void) const {
        return likely_flying;
    }

    /*
      return time in milliseconds since likely_flying was set
      true. Returns zero if likely_flying is currently false
    */
    uint32_t get_time_flying_ms(void) const {
        if (!likely_flying) {
            return 0;
        }
        return AP_HAL::millis() - _last_flying_ms;
    }

    // returns true if the vehicle has crashed
    virtual bool is_crashed() const;


    // returns true if vehicle is in the process of landing
    virtual bool is_landing() const { return false; }

    // returns true if vehicle is in the process of taking off
    virtual bool is_taking_off() const { return false; }

    // zeroing the RC outputs can prevent unwanted motor movement:
    virtual bool should_zero_rc_outputs_on_reboot() const { return false; }

    // reboot the vehicle in an orderly manner, doing various cleanups
    // and flashing LEDs as appropriate
    void reboot(bool hold_in_bootloader);

    /*
      get the distance to next wp in meters
      return false if failed or n/a
     */
    virtual bool get_wp_distance_m(float &distance) const { return false; }

    /*
      get the current wp bearing in degrees
      return false if failed or n/a
     */
    virtual bool get_wp_bearing_deg(float &bearing) const { return false; }

    /*
      get the current wp crosstrack error in meters
      return false if failed or n/a
     */
    virtual bool get_wp_crosstrack_error_m(float &xtrack_error) const { return false; }

    /*
      Returns the pan and tilt for use by onvif camera in scripting
     */
    virtual bool get_pan_tilt_norm(float &pan_norm, float &tilt_norm) const { return false; }

    // Returns roll and  pitch for OSD Horizon, Plane overrides to correct for VTOL view and fixed wing PTCH_TRIM_DEG
    virtual void get_osd_roll_pitch_rad(float &roll, float &pitch) const;

    /*
     get the target earth-frame angular velocities in rad/s (Z-axis component used by some gimbals)
     */
    virtual bool get_rate_ef_targets(Vector3f& rate_ef_targets) const { return false; }


protected:

    virtual void init_ardupilot() = 0;
    virtual void load_parameters() = 0;
    void load_parameters(AP_Int16 &format_version, const uint16_t expected_format_version);

    virtual void set_control_channels() {}

    // board specific config
    AP_BoardConfig BoardConfig;


#if AP_SCHEDULER_ENABLED
    // main loop scheduler
    AP_Scheduler scheduler;
#endif


#if HAL_LOGGING_ENABLED
    AP_Logger logger;
    AP_Int32 bitmask_unused;
    // method supplied by vehicle to provide log bitmask:
    virtual const AP_Int32 &get_log_bitmask() { return bitmask_unused; }
    virtual const struct LogStructure *get_log_structures() const { return nullptr; }
    virtual uint8_t get_num_log_structures() const { return 0; }
#endif

#if AP_IBUS_TELEM_ENABLED
    AP_IBus_Telem ibus_telem;
#endif

#if AP_RSSI_ENABLED
    AP_RSSI rssi;
#endif


#if AP_SERIALMANAGER_ENABLED
    AP_SerialManager serial_manager;
#endif

    // notification object for LEDs, buzzers etc (parameter set to
    // false disables external leds)
    AP_Notify notify;

#if AP_NETWORKING_ENABLED
    AP_Networking networking;
#endif

#if HAL_EFI_ENABLED
    // EFI Engine Monitor
    AP_EFI efi;
#endif


#if AP_STATS_ENABLED
    // vehicle statistics
    AP_Stats stats;
#endif


    static const struct AP_Param::GroupInfo var_info[];
#if AP_SCHEDULER_ENABLED
    static const struct AP_Scheduler::Task scheduler_tasks[];
#endif

#if OSD_ENABLED
    void publish_osd_info();
#endif
    // call the arming library's update function
    void update_arming();

    // check for motor noise at a particular frequency
    void check_motor_noise();

    ModeReason control_mode_reason = ModeReason::UNKNOWN;

#if AP_SIM_ENABLED
    SITL::SIM sitl;
#endif

#if AP_DDS_ENABLED
    // Declare the dds client for communication with ROS2 and DDS(common for all vehicles)
    AP_DDS_Client *dds_client;
    bool init_dds_client() WARN_IF_UNUSED;
#endif

    // Check if this mode can be entered from the GCS
    bool block_GCS_mode_change(uint8_t mode_num, const uint8_t *mode_list, uint8_t mode_list_length) const;


private:

#if AP_SCHEDULER_ENABLED
    // delay() callback that processing MAVLink packets
    static void scheduler_delay_callback();
#endif

    // if there's been a watchdog reset, notify the world via a
    // statustext:
    void send_watchdog_reset_statustext();


    // decimation for 1Hz update
    uint8_t one_Hz_counter;
    void one_Hz_update();

    bool likely_flying;         // true if vehicle is probably flying
    uint32_t _last_flying_ms;   // time when likely_flying last went true


    static AP_Vehicle *_singleton;

    bool done_safety_init;


    uint32_t _last_internal_errors;  // backup of AP_InternalError::internal_errors bitmask

#if AP_FILTER_ENABLED
    AP_Filters filters;
#endif

    // Bitmask of modes to disable from gcs
    AP_Int32 flight_mode_GCS_block;
};

namespace AP {
    AP_Vehicle *vehicle();
};

extern const AP_HAL::HAL& hal;

extern const AP_Param::Info vehicle_var_info[];

#include "AP_Vehicle_Type.h"

#endif  // AP_VEHICLE_ENABLED
