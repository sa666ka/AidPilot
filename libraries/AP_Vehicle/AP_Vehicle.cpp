
#include "AP_Vehicle.h"

#if AP_VEHICLE_ENABLED

#include <AP_Common/AP_FWVersion.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_OSD/AP_OSD.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Motors/AP_Motors.h>
#include <AR_Motors/AP_MotorsUGV.h>
#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <GCS_MAVLink/GCS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/sdcard.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#endif
#include <AP_DDS/AP_DDS_Client.h>
#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#define SCHED_TASK(func, rate_hz, max_time_micros, prio) SCHED_TASK_CLASS(AP_Vehicle, &vehicle, func, rate_hz, max_time_micros, prio)

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo AP_Vehicle::var_info[] = {


#if HAL_EFI_ENABLED
    // @Group: EFI
    // @Path: ../AP_EFI/AP_EFI.cpp
    AP_SUBGROUPINFO(efi, "EFI", 9, AP_Vehicle, AP_EFI),
#endif



#if AP_DDS_ENABLED
    // @Group: DDS
    // @Path: ../AP_DDS/AP_DDS_Client.cpp
    AP_SUBGROUPPTR(dds_client, "DDS", 18, AP_Vehicle, AP_DDS_Client),
#endif


#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_Rover)
    // @Param: FLTMODE_GCSBLOCK
    // @DisplayName: Flight mode block from GCS
    // @Description: Bitmask of flight modes to disable for GCS selection. Mode can still be accessed via RC or failsafe.
    // @Bitmask{Copter}: 0:Stabilize
    // @Bitmask{Copter}: 1:Acro
    // @Bitmask{Copter}: 2:AltHold
    // @Bitmask{Copter}: 3:Auto
    // @Bitmask{Copter}: 4:Guided
    // @Bitmask{Copter}: 5:Loiter
    // @Bitmask{Copter}: 6:Circle
    // @Bitmask{Copter}: 7:Drift
    // @Bitmask{Copter}: 8:Sport
    // @Bitmask{Copter}: 9:Flip
    // @Bitmask{Copter}: 10:AutoTune
    // @Bitmask{Copter}: 11:PosHold
    // @Bitmask{Copter}: 12:Brake
    // @Bitmask{Copter}: 13:Throw
    // @Bitmask{Copter}: 14:Avoid_ADSB
    // @Bitmask{Copter}: 15:Guided_NoGPS
    // @Bitmask{Copter}: 16:Smart_RTL
    // @Bitmask{Copter}: 17:FlowHold
    // @Bitmask{Copter}: 18:Follow
    // @Bitmask{Copter}: 19:ZigZag
    // @Bitmask{Copter}: 20:SystemID
    // @Bitmask{Copter}: 21:Heli_Autorotate
    // @Bitmask{Copter}: 22:Auto RTL
    // @Bitmask{Copter}: 23:Turtle
    // @Bitmask{Plane}: 0:Manual
    // @Bitmask{Plane}: 1:Circle
    // @Bitmask{Plane}: 2:Stabilize
    // @Bitmask{Plane}: 3:Training
    // @Bitmask{Plane}: 4:ACRO
    // @Bitmask{Plane}: 5:FBWA
    // @Bitmask{Plane}: 6:FBWB
    // @Bitmask{Plane}: 7:CRUISE
    // @Bitmask{Plane}: 8:AUTOTUNE
    // @Bitmask{Plane}: 9:Auto
    // @Bitmask{Plane}: 10:Loiter
    // @Bitmask{Plane}: 11:Takeoff
    // @Bitmask{Plane}: 12:AVOID_ADSB
    // @Bitmask{Plane}: 13:Guided
    // @Bitmask{Plane}: 14:THERMAL
    // @Bitmask{Plane}: 15:QSTABILIZE
    // @Bitmask{Plane}: 16:QHOVER
    // @Bitmask{Plane}: 17:QLOITER
    // @Bitmask{Plane}: 18:QACRO
    // @Bitmask{Plane}: 19:QAUTOTUNE
    // @Bitmask{Rover}: 0:Manual
    // @Bitmask{Rover}: 1:Acro
    // @Bitmask{Rover}: 2:Steering
    // @Bitmask{Rover}: 3:Loiter
    // @Bitmask{Rover}: 4:Follow
    // @Bitmask{Rover}: 5:Simple
    // @Bitmask{Rover}: 6:Circle
    // @Bitmask{Rover}: 7:Auto
    // @Bitmask{Rover}: 8:RTL
    // @Bitmask{Rover}: 9:SmartRTL
    // @Bitmask{Rover}: 10:Guided
    // @Bitmask{Rover}: 11:Dock
    // @User: Standard
    AP_GROUPINFO("FLTMODE_GCSBLOCK", 20, AP_Vehicle, flight_mode_GCS_block, 0),
#endif // APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_Rover)


#if AP_NETWORKING_ENABLED
    // @Group: NET_
    // @Path: ../AP_Networking/AP_Networking.cpp
    AP_SUBGROUPINFO(networking, "NET_", 21, AP_Vehicle, AP_Networking),

    /*
      the NET_Pn_ parameters need to be in AP_Vehicle as otherwise we
      are too deep in the parameter tree
     */

#if AP_NETWORKING_REGISTER_PORT_ENABLED
#if AP_NETWORKING_NUM_PORTS > 0
    // @Group: NET_P1_
    // @Path: ../AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[0], "NET_P1_", 22, AP_Vehicle, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 1
    // @Group: NET_P2_
    // @Path: ../AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[1], "NET_P2_", 23, AP_Vehicle, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 2
    // @Group: NET_P3_
    // @Path: ../AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[2], "NET_P3_", 24, AP_Vehicle, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 3
    // @Group: NET_P4_
    // @Path: ../AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[3], "NET_P4_", 25, AP_Vehicle, AP_Networking::Port),
#endif
#endif  // AP_NETWORKING_REGISTER_PORT_ENABLED
#endif // AP_NETWORKING_ENABLED

#if AP_FILTER_ENABLED
    // @Group: FILT
    // @Path: ../Filter/AP_Filter.cpp
    AP_SUBGROUPINFO(filters, "FILT", 26, AP_Vehicle, AP_Filters),
#endif

#if AP_STATS_ENABLED
    // @Group: STAT
    // @Path: ../AP_Stats/AP_Stats.cpp
    AP_SUBGROUPINFO(stats, "STAT", 27, AP_Vehicle, AP_Stats),
#endif


#if HAL_LOGGING_ENABLED
    // @Group: LOG
    // @Path: ../AP_Logger/AP_Logger.cpp
    AP_SUBGROUPINFO(logger, "LOG",  29, AP_Vehicle, AP_Logger),
#endif


#if AP_SERIALMANAGER_ENABLED
    // @Group: SERIAL
    // @Path: ../AP_SerialManager/AP_SerialManager.cpp
    AP_SUBGROUPINFO(serial_manager, "SERIAL", 31, AP_Vehicle, AP_SerialManager),
#endif

    AP_GROUPEND
};

// reference to the vehicle. using AP::vehicle() here does not work on clang
#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN) || APM_BUILD_TYPE(APM_BUILD_AP_Periph)
AP_Vehicle& vehicle = *AP_Vehicle::get_singleton();
#else
extern AP_Vehicle& vehicle;
#endif

/*
  setup is called when the sketch starts
 */
void AP_Vehicle::setup()
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

#if AP_SERIALMANAGER_ENABLED
    // initialise serial port
    serial_manager.init_console();
#endif

    DEV_PRINTF("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

#if AP_CHECK_FIRMWARE_ENABLED
    check_firmware_print();
#endif

    // validate the static parameter table, then load persistent
    // values from storage:
    AP_Param::check_var_info();
    load_parameters();

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if (AP_BoardConfig::get_sdcard_slowdown() != 0) {
        // user wants the SDcard slower, we need to remount
        sdcard_stop();
        sdcard_retry();
    }
#endif

#if AP_SCHEDULER_ENABLED
    // initialise the main loop scheduler
    const AP_Scheduler::Task *tasks;
    uint8_t task_count;
    uint32_t log_bit;
    get_scheduler_tasks(tasks, task_count, log_bit);
    AP::scheduler().init(tasks, task_count, log_bit);

    // time per loop - this gets updated in the main loop() based on
    // actual loop rate
    G_Dt = scheduler.get_loop_period_s();
#endif

    // this is here for Plane; its failsafe_check method requires the
    // RC channels to be set as early as possible for maximum
    // survivability.
    set_control_channels();

#if HAL_GCS_ENABLED
    // initialise serial manager as early as sensible to get
    // diagnostic output during boot process.  We have to initialise
    // the GCS singleton first as it sets the global mavlink system ID
    // which may get used very early on.
    gcs().init();
#endif

#if AP_SERIALMANAGER_ENABLED
#if HAL_WITH_IO_MCU
    if (BoardConfig.io_enabled()) {
        serial_manager.set_protocol_and_baud(HAL_UART_IOMCU_IDX, AP_SerialManager::SerialProtocol_IOMCU, 0);
    }
#endif
    // initialise serial ports
    serial_manager.init();
#endif
#if HAL_GCS_ENABLED
    gcs().setup_console();
#endif


#if AP_NETWORKING_ENABLED
    networking.init();
#endif

#if AP_SCHEDULER_ENABLED
    // Register scheduler_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(scheduler_delay_callback, 5);
#endif


#if AP_STATS_ENABLED
    // initialise stats module
    stats.init();
#endif

    BoardConfig.init();


#if HAL_LOGGING_ENABLED
    logger.init(get_log_bitmask(), get_log_structures(), get_num_log_structures());
#endif


    // init_ardupilot is where the vehicle does most of its initialisation.
    init_ardupilot();


#if AP_SRV_CHANNELS_ENABLED
    AP::srv().init();
#endif

#if AP_PARAM_KEY_DUMP
    AP_Param::show_all(hal.console, true);
#endif

    send_watchdog_reset_statustext();


// init EFI monitoring
#if HAL_EFI_ENABLED
    efi.init();
#endif

#if AP_FILTER_ENABLED
    filters.init();
#endif



    // invalidate count in case an enable parameter changed during
    // initialisation
    AP_Param::invalidate_count();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ArduPilot Ready");

#if AP_DDS_ENABLED
    if (!init_dds_client()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Failed to Initialize", AP_DDS_Client::msg_prefix);
    }
#endif

#if AP_IBUS_TELEM_ENABLED
    ibus_telem.init();
#endif
}

void AP_Vehicle::loop()
{
#if AP_SCHEDULER_ENABLED
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();
#else
    hal.scheduler->delay(1);
    G_Dt = 0.001;
#endif

    if (!done_safety_init) {
        /*
          disable safety if requested. This is delayed till after the
          first loop has run to ensure that all servos have received
          an update for their initial values. Otherwise we may end up
          briefly driving a servo to a position out of the configured
          range which could damage hardware
        */
        done_safety_init = true;
        BoardConfig.init_safety();

        // send RC output mode info if available
        char banner_msg[50];
        if (hal.rcout->get_output_mode_banner(banner_msg, sizeof(banner_msg))) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", banner_msg);
        }
    }
    const uint32_t new_internal_errors = AP::internalerror().errors();
    if(_last_internal_errors != new_internal_errors) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::INTERNAL_ERROR, LogErrorCode::INTERNAL_ERRORS_DETECTED);
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal Errors 0x%x", (unsigned)new_internal_errors);
        _last_internal_errors = new_internal_errors;
    }
}

#if AP_SCHEDULER_ENABLED
/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table presnet in each of the
  vehicles to determine the order in which tasks are run.  Convenience
  methods SCHED_TASK and SCHED_TASK_CLASS are provided to build
  entries in this structure:

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
const AP_Scheduler::Task AP_Vehicle::scheduler_tasks[] = {

#if COMPASS_CAL_ENABLED
    SCHED_TASK_CLASS(Compass,      &vehicle.compass,        cal_update,     100, 200, 75),
#endif
    SCHED_TASK_CLASS(AP_Notify,    &vehicle.notify,         update,                   50, 300, 78),

    SCHED_TASK(send_watchdog_reset_statustext,         0.1,     20, 225),

#if AP_NETWORKING_ENABLED
    SCHED_TASK_CLASS(AP_Networking, &vehicle.networking,    update,                   10,  50, 238),
#endif

#if OSD_ENABLED
    SCHED_TASK(publish_osd_info, 1, 10, 240),
#endif

#if HAL_EFI_ENABLED
    SCHED_TASK_CLASS(AP_EFI,       &vehicle.efi,            update,                   50, 200, 250),
#endif
    SCHED_TASK(one_Hz_update,                                                         1, 100, 252),

#if AP_FILTER_ENABLED
    SCHED_TASK_CLASS(AP_Filters,   &vehicle.filters,        update,                   1, 100, 252),
#endif
#if AP_STATS_ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &vehicle.stats,            update,           1, 100, 252),
#endif
#if AP_ARMING_ENABLED
    SCHED_TASK(update_arming,          1,     50, 253),
#endif
};

void AP_Vehicle::get_common_scheduler_tasks(const AP_Scheduler::Task*& tasks, uint8_t& num_tasks)
{
    tasks = scheduler_tasks;
    num_tasks = ARRAY_SIZE(scheduler_tasks);
}

/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
void AP_Vehicle::scheduler_delay_callback()
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    // compass.init() delays, so we end up here.
    return;
#endif

    static uint32_t last_1hz, last_50hz, last_5s;

#if HAL_LOGGING_ENABLED
    AP_Logger &logger = AP::logger();

    // don't allow potentially expensive logging calls:
    logger.EnableWrites(false);
#endif

    const uint32_t tnow = AP_HAL::millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        GCS_SEND_MESSAGE(MSG_HEARTBEAT);
        GCS_SEND_MESSAGE(MSG_SYS_STATUS);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
#if HAL_GCS_ENABLED
        gcs().update_receive();
        gcs().update_send();
#endif
        _singleton->notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        if (AP_BoardConfig::in_config_error()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Config Error: fix problem then reboot");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Initialising ArduPilot");
        }
    }

#if HAL_LOGGING_ENABLED
    logger.EnableWrites(true);
#endif
}
#endif  // AP_SCHEDULER_ENABLED

// if there's been a watchdog reset, notify the world via a statustext:
void AP_Vehicle::send_watchdog_reset_statustext()
{
    if (!hal.util->was_watchdog_reset()) {
        return;
    }
    const AP_HAL::Util::PersistentData &pd = hal.util->last_persistent_data;
    (void)pd;  // in case !HAL_GCS_ENABLED
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                    "WDG: T%d SL%u FL%u FT%u FA%x FTP%u FLR%x FICSR%u MM%u MC%u IE%u IEC%u TN:%.4s",
                    pd.scheduler_task,
                    pd.semaphore_line,
                    pd.fault_line,
                    pd.fault_type,
                    (unsigned)pd.fault_addr,
                    pd.fault_thd_prio,
                    (unsigned)pd.fault_lr,
                    (unsigned)pd.fault_icsr,
                    pd.last_mavlink_msgid,
                    pd.last_mavlink_cmd,
                    (unsigned)pd.internal_errors,
                    (unsigned)pd.internal_error_count,
                    pd.thread_name4
        );
}

bool AP_Vehicle::is_crashed() const
{
#if AP_ARMING_ENABLED
    if (AP::arming().is_armed()) {
        return false;
    }
    return AP::arming().last_disarm_method() == AP_Arming::Method::CRASH;
#else
    return false;
#endif
}


void AP_Vehicle::notify_no_such_mode(uint8_t mode_number)
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"No such mode %u", mode_number);
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode_number));
}

// reboot the vehicle in an orderly manner, doing various cleanups and
// flashing LEDs as appropriate
void AP_Vehicle::reboot(bool hold_in_bootloader)
{
    if (should_zero_rc_outputs_on_reboot()) {
        SRV_Channels::zero_rc_outputs();
    }

    // Notify might want to blink some LEDs:
    AP_Notify::flags.firmware_update = 1;
    notify.update();

    // force safety on
    hal.rcout->force_safety_on();

    // flush pending parameter writes
    AP_Param::flush();

    // do not process incoming mavlink messages while we delay:
    hal.scheduler->register_delay_callback(nullptr, 5);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // need to ensure the ack goes out:
    hal.serial(0)->flush();
#endif

    // delay to give the ACK a chance to get out, the LEDs to flash,
    // the IO board safety to be forced on, the parameters to flush, ...
    hal.scheduler->delay(200);

#if HAL_WITH_IO_MCU
    iomcu.soft_reboot();
#endif

    hal.scheduler->reboot(hold_in_bootloader);
}


#if OSD_ENABLED
void AP_Vehicle::publish_osd_info()
{
}
#endif

void AP_Vehicle::get_osd_roll_pitch_rad(float &roll, float &pitch) const
{
    roll = 0.0;
    pitch = 0.0;
}


#if AP_ARMING_ENABLED
// call the arming library's update function
void AP_Vehicle::update_arming()
{
    AP::arming().update();
}
#endif

/*
  one Hz checks common to all vehicles
 */
void AP_Vehicle::one_Hz_update(void)
{
    one_Hz_counter++;

    /*
      every 10s check if using a 2M firmware on a 1M board
     */
    if (one_Hz_counter % 10U == 0) {
#if defined(BOARD_CHECK_F427_USE_1M) && (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
        if (!hal.util->get_soft_armed() && check_limit_flash_1M()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, BOARD_CHECK_F427_USE_1M);
        }
#endif
    }

    /*
      every 30s check if using a 1M firmware on a 2M board
     */
    if (one_Hz_counter % 30U == 0) {
#if defined(BOARD_CHECK_F427_USE_1M) && (HAL_PROGRAM_SIZE_LIMIT_KB<=1024)
        if (!hal.util->get_soft_armed() && !check_limit_flash_1M()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, BOARD_CHECK_F427_USE_2M);
        }
#endif
    }


#if HAL_LOGGING_ENABLED && HAL_UART_STATS_ENABLED
    // Log data rates of physical and virtual serial ports
    hal.util->uart_log();
#if AP_SERIALMANAGER_REGISTER_ENABLED
    serial_manager.registered_ports_log();
#endif
#endif

}

void AP_Vehicle::check_motor_noise()
{

}

#if AP_DDS_ENABLED
bool AP_Vehicle::init_dds_client()
{
    dds_client = NEW_NOTHROW AP_DDS_Client();
    if (dds_client == nullptr) {
        return false;
    }
    return dds_client->start();
}
#endif // AP_DDS_ENABLED

// Check if this mode can be entered from the GCS
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_Rover)
bool AP_Vehicle::block_GCS_mode_change(uint8_t mode_num, const uint8_t *mode_list, uint8_t mode_list_length) const
{
    if (mode_list == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < mode_list_length; i++) {
        // Find index of mode
        if (mode_list[i] == mode_num) {
            const uint32_t mask = 1U << i;
            return (uint32_t(flight_mode_GCS_block) & mask) != 0;
        }
    }

    return false;
}
#endif


AP_Vehicle *AP_Vehicle::_singleton = nullptr;

AP_Vehicle *AP_Vehicle::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_Vehicle *vehicle()
{
    return AP_Vehicle::get_singleton();
}

};

#endif  // AP_VEHICLE_ENABLED
