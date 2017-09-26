#include "Rover.h"

// initialise compass
void Rover::init_compass()
{
    if (!g.compass_enabled) {
        return;
    }

    if (!compass.init()|| !compass.read()) {
        cliSerial->printf("Compass initialisation failed!\n");
        g.compass_enabled = false;
    } else {
        ahrs.set_compass(&compass);
    }
}

/*
  if the compass is enabled then try to accumulate a reading
  also update initial location used for declination
 */
void Rover::compass_accumulate(void)
{
    if (!g.compass_enabled) {
        return;
    }

    compass.accumulate();

    // update initial location used for declination
    if (!compass_init_location) {
        Location loc;
        if (ahrs.get_position(loc)) {
            compass.set_initial_location(loc.lat, loc.lng);
            compass_init_location = true;
        }
    }
}

void Rover::init_barometer(bool full_calibration)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    } else {
        barometer.update_calibration();
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

void Rover::init_rangefinder(void)
{
    rangefinder.init();
}

// init beacons used for non-gps position estimates
void Rover::init_beacon()
{
    g2.beacon.init();
}

// update beacons
void Rover::update_beacon()
{
    g2.beacon.update();
}

// init visual odometry sensor
void Rover::init_visual_odom()
{
    g2.visual_odom.init();
}

// update visual odometry sensor
void Rover::update_visual_odom()
{
    // check for updates
    if (g2.visual_odom.enabled() && (g2.visual_odom.get_last_update_ms() != visual_odom_last_update_ms)) {
        visual_odom_last_update_ms = g2.visual_odom.get_last_update_ms();
        const float time_delta_sec = g2.visual_odom.get_time_delta_usec() / 1000000.0f;
        ahrs.writeBodyFrameOdom(g2.visual_odom.get_confidence(),
                                g2.visual_odom.get_position_delta(),
                                g2.visual_odom.get_angle_delta(),
                                time_delta_sec,
                                visual_odom_last_update_ms,
                                g2.visual_odom.get_pos_offset());
        // log sensor data
        DataFlash.Log_Write_VisualOdom(time_delta_sec,
                                       g2.visual_odom.get_angle_delta(),
                                       g2.visual_odom.get_position_delta(),
                                       g2.visual_odom.get_confidence());
    }
}

// update wheel encoders
void Rover::update_wheel_encoder()
{
    g2.wheel_encoder.update();
}

// read_battery - reads battery voltage and current and invokes failsafe
// should be called at 10hz
void Rover::read_battery(void)
{
    battery.read();
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Rover::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

// Calibrate compass
void Rover::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

// Accel calibration

void Rover::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them    float trim_roll, trim_pitch;
    float trim_roll, trim_pitch;
    if (ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}
void Rover::read_rangefinders(void)
{
    uint32_t crnt_snr_tm;
    rangefinder.update();
    if (rangefinder.status(0) == RangeFinder::RangeFinder_NotConnected) 
    {
        // this makes it possible to disable rangefinder at runtime
	obstacle.turning = false;
        return;
    }

    obstacle.rangefinder1_distance_cm = rangefinder.distance_cm(0);
    crnt_snr_tm = AP_HAL::millis();
    if (obstacle.rangefinder1_distance_cm < static_cast<uint16_t>(g.rangefinder_trigger_cm) && 
        obstacle.rangefinder1_distance_cm > 15)  
    {
        if (obstacle.detected_count < g.rangefinder_debounce) obstacle.detected_count++;
        else if (obstacle.detected_count == g.rangefinder_debounce) 
        {
            if(obstacle.turning == false) gcs().send_text(MAV_SEVERITY_INFO, "LIDAR: AVOIDANCE TRIGGERED @ %u ms", 
                static_cast<uint32_t>(crnt_snr_tm));
            obstacle.detected_time_ms = crnt_snr_tm;
            obstacle.detected_count = 0;
            if(turning_right) obstacle.turn_angle = g.rangefinder_turn_angle;
            else obstacle.turn_angle = -g.rangefinder_turn_angle;
            obstacle.turning = true;
        }

    }
    else if(obstacle.turning == false) obstacle.detected_count = 0;

    Log_Write_Rangefinder();   
    if(obstacle.turning == true)
    {
        if (crnt_snr_tm > (obstacle.detected_time_ms + g.rangefinder_turn_time*1000)) 
        {
            obstacle.turn_angle = 0;
            obstacle.detected_count = 0;
            obstacle.turning = false;
            obstacle.detected_time_ms = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "LIDAR: OBSTACLE CLEARED @ %u ms", static_cast<uint32_t>(crnt_snr_tm));
        }
        else if(crnt_snr_tm > obstacle.detected_time_ms + 4000 || crnt_snr_tm < obstacle.detected_time_ms)
        {
            obstacle.turn_angle = 0;
            obstacle.detected_count = 0;
            obstacle.turning = false;
            obstacle.detected_time_ms = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "LIDAR: AVOIDANCE TIMED OUT");
        }
    }
}

/*
  update AP_Button
 */
void Rover::button_update(void)
{
    button.update();
}

// update error mask of sensors and subsystems. The mask
// uses the MAV_SYS_STATUS_* values from mavlink. If a bit is set
// then it indicates that the sensor or subsystem is present but
// not functioning correctly.
void Rover::update_sensor_status_flags(void)
{
    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;  // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (g2.visual_odom.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    if (rover.DataFlash.logging_present()) {  // primary logging only (usually File)
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }


    // all present sensors enabled by default except rate control, attitude stabilization, yaw, altitude, position control and motor output which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION &
                                                         ~MAV_SYS_STATUS_SENSOR_YAW_POSITION &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS &
                                                         ~MAV_SYS_STATUS_LOGGING);

    switch (control_mode) {
    case MANUAL:
    case HOLD:
        break;

    case LEARNING:
    case STEERING:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;    // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;  // attitude stabilisation
        break;

    case AUTO:
    case RTL:
    case GUIDED:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;    // 3D angular rate control
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;  // attitude stabilisation
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;            // yaw position
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;     // X/Y position control
        break;

    case INITIALISING:
        break;
    }

    if (rover.DataFlash.logging_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default to all healthy except compass and gps which we set individually
    control_sensors_health = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_3D_MAG & ~MAV_SYS_STATUS_SENSOR_GPS);
    if (g.compass_enabled && compass.healthy(0) && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (g2.visual_odom.enabled() && !g2.visual_odom.healthy()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    if (rangefinder.num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (g.rangefinder_trigger_cm > 0) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
        if (rangefinder.has_data(0)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }

    if (rover.DataFlash.logging_failed()) {
        control_sensors_health &= ~MAV_SYS_STATUS_LOGGING;
    }

    if (AP_Notify::flags.initialising) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }
#if FRSKY_TELEM_ENABLED == ENABLED
    // give mask of error flags to Frsky_Telemetry
    frsky_telemetry.update_sensor_status_flags(~control_sensors_health & control_sensors_enabled & control_sensors_present);
#endif
}
