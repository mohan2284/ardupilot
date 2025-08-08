#include "AP_Camera_config.h"

#if AP_CAMERA_ENABLED

#include "AP_Camera_Backend.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_AHRS/AP_AHRS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

// Constructor
AP_Camera_Backend::AP_Camera_Backend(AP_Camera &frontend, AP_Camera_Params &params, uint8_t instance) :
    _frontend(frontend),
    _params(params),
    _instance(instance)
{}

void AP_Camera_Backend::init()
{
#if AP_CAMERA_INFO_FROM_SCRIPT_ENABLED
    _camera_info.focal_length = NaNf;
    _camera_info.sensor_size_h = NaNf;
    _camera_info.sensor_size_v = NaNf;

    _camera_info.flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
#endif // AP_CAMERA_INFO_FROM_SCRIPT_ENABLED
}

// update - should be called at 50hz
void AP_Camera_Backend::update()
{
    // Check camera options and start/stop recording based on arm/disarm
    if (option_is_enabled(Option::RecordWhileArmed)) {
        if (hal.util->get_soft_armed() != last_is_armed) {
            last_is_armed = hal.util->get_soft_armed();
            if (!record_video(last_is_armed)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Camera: failed to %s recording", last_is_armed ? "start" : "stop");
            }
        }
    }

    const bool using_feedback_pin = _params.feedback_pin > 0;

    if (using_feedback_pin && !fb_high) {
        // ensure we are in input mode
        hal.gpio->pinMode(_params.feedback_pin, HAL_GPIO_INPUT);
        // enable pullup/pulldown
        uint8_t trigger_polarity = _params.feedback_polarity == 0 ? 0 : 1;
        hal.gpio->write(_params.feedback_pin, !trigger_polarity);
        fb_high = true;
    }

    // try to take picture if pending
    if (trigger_pending) {
        take_picture();
    }

    // check feedback pin
    check_feedback();

    if( ((_params._cam1_bootup_trig_time * 1000) > 0) && !abort_camInit)
    {
        //do the camera bootup trigger
        if(!trig_init_done)
        {
            camTrig_init();
        }
    }

    // time based triggering
    // if time and distance triggering both are enabled then we only do time based triggering
    if (time_interval_settings.num_remaining != 0) {
        uint32_t delta_ms = AP_HAL::millis() - last_picture_time_ms;
        if (delta_ms > time_interval_settings.time_interval_ms) {
            if (take_picture()) {
                // decrease num_remaining except when its -1 i.e. capture forever
                if (time_interval_settings.num_remaining > 0) {
                    time_interval_settings.num_remaining--;
                }
            }
        }
        return;
    }

    // implement trigger distance
    if (!is_positive(_params.trigg_dist)) {
        last_location.lat = 0;
        last_location.lng = 0;
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    Location current_loc;
    if (!ahrs.get_location(current_loc)) {
        return;
    }

    // check vehicle flight mode supports trigg dist
    if (!_frontend.vehicle_mode_ok_for_trigg_dist()) {
        return;
    }

    // check vehicle roll angle is less than configured maximum
    if ((_frontend.get_roll_max() > 0) && (fabsf(ahrs.roll_sensor * 1e-2f) > _frontend.get_roll_max())) {
        return;
    }

    // initialise last location to current location
    if (last_location.lat == 0 && last_location.lng == 0) {
        last_location = current_loc;
        return;
    }
    if (last_location.lat == current_loc.lat && last_location.lng == current_loc.lng) {
        // we haven't moved - this can happen as update() may
        // be called without a new GPS fix
        return;
    }

    // check vehicle has moved at least trigg_dist meters
    if (current_loc.get_distance(last_location) < _params.trigg_dist) {
        return;
    }

    take_picture();
}

// get corresponding mount instance for the camera
uint8_t AP_Camera_Backend::get_mount_instance() const
{
    // instance 0 means default
    if (_params.mount_instance.get() == 0) {
        return _instance;
    }
    return _params.mount_instance.get() - 1;
}

// get mavlink gimbal device id which is normally mount_instance+1
uint8_t AP_Camera_Backend::get_gimbal_device_id() const
{
#if HAL_MOUNT_ENABLED
    const uint8_t mount_instance = get_mount_instance();
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        if (mount->get_mount_type(mount_instance) != AP_Mount::Type::None) {
            return (mount_instance + 1);
        }
    }
#endif
    return 0;
}


// take a picture.  returns true on success
bool AP_Camera_Backend::take_picture()
{
    // setup feedback pin interrupt or timer
    setup_feedback_callback();

    // check minimum time interval since last picture taken
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_picture_time_ms < (uint32_t)(_params.interval_min * 1000)) {
        trigger_pending = true;
        return false;
    }

    trigger_pending = false;

    // trigger actually taking picture and update image count
    if (trigger_pic()) {
        camTrig = true;
        image_index++;
        last_picture_time_ms = now_ms;
        IGNORE_RETURN(AP::ahrs().get_location(last_location));
#if HAL_LOGGING_ENABLED
        log_picture();
#endif
        return true;
    }

    return false;
}

/*Start: Asteria Code Change*/
bool AP_Camera_Backend::camTrig_init(){
    if(!trig_init_start){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Please wait while geotagging is being sync!");
        trig_init_start = true;
    }

    static const uint32_t tstart = AP_HAL::millis();
    static uint8_t count = 0;

    if(!first_trig && ((AP_HAL::millis() - tstart) > (uint32_t)(_params._cam1_bootup_trig_time * 1000)))
    {
        if(feedback_rcvd && trig_pic){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM1 INIT TRIG-1 completed");
            first_trig = true;
            feedback_rcvd = false;
            count = 0;
            trig_pic = false;
            trig_fail = false;
            return true;
        }

        if(trig_pic && ((AP_HAL::millis() - last_trig) < ((_params.trigger_duration * 1000)+500)))
        {
            return false;
        }

        if(!feedback_rcvd && trig_pic && !trig_fail)
        {
            count++;
            trig_fail = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRIG-1 attempt-%d feedback, failed", count);
            if(count > 2){
                _params.trigger_duration.set_and_save(_frontend.trigger_duration_copy);
                abort_camInit = true;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM1 INIT TRIG-1 failed");
                return false;
            }
        } 

        if(take_picture())
        {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRIG-1 attempt-%d", count+1);
            trig_pic = true;
            trig_fail = false;
            last_trig = AP_HAL::millis();
            last_tstart = AP_HAL::millis();
            return true;
        }
    }
    else if(!trig_init_done && first_trig && ((AP_HAL::millis() - last_tstart) > 5000))
    {
        if(feedback_rcvd && trig_pic){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM1 INIT TRIG-2 completed");
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Ready for Mapping Missions");
            _params.trigger_duration.set_and_save(_frontend.trigger_duration_copy);
            trig_init_done = true;
            feedback_rcvd = false;
            count = 0;
            last_tstart = 0;
            trig_pic = false;
            trig_fail = false;
            return true;
        }
        

        if(trig_pic && ((AP_HAL::millis() - last_trig) < ((_params.trigger_duration * 1000)+500)))
        {
            return false;
        }

        if(!feedback_rcvd && trig_pic && !trig_fail)
        {
            count++;
            trig_fail = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRIG-2 attempt-%d feedback, failed", count);
            if(count > 2){
                _params.trigger_duration.set_and_save(_frontend.trigger_duration_copy);
                abort_camInit = true;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM1 INIT TRIG-2 failed");
                return false;
            }
        }

        if(take_picture()){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TRIG-2 attempt-%d", count+1);
            trig_pic = true;
            trig_fail = false;
            last_trig = AP_HAL::millis();
            return true;
        }
    }
    return false;
}
/*End: Asteria Code Change*/

// take multiple pictures, time_interval between two consecutive pictures is in miliseconds
// total_num is number of pictures to be taken, -1 means capture forever
void AP_Camera_Backend::take_multiple_pictures(uint32_t time_interval_ms, int16_t total_num)
{
    time_interval_settings = {time_interval_ms, total_num};
}

// stop capturing multiple image sequence
void AP_Camera_Backend::stop_capture()
{
    time_interval_settings = {0, 0};
}

// handle camera control
void AP_Camera_Backend::control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id)
{
    // take picture
    if (shooting_cmd == 1) {
        take_picture();
    }
}

// send camera feedback message to GCS
void AP_Camera_Backend::send_camera_feedback(mavlink_channel_t chan)
{
    int32_t altitude = 0;
    if (camera_feedback.location.initialised() && !camera_feedback.location.get_alt_cm(Location::AltFrame::ABSOLUTE, altitude)) {
        // completely ignore this failure!  this is a shouldn't-happen
        // as current_loc should never be in an altitude we can't
        // convert.
    }
    int32_t altitude_rel = 0;
    if (camera_feedback.location.initialised() && !camera_feedback.location.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_rel)) {
        // completely ignore this failure!  this is a shouldn't-happen
        // as current_loc should never be in an altitude we can't
        // convert.
    }

    // send camera feedback message
    mavlink_msg_camera_feedback_send(
        chan,
        camera_feedback.timestamp_us,       // image timestamp
        0,                                  // target system id
        _instance,                          // camera id
        image_index,                        // image index
        camera_feedback.location.lat,       // latitude
        camera_feedback.location.lng,       // longitude
        altitude*1e-2f,                     // alt MSL
        altitude_rel*1e-2f,                 // alt relative to home
        camera_feedback.roll_sensor*1e-2f,  // roll angle (deg)
        camera_feedback.pitch_sensor*1e-2f, // pitch angle (deg)
        camera_feedback.yaw_sensor*1e-2f,   // yaw angle (deg)
        0.0f,                               // focal length
        CAMERA_FEEDBACK_PHOTO,              // flags
        camera_feedback.feedback_trigger_logged_count); // completed image captures
}

// send camera information message to GCS
void AP_Camera_Backend::send_camera_information(mavlink_channel_t chan) const
{
    mavlink_camera_information_t camera_info;
#if AP_CAMERA_INFO_FROM_SCRIPT_ENABLED

    camera_info = _camera_info;

#else

    memset(&camera_info, 0, sizeof(camera_info));

    camera_info.focal_length = NaNf;
    camera_info.sensor_size_h = NaNf;
    camera_info.sensor_size_v = NaNf;

    camera_info.flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE;

#endif // AP_CAMERA_INFO_FROM_SCRIPT_ENABLED

    // Set fixed fields
    // lens_id is populated with the instance number, to disambiguate multiple cameras
    camera_info.lens_id = _instance;
    camera_info.gimbal_device_id = get_gimbal_device_id();
    camera_info.time_boot_ms = AP_HAL::millis();

    // Send CAMERA_INFORMATION message
    mavlink_msg_camera_information_send_struct(chan, &camera_info);
}

#if AP_CAMERA_INFO_FROM_SCRIPT_ENABLED
void AP_Camera_Backend::set_camera_information(mavlink_camera_information_t camera_info)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Camera %u CAMERA_INFORMATION (%s %s) set from script", _instance, camera_info.vendor_name, camera_info.model_name);
    _camera_info = camera_info;
};
#endif // AP_CAMERA_INFO_FROM_SCRIPT_ENABLED

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
// send video stream information message to GCS
void AP_Camera_Backend::send_video_stream_information(mavlink_channel_t chan) const
{
#if AP_CAMERA_INFO_FROM_SCRIPT_ENABLED

    // Send VIDEO_STREAM_INFORMATION message
    mavlink_msg_video_stream_information_send_struct(chan, &_stream_info);

#endif // AP_CAMERA_INFO_FROM_SCRIPT_ENABLED
}
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED

#if AP_CAMERA_INFO_FROM_SCRIPT_ENABLED
void AP_Camera_Backend::set_stream_information(mavlink_video_stream_information_t stream_info)
{
    _stream_info = stream_info;
};
#endif // AP_CAMERA_INFO_FROM_SCRIPT_ENABLED

// send camera settings message to GCS
void AP_Camera_Backend::send_camera_settings(mavlink_channel_t chan) const
{
    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        CAMERA_MODE_IMAGE,  // camera mode (0:image, 1:video, 2:image survey)
        NaNf,               // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaNf);              // focusLevel float, percentage from 0 to 100, NaN if unknown
}

#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
// send camera field of view status
void AP_Camera_Backend::send_camera_fov_status(mavlink_channel_t chan) const
{
    // getting corresponding mount instance for camera
    AP_Mount* mount = AP::mount();
    if (mount == nullptr) {
        return;
    }

    // get latest POI from mount
    Quaternion quat;
    Location camera_loc;
    Location poi_loc;
    const bool have_poi_loc = mount->get_poi(get_mount_instance(), quat, camera_loc, poi_loc);

    // if failed to get POI, get camera location directly from AHRS
    // and attitude directly from mount
    bool have_camera_loc = have_poi_loc;
    if (!have_camera_loc) {
        have_camera_loc = AP::ahrs().get_location(camera_loc);
        mount->get_attitude_quaternion(get_mount_instance(), quat);
    }

    // calculate attitude quaternion in earth frame using AHRS yaw
    Quaternion quat_ef;
    quat_ef.from_euler(0, 0, AP::ahrs().get_yaw());
    quat_ef *= quat;

    // send camera fov status message only if the last calculated values aren't stale
    const float quat_array[4] = {
        quat_ef.q1,
        quat_ef.q2,
        quat_ef.q3,
        quat_ef.q4
    };
    mavlink_msg_camera_fov_status_send(
        chan,
        AP_HAL::millis(),
        have_camera_loc ? camera_loc.lat : INT32_MAX,
        have_camera_loc ? camera_loc.lng : INT32_MAX,
        have_camera_loc ? camera_loc.alt * 10 : INT32_MAX,
        have_poi_loc ? poi_loc.lat : INT32_MAX,
        have_poi_loc ? poi_loc.lng : INT32_MAX,
        have_poi_loc ? poi_loc.alt * 10 : INT32_MAX,
        quat_array,
        horizontal_fov() > 0 ? horizontal_fov() : NaNf,
        vertical_fov() > 0 ? vertical_fov() : NaNf
    );
}
#endif

// send camera capture status message to GCS
void AP_Camera_Backend::send_camera_capture_status(mavlink_channel_t chan) const
{
    // Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
    const uint8_t image_status = (time_interval_settings.num_remaining > 0) ? 2 : 0;

    // send CAMERA_CAPTURE_STATUS message
    mavlink_msg_camera_capture_status_send(
        chan,
        AP_HAL::millis(),
        image_status,
        0,                // current status of video capturing (0: idle, 1: capture in progress)
        static_cast<float>(time_interval_settings.time_interval_ms) / 1000.0, // image capture interval (s)
        0,                // elapsed time since recording started (ms)
        NaNf,             // available storage capacity (ms)
        image_index);     // total number of images captured
}

// setup a callback for a feedback pin. When on PX4 with the right FMU
// mode we can use the microsecond timer.
void AP_Camera_Backend::setup_feedback_callback()
{
    if (_params.feedback_pin <= 0 || timer_installed || isr_installed) {
        // invalid or already installed
        return;
    }

    // ensure we are in input mode
    hal.gpio->pinMode(_params.feedback_pin, HAL_GPIO_INPUT);

    // enable pullup/pulldown
    uint8_t trigger_polarity = _params.feedback_polarity == 0 ? 0 : 1;
    hal.gpio->write(_params.feedback_pin, !trigger_polarity);

    /* if (hal.gpio->attach_interrupt(_params.feedback_pin, FUNCTOR_BIND_MEMBER(&AP_Camera_Backend::feedback_pin_isr, void, uint8_t, bool, uint32_t),
                                   trigger_polarity?AP_HAL::GPIO::INTERRUPT_RISING:AP_HAL::GPIO::INTERRUPT_FALLING)) {
        isr_installed = true;
    } else { */
        // install a 1kHz timer to check feedback pin
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Camera_Backend::feedback_pin_timer, void));

        timer_installed = true;
    //}
}

// interrupt handler for interrupt based feedback trigger
void AP_Camera_Backend::feedback_pin_isr(uint8_t pin, bool high, uint32_t timestamp_us)
{
    feedback_trigger_timestamp_us = timestamp_us;
    feedback_trigger_count++;
}

// check if feedback pin is high for timer based feedback trigger, when
// attach_interrupt fails
void AP_Camera_Backend::feedback_pin_timer()
{
    uint8_t pin_state = hal.gpio->read(_params.feedback_pin);
    uint8_t trigger_polarity = _params.feedback_polarity == 0 ? 0 : 1;
    if (pin_state == trigger_polarity &&
        last_pin_state != trigger_polarity && camTrig) {
        feedback_trigger_timestamp_us = AP_HAL::micros();
        feedback_trigger_count++;
        feedback_rcvd = true;
        camTrig = false;
    }
    last_pin_state = pin_state;
}

// check for feedback pin update and log if necessary
void AP_Camera_Backend::check_feedback()
{
    if (feedback_trigger_logged_count != feedback_trigger_count) {
#if HAL_LOGGING_ENABLED
        const uint32_t timestamp32 = feedback_trigger_timestamp_us;
#endif
        feedback_trigger_logged_count = feedback_trigger_count;

        // we should consider doing this inside the ISR and pin_timer
        prep_mavlink_msg_camera_feedback(feedback_trigger_timestamp_us);

#if HAL_LOGGING_ENABLED
        // log camera message
        uint32_t tdiff = AP_HAL::micros() - timestamp32;
        uint64_t timestamp = AP_HAL::micros64();
        Write_Camera(timestamp - tdiff);
#endif
    }
}

void AP_Camera_Backend::prep_mavlink_msg_camera_feedback(uint64_t timestamp_us)
{
    const AP_AHRS &ahrs = AP::ahrs();
    if (!ahrs.get_location(camera_feedback.location)) {
        // completely ignore this failure!  AHRS will provide its best guess.
    }
    camera_feedback.timestamp_us = timestamp_us;
    camera_feedback.roll_sensor = ahrs.roll_sensor;
    camera_feedback.pitch_sensor = ahrs.pitch_sensor;
    camera_feedback.yaw_sensor = ahrs.yaw_sensor;
    camera_feedback.feedback_trigger_logged_count = feedback_trigger_logged_count;

    GCS_SEND_MESSAGE(MSG_CAMERA_FEEDBACK);
    GCS_SEND_MESSAGE(MSG_CAMERA_TRIGGER_ORG);
}

#if HAL_LOGGING_ENABLED
// log picture
void AP_Camera_Backend::log_picture()
{
    const bool using_feedback_pin = _params.feedback_pin > 0;

    if (!using_feedback_pin) {
        // if we're using a feedback pin then when the event occurs we
        // stash the feedback data.  Since we're not using a feedback
        // pin, we just use "now".
        prep_mavlink_msg_camera_feedback(AP::gps().time_epoch_usec());
    }

    if (!using_feedback_pin) {
        Write_Camera();
        Write_Camera_org();
    } else {
        Write_Trigger();
        Write_Trigger_org();
    }
}
#endif

void AP_Camera_Backend::send_camera_trigger_org(mavlink_channel_t chan) const
{
    const AP_AHRS &ahrs = AP::ahrs();
    const AP_GPS &gps = AP::gps();
    const Location &loc = gps.location(0);
    int32_t height_ellipsoid_mm=0;
    float undulation=0.0;
    int32_t altitude_rel = 0;
    if (loc.initialised() && !loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_rel)) {
        // completely ignore this failure!  this is a shouldn't-happen
        // as current_loc should never be in an altitude we can't
        // convert.
    }
    uint64_t number_of_week=gps.time_week(0);
    uint32_t week_elapsed_time=gps.time_week_ms(0);
    uint8_t fixType = (uint8_t)gps.status(0);
    if(gps.get_undulation(0,undulation)){
        height_ellipsoid_mm=loc.alt*10 - undulation*1000;
    }
    mavlink_msg_camera_trigger_org_send(
        chan,
        image_index,//image index
        camera_feedback.timestamp_us,//image timestamp
        loc.lat,       // latitude
        loc.lng,       // longitude
        altitude_rel*1e-2f,                 // alt relative to home
        wrap_180(ahrs.roll_sensor*1e-2f),  // roll angle (deg)
        wrap_180(ahrs.pitch_sensor*1e-2f), // pitch angle (deg)
        wrap_180(ahrs.yaw_sensor*1e-2f),   // yaw angle (deg)
        fixType,//GPS fix type
        height_ellipsoid_mm, //Ellipsoid height in mm
        ahrs.get_roll(),//Roll
        ahrs.get_pitch(),//Pitch
        ahrs.get_yaw(),//Yaw
        number_of_week,
        week_elapsed_time);

    char buf[250];
    const int16_t offset = snprintf(buf, sizeof(buf), "ImageCaptureTimestamp,ImgIndex,GPSLatitude,GPSLongitude,RelALT,CAMRoll,CAMPitch,CAMYaw,GPSFixType,GNSSAntennaALT,UASRoll,UASPitch,UASYaw,GPSWeek,GPSTime\r\n");
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Creating the header");
    snprintf(buf + offset, sizeof(buf), "%" PRIu64",%u,%0.7f,%0.7f,%0.2f,%0.2f,%0.2f,%0.2f,%u,%" PRId32 ",%0.2f,%0.2f,%0.2f,%" PRIu64 ",%" PRIu32 "\r\n", camera_feedback.timestamp_us, image_index, loc.lat * 1e-7f, loc.lng * 1e-7f, altitude_rel*1e-2f, wrap_180(ahrs.roll_sensor*1e-2f), wrap_180(ahrs.pitch_sensor*1e-2f), wrap_180(ahrs.yaw_sensor*1e-2f), fixType, height_ellipsoid_mm, ahrs.get_roll()*1e-2f, ahrs.get_pitch()*1e-2f, ahrs.get_yaw()*1e-2f, number_of_week, week_elapsed_time);
    AP::logger().WriteLogHeader(buf, strlen(buf));
     
    if(camera_feedback.timestamp_us){
        char data[130];
        snprintf(data, sizeof(data), "%" PRIu64 ",%u,%0.7f,%0.7f,%0.2f,%0.2f,%0.2f,%0.2f,%u,%" PRId32 ",%0.2f,%0.2f,%0.2f,%" PRIu64",%" PRIu32 "\r\n", camera_feedback.timestamp_us, image_index, loc.lat * 1e-7f, loc.lng * 1e-7f, altitude_rel*1e-2f, wrap_180(ahrs.roll_sensor*1e-2f), wrap_180(ahrs.pitch_sensor*1e-2f), wrap_180(ahrs.yaw_sensor*1e-2f), fixType, height_ellipsoid_mm, ahrs.get_roll()*1e-2f, ahrs.get_pitch()*1e-2f, ahrs.get_yaw()*1e-2f, number_of_week, week_elapsed_time);
        AP::logger().WriteCSVBlock(data, strlen(data));
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Camera trigger org feedback sent!");
}

#endif // AP_CAMERA_ENABLED
