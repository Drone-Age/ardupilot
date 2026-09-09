#include "AP_CargoImpact.h"

#if AP_CARGO_IMPACT_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Mount/AP_Mount.h>
#include <cstring>

extern const AP_HAL::HAL &hal;

AP_CargoImpact *AP_CargoImpact::_singleton;

const AP_Param::GroupInfo AP_CargoImpact::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Cargo impact assistant enable
    // @Description: Enables passive prediction of a released civil cargo item's ground impact point. This feature never operates a release mechanism or provides a navigation target.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_CargoImpact, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MODE
    // @DisplayName: Cargo impact prediction mode
    // @Description: Selects the prediction model. Simple ignores aerodynamic drag. DragWind uses mass, drag area and the AHRS wind estimate. External accepts an earth-fixed local NED point over the external interface.
    // @Values: 0:Disabled,1:Simple,2:DragWind,3:External
    // @User: Advanced
    AP_GROUPINFO("MODE", 2, AP_CargoImpact, _mode, static_cast<int8_t>(Mode::SIMPLE)),

    // @Param: MASS
    // @DisplayName: Cargo mass
    // @Description: Mass of the civil cargo test article used by the DragWind model
    // @Units: kg
    // @Range: 0.05 50
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("MASS", 3, AP_CargoImpact, _mass_kg, 2.0f),

    // @Param: CDA
    // @DisplayName: Cargo drag area
    // @Description: Combined drag coefficient multiplied by reference area for the cargo test article
    // @Units: m^2
    // @Range: 0 2
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("CDA", 4, AP_CargoImpact, _cda_m2, 0.03f),

    // @Param: DELAY
    // @DisplayName: Cargo separation delay
    // @Description: Estimated delay between the observed release event and physical separation. Constant vehicle velocity is assumed during this delay.
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("DELAY", 5, AP_CargoImpact, _release_delay_s, 0.0f),

    // @Param: RHO
    // @DisplayName: Air density
    // @Description: Air density used by the DragWind prediction model
    // @Units: kg/m^3
    // @Range: 0.5 1.5
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("RHO", 6, AP_CargoImpact, _air_density_kgm3, 1.225f),

    // @Param: EXT_TO
    // @DisplayName: External prediction timeout
    // @Description: Maximum age of an external local NED impact prediction before it is hidden
    // @Units: ms
    // @Range: 100 5000
    // @Increment: 50
    // @User: Advanced
    AP_GROUPINFO("EXT_TO", 7, AP_CargoImpact, _external_timeout_ms, 500),

    // @Param: CAMSEL
    // @DisplayName: Active video camera
    // @Description: Selects the camera calibration used for OSD projection
    // @Values: 0:Camera1,1:Camera2
    // @User: Advanced
    AP_GROUPINFO("CAMSEL", 8, AP_CargoImpact, _active_camera, 0),

    // @Param: C1_HFOV
    // @DisplayName: Camera 1 horizontal field of view
    // @Description: Fallback field of view used only when the standard CAM1_HFOV parameter is unset
    // @Units: deg
    // @Range: 5 179
    // @User: Advanced
    AP_GROUPINFO("C1_HFOV", 9, AP_CargoImpact, _camera1_hfov_deg, 65.0f),

    // @Param: C1_VFOV
    // @DisplayName: Camera 1 vertical field of view
    // @Description: Fallback field of view used only when the standard CAM1_VFOV parameter is unset
    // @Units: deg
    // @Range: 5 179
    // @User: Advanced
    AP_GROUPINFO("C1_VFOV", 10, AP_CargoImpact, _camera1_vfov_deg, 51.1f),

    // @Param: C1_ROLL
    // @DisplayName: Camera 1 body roll
    // @Description: Camera optical-frame roll relative to the vehicle body frame
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("C1_ROLL", 11, AP_CargoImpact, _camera1_roll_deg, 0.0f),

    // @Param: C1_PITCH
    // @DisplayName: Camera 1 body pitch
    // @Description: Camera optical-frame pitch relative to the vehicle body frame. Minus 90 degrees is nadir.
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("C1_PITCH", 12, AP_CargoImpact, _camera1_pitch_deg, -90.0f),

    // @Param: C1_YAW
    // @DisplayName: Camera 1 body yaw
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("C1_YAW", 13, AP_CargoImpact, _camera1_yaw_deg, 0.0f),

    // @Param: C2_HFOV
    // @DisplayName: Camera 2 horizontal field of view
    // @Description: Fallback field of view used only when the standard CAM2_HFOV parameter is unset
    // @Units: deg
    // @Range: 5 179
    // @User: Advanced
    AP_GROUPINFO("C2_HFOV", 14, AP_CargoImpact, _camera2_hfov_deg, 65.0f),

    // @Param: C2_VFOV
    // @DisplayName: Camera 2 vertical field of view
    // @Description: Fallback field of view used only when the standard CAM2_VFOV parameter is unset
    // @Units: deg
    // @Range: 5 179
    // @User: Advanced
    AP_GROUPINFO("C2_VFOV", 15, AP_CargoImpact, _camera2_vfov_deg, 51.1f),

    // @Param: C2_ROLL
    // @DisplayName: Camera 2 body roll
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("C2_ROLL", 16, AP_CargoImpact, _camera2_roll_deg, 0.0f),

    // @Param: C2_PITCH
    // @DisplayName: Camera 2 body pitch
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("C2_PITCH", 17, AP_CargoImpact, _camera2_pitch_deg, -90.0f),

    // @Param: C2_YAW
    // @DisplayName: Camera 2 body yaw
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("C2_YAW", 18, AP_CargoImpact, _camera2_yaw_deg, 0.0f),

    // @Param: C1_TYPE
    // @DisplayName: Camera 1 lens type
    // @Description: Fixed uses configured field of view. Dynamic requires fresh field of view updates from the camera companion protocol.
    // @Values: 0:Fixed,1:DynamicVarifocal
    // @User: Advanced
    AP_GROUPINFO("C1_TYPE", 19, AP_CargoImpact, _camera1_type, 0),

    // @Param: C1_MNT
    // @DisplayName: Camera 1 mount instance
    // @Description: Mount instance supplying live camera attitude, or minus one to use the configured body angles
    // @Range: -1 1
    // @User: Advanced
    AP_GROUPINFO("C1_MNT", 20, AP_CargoImpact, _camera1_mount_instance, -1),

    // @Param: C2_TYPE
    // @DisplayName: Camera 2 lens type
    // @Description: Fixed uses configured field of view. Dynamic requires fresh field of view updates from the camera companion protocol.
    // @Values: 0:Fixed,1:DynamicVarifocal
    // @User: Advanced
    AP_GROUPINFO("C2_TYPE", 21, AP_CargoImpact, _camera2_type, 0),

    // @Param: C2_MNT
    // @DisplayName: Camera 2 mount instance
    // @Description: Mount instance supplying live camera attitude, or minus one to use the configured body angles
    // @Range: -1 1
    // @User: Advanced
    AP_GROUPINFO("C2_MNT", 22, AP_CargoImpact, _camera2_mount_instance, -1),

    // @Param: WIND_SRC
    // @DisplayName: Wind source
    // @Description: Selects AHRS-estimated wind or a configured NED wind vector. The configured source is useful when the simulator or a ground station supplies known wind without an airspeed sensor.
    // @Values: 0:AHRS,1:Configured
    // @User: Advanced
    AP_GROUPINFO("WIND_SRC", 23, AP_CargoImpact, _wind_source, 0),

    // @Param: WIND_N
    // @DisplayName: Configured north wind
    // @Description: North component of the configured air-mass velocity used by the DragWind model
    // @Units: m/s
    // @Range: -50 50
    // @User: Advanced
    AP_GROUPINFO("WIND_N", 24, AP_CargoImpact, _wind_north_ms, 0.0f),

    // @Param: WIND_E
    // @DisplayName: Configured east wind
    // @Description: East component of the configured air-mass velocity used by the DragWind model
    // @Units: m/s
    // @Range: -50 50
    // @User: Advanced
    AP_GROUPINFO("WIND_E", 25, AP_CargoImpact, _wind_east_ms, 0.0f),

    AP_GROUPEND
};

AP_CargoImpact::AP_CargoImpact()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    _result.status = Status::DISABLED;
    _external_result.status = Status::EXTERNAL_STALE;
}

static bool vector_is_finite(const Vector3f &value)
{
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

static bool bytes_are_zero(const uint8_t *bytes, const uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        if (bytes[i] != 0) {
            return false;
        }
    }
    return true;
}

bool AP_CargoImpact::calculate_simple(const Vector3f &position_ned_m,
                                      const Vector3f &velocity_ned_ms,
                                      const float height_agl_m,
                                      const float release_delay_s,
                                      Result &result)
{
    if (!vector_is_finite(position_ned_m) || !vector_is_finite(velocity_ned_ms) ||
        !isfinite(height_agl_m) || height_agl_m <= 0.0f ||
        !isfinite(release_delay_s) || release_delay_s < 0.0f) {
        return false;
    }

    constexpr float gravity_mss = GRAVITY_MSS;
    const Vector3f separation_position = position_ned_m + velocity_ned_ms * release_delay_s;
    const float ground_down_m = position_ned_m.z + height_agl_m;
    const float fall_distance_m = ground_down_m - separation_position.z;
    if (fall_distance_m <= 0.0f) {
        return false;
    }

    const float discriminant = sq(velocity_ned_ms.z) + 2.0f * gravity_mss * fall_distance_m;
    if (discriminant < 0.0f) {
        return false;
    }
    const float flight_time_s = (-velocity_ned_ms.z + sqrtf(discriminant)) / gravity_mss;
    if (!isfinite(flight_time_s) || flight_time_s < 0.0f) {
        return false;
    }

    result.impact_ned_m = separation_position + velocity_ned_ms * flight_time_s;
    result.impact_ned_m.z = ground_down_m;
    result.time_to_impact_s = release_delay_s + flight_time_s;
    result.horizontal_uncertainty_m = 0.0f;
    result.status = Status::VALID;
    return true;
}

static Vector3f cargo_acceleration(const Vector3f &velocity_ned_ms,
                                   const Vector3f &wind_ned_ms,
                                   const float drag_scale)
{
    const Vector3f relative_airflow = wind_ned_ms - velocity_ned_ms;
    Vector3f acceleration = relative_airflow * (drag_scale * relative_airflow.length());
    acceleration.z += GRAVITY_MSS;
    return acceleration;
}

bool AP_CargoImpact::calculate_drag_wind(const Vector3f &position_ned_m,
                                         const Vector3f &velocity_ned_ms,
                                         const Vector3f &wind_ned_ms,
                                         const float height_agl_m,
                                         const float release_delay_s,
                                         const float mass_kg,
                                         const float cda_m2,
                                         const float air_density_kgm3,
                                         Result &result)
{
    if (!vector_is_finite(position_ned_m) || !vector_is_finite(velocity_ned_ms) ||
        !vector_is_finite(wind_ned_ms) || !isfinite(height_agl_m) || height_agl_m <= 0.0f ||
        !isfinite(release_delay_s) || release_delay_s < 0.0f ||
        !isfinite(mass_kg) || mass_kg <= 0.0f || !isfinite(cda_m2) || cda_m2 < 0.0f ||
        !isfinite(air_density_kgm3) || air_density_kgm3 <= 0.0f) {
        return false;
    }

    constexpr float dt_s = 0.02f;
    constexpr uint16_t max_steps = 3000;
    const float ground_down_m = position_ned_m.z + height_agl_m;
    Vector3f position = position_ned_m + velocity_ned_ms * release_delay_s;
    Vector3f velocity = velocity_ned_ms;
    const float drag_scale = 0.5f * air_density_kgm3 * cda_m2 / mass_kg;
    float elapsed_s = 0.0f;

    for (uint16_t i = 0; i < max_steps; i++) {
        const Vector3f previous_position = position;

        const Vector3f a1 = cargo_acceleration(velocity, wind_ned_ms, drag_scale);
        const Vector3f v2 = velocity + a1 * (0.5f * dt_s);
        const Vector3f a2 = cargo_acceleration(v2, wind_ned_ms, drag_scale);
        const Vector3f v3 = velocity + a2 * (0.5f * dt_s);
        const Vector3f a3 = cargo_acceleration(v3, wind_ned_ms, drag_scale);
        const Vector3f v4 = velocity + a3 * dt_s;
        const Vector3f a4 = cargo_acceleration(v4, wind_ned_ms, drag_scale);

        position += (velocity + v2 * 2.0f + v3 * 2.0f + v4) * (dt_s / 6.0f);
        velocity += (a1 + a2 * 2.0f + a3 * 2.0f + a4) * (dt_s / 6.0f);
        elapsed_s += dt_s;

        if (!vector_is_finite(position) || !vector_is_finite(velocity)) {
            return false;
        }
        if (position.z >= ground_down_m) {
            const float step_down_m = position.z - previous_position.z;
            const float fraction = is_positive(step_down_m) ?
                constrain_float((ground_down_m - previous_position.z) / step_down_m, 0.0f, 1.0f) : 1.0f;
            result.impact_ned_m = previous_position + (position - previous_position) * fraction;
            result.impact_ned_m.z = ground_down_m;
            result.time_to_impact_s = release_delay_s + elapsed_s - dt_s + fraction * dt_s;
            result.horizontal_uncertainty_m = 0.0f;
            result.status = Status::VALID;
            return true;
        }
    }
    return false;
}

void AP_CargoImpact::update()
{
    _result.mode = _mode;
    if (!_enable || _mode == Mode::DISABLED) {
        _result.status = Status::DISABLED;
        write_log();
        return;
    }

    if (_mode == Mode::EXTERNAL) {
        Location current_origin;
        if (!AP::ahrs().get_origin(current_origin)) {
            _external_update_ms = 0;
            _external_origin_valid = false;
            _result.status = Status::EXTERNAL_STALE;
            write_log();
            return;
        }
        if (!_external_origin_valid) {
            _external_origin = current_origin;
            _external_origin_valid = true;
        } else if (!current_origin.same_loc_as(_external_origin)) {
            _external_update_ms = 0;
            _external_origin_valid = false;
            _result.status = Status::EXTERNAL_STALE;
            write_log();
            return;
        }
        const uint32_t timeout_ms = MIN(uint32_t(MAX(100, _external_timeout_ms.get())),
                                        uint32_t(_external_valid_for_ms));
        if (_external_update_ms == 0 || AP_HAL::millis() - _external_update_ms > timeout_ms) {
            _result.status = Status::EXTERNAL_STALE;
            write_log();
            return;
        }
        _result = _external_result;
        _result.mode = Mode::EXTERNAL;
        write_log();
        return;
    }

    AP_AHRS &ahrs = AP::ahrs();
    Vector3f position_ned_m;
    if (!ahrs.get_relative_position_NED_origin_float(position_ned_m)) {
        _result.status = Status::NO_POSITION;
        write_log();
        return;
    }
    Vector3f velocity_ned_ms;
    if (!ahrs.get_velocity_NED(velocity_ned_ms)) {
        _result.status = Status::NO_VELOCITY;
        write_log();
        return;
    }
    float height_agl_m;
    if (!ahrs.get_hagl(height_agl_m) || !is_positive(height_agl_m)) {
        // GPS-only Copter operation does not necessarily provide terrain HAGL.
        // The EKF origin is established at the launch surface, so NED down
        // position supplies the relative GPS altitude without Odometry/VINS.
        const float gps_relative_height_m = -position_ned_m.z;
        if (is_positive(gps_relative_height_m)) {
            height_agl_m = gps_relative_height_m;
        } else if (!hal.util->get_soft_armed()) {
            // Provide a deterministic ground-level preview so an operator can
            // verify the dedicated OSD marker before allowing a mission to arm.
            height_agl_m = 0.1f;
        } else {
            _result.status = Status::NO_HEIGHT;
            write_log();
            return;
        }
    }

    bool success = false;
    if (_mode == Mode::SIMPLE) {
        success = calculate_simple(position_ned_m, velocity_ned_ms, height_agl_m,
                                   _release_delay_s, _result);
    } else if (_mode == Mode::DRAG_WIND) {
        Vector3f wind_ned_ms;
        if (_wind_source.get() == 1) {
            wind_ned_ms = Vector3f{_wind_north_ms.get(), _wind_east_ms.get(), 0.0f};
        } else if (!ahrs.wind_estimate(wind_ned_ms)) {
            _result.status = Status::NO_WIND;
            write_log();
            return;
        }
        success = calculate_drag_wind(position_ned_m, velocity_ned_ms, wind_ned_ms,
                                      height_agl_m, _release_delay_s, _mass_kg,
                                      _cda_m2, _air_density_kgm3, _result);
    }

    _result.mode = _mode;
    if (!success) {
        _result.status = Status::INVALID_CONFIG;
        write_log();
        return;
    }
    _result.sequence++;
    write_log();
}

void AP_CargoImpact::write_log() const
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: CIMP
    // @Vehicles: Copter
    // @Description: Passive civil cargo impact prediction
    // @Field: TimeUS: Time since system startup
    // @Field: Mode: Prediction mode
    // @Field: Status: Prediction validity status
    // @Field: Seq: Prediction sequence
    // @Field: N: Predicted impact north of the EKF origin
    // @Field: E: Predicted impact east of the EKF origin
    // @Field: D: Predicted impact down from the EKF origin
    // @Field: TTI: Predicted time to impact
    // @Field: Unc: Horizontal one-sigma uncertainty
    AP::logger().WriteStreaming("CIMP",
                                "TimeUS,Mode,Status,Seq,N,E,D,TTI,Unc",
                                "s---mmmsm",
                                "F00000000",
                                "QBBIfffff",
                                AP_HAL::micros64(),
                                uint8_t(_result.mode),
                                uint8_t(_result.status),
                                _result.sequence,
                                _result.impact_ned_m.x,
                                _result.impact_ned_m.y,
                                _result.impact_ned_m.z,
                                _result.time_to_impact_s,
                                _result.horizontal_uncertainty_m);
#endif
}

bool AP_CargoImpact::set_external_prediction(const Vector3f &impact_ned_m,
                                              const float time_to_impact_s,
                                              const float horizontal_uncertainty_m,
                                              const uint32_t sequence,
                                              const uint16_t valid_for_ms)
{
    if (!vector_is_finite(impact_ned_m) || !isfinite(time_to_impact_s) || time_to_impact_s < 0.0f ||
        !isfinite(horizontal_uncertainty_m) || horizontal_uncertainty_m < 0.0f || valid_for_ms == 0) {
        return false;
    }
    if (_external_update_ms != 0 && sequence <= _external_result.sequence) {
        return false;
    }

    _external_result.impact_ned_m = impact_ned_m;
    _external_result.time_to_impact_s = time_to_impact_s;
    _external_result.horizontal_uncertainty_m = horizontal_uncertainty_m;
    _external_result.sequence = sequence;
    _external_result.mode = Mode::EXTERNAL;
    _external_result.status = Status::VALID;
    _external_update_ms = AP_HAL::millis();
    _external_valid_for_ms = valid_for_ms;
    _external_origin_valid = AP::ahrs().get_origin(_external_origin);
    return true;
}

bool AP_CargoImpact::handle_external_packet(const uint8_t *payload, const uint8_t payload_length)
{
    if (payload == nullptr || payload_length != sizeof(ExternalPredictionPacket)) {
        return false;
    }
    ExternalPredictionPacket packet{};
    memcpy(&packet, payload, sizeof(packet));
    constexpr uint8_t input_message_type = 1;
    constexpr uint8_t valid_flag = 1U;
    if (packet.magic != EXTERNAL_MAGIC || packet.version != EXTERNAL_PROTOCOL_VERSION ||
        (packet.flags & valid_flag) == 0) {
        return false;
    }
    if (packet.message_type == 2) {
        ExternalCameraPacket camera_packet{};
        memcpy(&camera_packet, payload, sizeof(camera_packet));
        if (camera_packet.frame != 0 ||
            !bytes_are_zero(camera_packet.reserved0, sizeof(camera_packet.reserved0)) ||
            !bytes_are_zero(camera_packet.reserved1, sizeof(camera_packet.reserved1))) {
            return false;
        }
        return set_dynamic_camera_fov(camera_packet.camera_index,
                                      camera_packet.horizontal_fov_deg,
                                      camera_packet.vertical_fov_deg,
                                      camera_packet.sequence,
                                      camera_packet.valid_for_ms);
    }
    if (packet.message_type != input_message_type || packet.frame != LOCAL_NED_FRAME ||
        packet.reserved != 0) {
        return false;
    }
    return set_external_prediction(Vector3f{packet.x_north_m, packet.y_east_m, packet.z_down_m},
                                   packet.time_to_impact_s, packet.horizontal_uncertainty_m,
                                   packet.sequence, packet.valid_for_ms);
}

bool AP_CargoImpact::set_dynamic_camera_fov(const uint8_t camera_index,
                                            const float horizontal_fov_deg,
                                            const float vertical_fov_deg,
                                            const uint32_t sequence,
                                            const uint16_t valid_for_ms)
{
    if (camera_index > 1 || !isfinite(horizontal_fov_deg) || !isfinite(vertical_fov_deg) ||
        horizontal_fov_deg < 5.0f || horizontal_fov_deg >= 180.0f ||
        vertical_fov_deg < 5.0f || vertical_fov_deg >= 180.0f || valid_for_ms == 0) {
        return false;
    }
    DynamicCameraState &state = _dynamic_camera[camera_index];
    if (state.valid && sequence <= state.sequence) {
        return false;
    }
    state.horizontal_fov_deg = horizontal_fov_deg;
    state.vertical_fov_deg = vertical_fov_deg;
    state.sequence = sequence;
    state.update_ms = AP_HAL::millis();
    state.valid_for_ms = valid_for_ms;
    state.valid = true;
    return true;
}

bool AP_CargoImpact::get_osd_projection(const uint8_t centre_x,
                                        const uint8_t centre_y,
                                        OSDProjection &projection) const
{
    if (!_result.valid()) {
        return false;
    }

    Vector3f vehicle_position_ned_m;
    const AP_AHRS &ahrs = AP::ahrs();
    if (!ahrs.get_relative_position_NED_origin_float(vehicle_position_ned_m)) {
        return false;
    }

    const Vector3f line_of_sight_ned = _result.impact_ned_m - vehicle_position_ned_m;

    const bool camera2 = _active_camera.get() == 1;
    float hfov_deg = camera2 ? _camera2_hfov_deg.get() : _camera1_hfov_deg.get();
    float vfov_deg = camera2 ? _camera2_vfov_deg.get() : _camera1_vfov_deg.get();
    const bool dynamic_camera = (camera2 ? _camera2_type.get() : _camera1_type.get()) == 1;
    if (dynamic_camera) {
        const DynamicCameraState &state = _dynamic_camera[camera2 ? 1 : 0];
        if (!state.valid || AP_HAL::millis() - state.update_ms > state.valid_for_ms) {
            return false;
        }
        hfov_deg = state.horizontal_fov_deg;
        vfov_deg = state.vertical_fov_deg;
    } else {
        // Fixed cameras use ArduPilot's standard CAM1_/CAM2_ calibration.
        // CIMP_C1_/C2_ values remain a fallback for a plain analogue camera.
#if AP_CAMERA_ENABLED
        AP_Camera *camera = AP::camera();
        float standard_hfov_deg;
        float standard_vfov_deg;
        if (camera != nullptr &&
            camera->get_configured_fov(camera2 ? 1 : 0, standard_hfov_deg, standard_vfov_deg)) {
            hfov_deg = standard_hfov_deg;
            vfov_deg = standard_vfov_deg;
        }
#endif
    }
    float roll_deg = camera2 ? _camera2_roll_deg.get() : _camera1_roll_deg.get();
    float pitch_deg = camera2 ? _camera2_pitch_deg.get() : _camera1_pitch_deg.get();
    float yaw_deg = camera2 ? _camera2_yaw_deg.get() : _camera1_yaw_deg.get();
    const int8_t mount_instance = camera2 ? _camera2_mount_instance.get() : _camera1_mount_instance.get();
    Vector3f line_of_sight_camera;
#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP::mount();
    if (mount_instance >= 0) {
        if (mount == nullptr ||
            !mount->get_attitude_euler(uint8_t(mount_instance), roll_deg, pitch_deg, yaw_deg)) {
            return false;
        }
        Matrix3f camera_to_ned;
        camera_to_ned.from_euler(radians(roll_deg), radians(pitch_deg),
                                 ahrs.get_yaw_rad() + radians(yaw_deg));
        line_of_sight_camera = camera_to_ned.mul_transpose(line_of_sight_ned);
    } else
#else
    if (mount_instance >= 0) {
        return false;
    }
#endif
    {
        const Vector3f line_of_sight_body =
            ahrs.get_rotation_body_to_ned().mul_transpose(line_of_sight_ned);
        Matrix3f camera_to_body;
        camera_to_body.from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg));
        line_of_sight_camera = camera_to_body.mul_transpose(line_of_sight_body);
    }
    if (hfov_deg < 5.0f || hfov_deg >= 180.0f || vfov_deg < 5.0f || vfov_deg >= 180.0f) {
        return false;
    }

    return project_camera_line_of_sight(line_of_sight_camera, hfov_deg, vfov_deg,
                                        centre_x, centre_y, projection);
}

bool AP_CargoImpact::project_camera_line_of_sight(const Vector3f &line_of_sight_camera,
                                                  const float horizontal_fov_deg,
                                                  const float vertical_fov_deg,
                                                  const uint8_t centre_x,
                                                  const uint8_t centre_y,
                                                  OSDProjection &projection)
{
    if (!vector_is_finite(line_of_sight_camera) ||
        horizontal_fov_deg < 5.0f || horizontal_fov_deg >= 180.0f ||
        vertical_fov_deg < 5.0f || vertical_fov_deg >= 180.0f) {
        return false;
    }
    projection.behind_camera = line_of_sight_camera.x <= 0.0f;
    if (projection.behind_camera) {
        projection.clipped = false;
        return true;
    }

    const float horizontal_angle = atan2f(line_of_sight_camera.y, line_of_sight_camera.x);
    const float vertical_angle = atan2f(line_of_sight_camera.z, line_of_sight_camera.x);
    const float x_offset = horizontal_angle / radians(horizontal_fov_deg * 0.5f) * 15.0f;
    const float y_offset = vertical_angle / radians(vertical_fov_deg * 0.5f) * 8.0f;
    const int16_t raw_x = int16_t(roundf(centre_x + x_offset));
    const int16_t raw_y = int16_t(roundf(centre_y + y_offset));
    // Keep one-cell margins for the 3x3 OSD reticle.
    projection.x = constrain_int16(raw_x, 1, 28);
    projection.y = constrain_int16(raw_y, 1, 14);
    projection.clipped = raw_x != projection.x || raw_y != projection.y;
    return true;
}

namespace AP {
AP_CargoImpact *cargo_impact()
{
    return AP_CargoImpact::get_singleton();
}
}

#endif // AP_CARGO_IMPACT_ENABLED
