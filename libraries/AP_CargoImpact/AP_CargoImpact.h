#pragma once

#include "AP_CargoImpact_config.h"

#if AP_CARGO_IMPACT_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>

class AP_CargoImpact {
public:
    static constexpr uint16_t TUNNEL_PAYLOAD_TYPE = 200;
    static constexpr uint32_t EXTERNAL_MAGIC = 0x49434144U; // "DACI" on the wire
    static constexpr uint8_t EXTERNAL_PROTOCOL_VERSION = 1;
    static constexpr uint8_t LOCAL_NED_FRAME = 1;

    struct PACKED ExternalPredictionPacket {
        uint32_t magic;
        uint8_t version;
        uint8_t message_type;
        uint8_t frame;
        uint8_t flags;
        uint32_t sequence;
        float x_north_m;
        float y_east_m;
        float z_down_m;
        float time_to_impact_s;
        float horizontal_uncertainty_m;
        uint16_t valid_for_ms;
        uint16_t reserved;
    };

    struct PACKED ExternalCameraPacket {
        uint32_t magic;
        uint8_t version;
        uint8_t message_type;
        uint8_t frame;
        uint8_t flags;
        uint32_t sequence;
        uint8_t camera_index;
        uint8_t reserved0[3];
        float horizontal_fov_deg;
        float vertical_fov_deg;
        uint16_t valid_for_ms;
        uint8_t reserved1[10];
    };

    AP_CargoImpact();

    CLASS_NO_COPY(AP_CargoImpact);

    enum class Mode : int8_t {
        DISABLED = 0,
        SIMPLE = 1,
        DRAG_WIND = 2,
        EXTERNAL = 3,
    };

    enum class Status : uint8_t {
        DISABLED = 0,
        VALID = 1,
        NO_POSITION = 2,
        NO_VELOCITY = 3,
        NO_HEIGHT = 4,
        NO_WIND = 5,
        INVALID_CONFIG = 6,
        EXTERNAL_STALE = 7,
    };

    struct Result {
        Vector3f impact_ned_m;
        float time_to_impact_s;
        float horizontal_uncertainty_m;
        uint32_t sequence;
        Mode mode;
        Status status;

        bool valid() const { return status == Status::VALID; }
    };

    struct OSDProjection {
        int8_t x;
        int8_t y;
        bool clipped;
        bool behind_camera;
    };

    static const AP_Param::GroupInfo var_info[];

    static AP_CargoImpact *get_singleton() { return _singleton; }

    void update();
    const Result &result() const { return _result; }
    bool get_osd_projection(uint8_t centre_x, uint8_t centre_y, OSDProjection &projection) const;

    bool set_external_prediction(const Vector3f &impact_ned_m,
                                 float time_to_impact_s,
                                 float horizontal_uncertainty_m,
                                 uint32_t sequence,
                                 uint16_t valid_for_ms);
    bool handle_external_packet(const uint8_t *payload, uint8_t payload_length);
    bool set_dynamic_camera_fov(uint8_t camera_index, float horizontal_fov_deg,
                                float vertical_fov_deg, uint32_t sequence,
                                uint16_t valid_for_ms);

    static bool calculate_simple(const Vector3f &position_ned_m,
                                 const Vector3f &velocity_ned_ms,
                                 float height_agl_m,
                                 float release_delay_s,
                                 Result &result);

    static bool calculate_drag_wind(const Vector3f &position_ned_m,
                                    const Vector3f &velocity_ned_ms,
                                    const Vector3f &wind_ned_ms,
                                    float height_agl_m,
                                    float release_delay_s,
                                    float mass_kg,
                                    float cda_m2,
                                    float air_density_kgm3,
                                    Result &result);
    static bool project_camera_line_of_sight(const Vector3f &line_of_sight_camera,
                                             float horizontal_fov_deg,
                                             float vertical_fov_deg,
                                             uint8_t centre_x,
                                             uint8_t centre_y,
                                             OSDProjection &projection);

private:
    static AP_CargoImpact *_singleton;

    AP_Int8 _enable;
    AP_Enum<Mode> _mode;
    AP_Float _mass_kg;
    AP_Float _cda_m2;
    AP_Float _release_delay_s;
    AP_Float _air_density_kgm3;
    AP_Int8 _wind_source;
    AP_Float _wind_north_ms;
    AP_Float _wind_east_ms;
    AP_Int16 _external_timeout_ms;
    AP_Int8 _active_camera;
    AP_Int8 _camera1_type;
    AP_Int8 _camera1_mount_instance;
    AP_Float _camera1_hfov_deg;
    AP_Float _camera1_vfov_deg;
    AP_Float _camera1_roll_deg;
    AP_Float _camera1_pitch_deg;
    AP_Float _camera1_yaw_deg;
    AP_Int8 _camera2_type;
    AP_Int8 _camera2_mount_instance;
    AP_Float _camera2_hfov_deg;
    AP_Float _camera2_vfov_deg;
    AP_Float _camera2_roll_deg;
    AP_Float _camera2_pitch_deg;
    AP_Float _camera2_yaw_deg;

    Result _result{};
    Result _external_result{};
    uint32_t _external_update_ms;
    uint16_t _external_valid_for_ms;
    Location _external_origin{};
    bool _external_origin_valid;

    struct DynamicCameraState {
        float horizontal_fov_deg;
        float vertical_fov_deg;
        uint32_t sequence;
        uint32_t update_ms;
        uint16_t valid_for_ms;
    } _dynamic_camera[2]{};

    void write_log() const;
};

namespace AP {
AP_CargoImpact *cargo_impact();
}

#endif // AP_CARGO_IMPACT_ENABLED
