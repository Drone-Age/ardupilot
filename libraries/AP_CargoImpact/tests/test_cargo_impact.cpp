#include <AP_gtest.h>

#include <AP_CargoImpact/AP_CargoImpact.h>

#if AP_CARGO_IMPACT_ENABLED

static_assert(sizeof(AP_CargoImpact::ExternalPredictionPacket) == 36, "prediction wire size changed");
static_assert(sizeof(AP_CargoImpact::ExternalCameraPacket) == 36, "camera wire size changed");

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

TEST(CargoImpact, SimpleStationaryDrop)
{
    AP_CargoImpact::Result result{};
    ASSERT_TRUE(AP_CargoImpact::calculate_simple(Vector3f{}, Vector3f{}, 100.0f, 0.0f, result));
    EXPECT_NEAR(result.time_to_impact_s, sqrtf(200.0f / GRAVITY_MSS), 1.0e-4f);
    EXPECT_NEAR(result.impact_ned_m.x, 0.0f, 1.0e-4f);
    EXPECT_NEAR(result.impact_ned_m.y, 0.0f, 1.0e-4f);
    EXPECT_NEAR(result.impact_ned_m.z, 100.0f, 1.0e-4f);
}

TEST(CargoImpact, SimpleRetainsHorizontalVelocity)
{
    AP_CargoImpact::Result result{};
    ASSERT_TRUE(AP_CargoImpact::calculate_simple(Vector3f{}, Vector3f{10.0f, -2.0f, 0.0f},
                                                  100.0f, 0.25f, result));
    EXPECT_NEAR(result.impact_ned_m.x, 10.0f * result.time_to_impact_s, 1.0e-3f);
    EXPECT_NEAR(result.impact_ned_m.y, -2.0f * result.time_to_impact_s, 1.0e-3f);
}

TEST(CargoImpact, DragReducesStillAirTravel)
{
    AP_CargoImpact::Result simple{};
    AP_CargoImpact::Result drag{};
    const Vector3f velocity{25.0f, 0.0f, 0.0f};
    ASSERT_TRUE(AP_CargoImpact::calculate_simple(Vector3f{}, velocity, 100.0f, 0.0f, simple));
    ASSERT_TRUE(AP_CargoImpact::calculate_drag_wind(Vector3f{}, velocity, Vector3f{}, 100.0f,
                                                    0.0f, 2.0f, 0.10f, 1.225f, drag));
    EXPECT_LT(drag.impact_ned_m.x, simple.impact_ned_m.x);
    EXPECT_GT(drag.time_to_impact_s, simple.time_to_impact_s);
}

TEST(CargoImpact, RejectsInvalidInputs)
{
    AP_CargoImpact::Result result{};
    EXPECT_FALSE(AP_CargoImpact::calculate_simple(Vector3f{}, Vector3f{}, 0.0f, 0.0f, result));
    EXPECT_FALSE(AP_CargoImpact::calculate_drag_wind(Vector3f{}, Vector3f{}, Vector3f{}, 10.0f,
                                                     0.0f, 0.0f, 0.03f, 1.225f, result));
}

TEST(CargoImpact, DragWithZeroAreaMatchesSimpleModel)
{
    AP_CargoImpact::Result simple{};
    AP_CargoImpact::Result drag{};
    const Vector3f velocity{12.0f, -3.0f, 1.0f};
    ASSERT_TRUE(AP_CargoImpact::calculate_simple(Vector3f{}, velocity, 400.0f, 0.0f, simple));
    ASSERT_TRUE(AP_CargoImpact::calculate_drag_wind(Vector3f{}, velocity, Vector3f{15.0f, 0.0f, 0.0f},
                                                    400.0f, 0.0f, 5.0f, 0.0f, 1.225f, drag));
    EXPECT_NEAR(drag.impact_ned_m.x, simple.impact_ned_m.x, 0.3f);
    EXPECT_NEAR(drag.impact_ned_m.y, simple.impact_ned_m.y, 0.1f);
    EXPECT_NEAR(drag.time_to_impact_s, simple.time_to_impact_s, 0.03f);
}

TEST(CargoImpact, DragModelRespondsToFifteenMetrePerSecondWindAtFourHundredMetres)
{
    AP_CargoImpact::Result result{};
    ASSERT_TRUE(AP_CargoImpact::calculate_drag_wind(Vector3f{}, Vector3f{},
                                                    Vector3f{0.0f, 15.0f, 0.0f}, 400.0f,
                                                    0.0f, 2.0f, 0.10f, 1.225f, result));
    EXPECT_GT(result.impact_ned_m.y, 1.0f);
    EXPECT_GT(result.time_to_impact_s, 0.0f);
}

TEST(CargoImpact, DynamicCameraFovRejectsInvalidAndOutOfOrderUpdates)
{
    AP_CargoImpact assistant;
    EXPECT_FALSE(assistant.set_dynamic_camera_fov(2, 60.0f, 45.0f, 1, 500));
    EXPECT_FALSE(assistant.set_dynamic_camera_fov(0, 2.0f, 45.0f, 1, 500));
    EXPECT_TRUE(assistant.set_dynamic_camera_fov(0, 60.0f, 45.0f, 2, 500));
    EXPECT_FALSE(assistant.set_dynamic_camera_fov(0, 55.0f, 40.0f, 2, 500));
}

TEST(CargoImpact, CameraProjectionCentersAndClips)
{
    AP_CargoImpact::OSDProjection projection{};
    ASSERT_TRUE(AP_CargoImpact::project_camera_line_of_sight(Vector3f{10.0f, 0.0f, 0.0f},
                                                              60.0f, 40.0f, 15, 8, projection));
    EXPECT_EQ(projection.x, 15);
    EXPECT_EQ(projection.y, 8);
    EXPECT_FALSE(projection.clipped);
    ASSERT_TRUE(AP_CargoImpact::project_camera_line_of_sight(Vector3f{1.0f, 10.0f, 0.0f},
                                                              60.0f, 40.0f, 15, 8, projection));
    EXPECT_EQ(projection.x, 28);
    EXPECT_TRUE(projection.clipped);
    ASSERT_TRUE(AP_CargoImpact::project_camera_line_of_sight(Vector3f{-1.0f, 0.0f, 0.0f},
                                                              60.0f, 40.0f, 15, 8, projection));
    EXPECT_TRUE(projection.behind_camera);
}

TEST(CargoImpact, CameraPacketValidatesEnvelopeAndReservedBytes)
{
    AP_CargoImpact assistant;
    AP_CargoImpact::ExternalCameraPacket packet{};
    packet.magic = AP_CargoImpact::EXTERNAL_MAGIC;
    packet.version = AP_CargoImpact::EXTERNAL_PROTOCOL_VERSION;
    packet.message_type = 2;
    packet.flags = 1;
    packet.sequence = 1;
    packet.camera_index = 0;
    packet.horizontal_fov_deg = 60.0f;
    packet.vertical_fov_deg = 45.0f;
    packet.valid_for_ms = 500;
    EXPECT_TRUE(assistant.handle_external_packet(reinterpret_cast<const uint8_t *>(&packet),
                                                  sizeof(packet)));
    packet.sequence++;
    packet.frame = 1;
    EXPECT_FALSE(assistant.handle_external_packet(reinterpret_cast<const uint8_t *>(&packet),
                                                   sizeof(packet)));
    packet.frame = 0;
    packet.reserved1[0] = 1;
    EXPECT_FALSE(assistant.handle_external_packet(reinterpret_cast<const uint8_t *>(&packet),
                                                   sizeof(packet)));
}

#endif

AP_GTEST_MAIN()
