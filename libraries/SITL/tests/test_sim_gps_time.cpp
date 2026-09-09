#include <AP_gtest.h>
#include <SITL/SIM_GPS.h>

TEST(GPSTime, TenHertzSamplesRemainDistinct)
{
    struct timeval tv {};
    tv.tv_sec = 1700000000;
    const auto start = SITL::GPS_Backend::gps_time(tv);
    for (uint32_t ms = 0; ms < 1000; ms += 100) {
        tv.tv_usec = ms * 1000;
        const auto sample = SITL::GPS_Backend::gps_time(tv);
        EXPECT_EQ(sample.week, start.week);
        EXPECT_EQ(sample.ms, start.ms + ms);
    }
}

TEST(GPSTime, PreservesActualSamplePhase)
{
    struct timeval tv {};
    tv.tv_sec = 1700000000;
    const auto start = SITL::GPS_Backend::gps_time(tv);
    tv.tv_usec = 123456;
    EXPECT_EQ(SITL::GPS_Backend::gps_time(tv).ms, start.ms + 123);
    tv.tv_sec++;
    tv.tv_usec = 234567;
    EXPECT_EQ(SITL::GPS_Backend::gps_time(tv).ms, start.ms + 1234);
}

TEST(GPSTime, WeekRollover)
{
    // GPS week 2289 begins at 2023-11-18 23:59:42 UTC (18 leap seconds).
    struct timeval tv {};
    tv.tv_sec = 1700351981;
    tv.tv_usec = 900000;
    const auto before = SITL::GPS_Backend::gps_time(tv);
    tv.tv_sec++;
    tv.tv_usec = 0;
    const auto after = SITL::GPS_Backend::gps_time(tv);
    EXPECT_EQ(before.ms, 604799900U);
    EXPECT_EQ(after.ms, 0U);
    EXPECT_EQ(after.week, before.week + 1);
}

AP_GTEST_MAIN()
