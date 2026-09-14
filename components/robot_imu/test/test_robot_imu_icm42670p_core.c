#include "robot_imu_icm42670p_core.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>

static unsigned s_checks;
static unsigned s_failures;

#define CHECK(condition)                                                     \
    do {                                                                     \
        ++s_checks;                                                          \
        if (!(condition)) {                                                  \
            ++s_failures;                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,       \
                    #condition);                                             \
        }                                                                    \
    } while (0)

static void check_near(float actual, float expected, float tolerance)
{
    CHECK(isfinite(actual));
    CHECK(fabsf(actual - expected) <= tolerance);
}

static void check_raw_sample_equal(
    const robot_imu_icm42670p_raw_sample_t *actual,
    const robot_imu_icm42670p_raw_sample_t *expected)
{
    for (size_t axis = 0; axis < ROBOT_IMU_ICM42670P_AXIS_COUNT; ++axis) {
        CHECK(actual->acceleration_raw[axis] ==
              expected->acceleration_raw[axis]);
        CHECK(actual->angular_velocity_raw[axis] ==
              expected->angular_velocity_raw[axis]);
    }
}

static void check_si_sample_equal(
    const robot_imu_icm42670p_si_sample_t *actual,
    const robot_imu_icm42670p_si_sample_t *expected)
{
    for (size_t axis = 0; axis < ROBOT_IMU_ICM42670P_AXIS_COUNT; ++axis) {
        check_near(actual->linear_acceleration_mps2[axis],
                   expected->linear_acceleration_mps2[axis], 0.0f);
        check_near(actual->angular_velocity_rad_s[axis],
                   expected->angular_velocity_rad_s[axis], 0.0f);
    }
}

static void test_register_contract(void)
{
    CHECK(ROBOT_IMU_ICM42670P_DATA_START_REGISTER == UINT8_C(0x0B));
    CHECK(ROBOT_IMU_ICM42670P_DATA_LENGTH_BYTES == 12U);
    CHECK(ROBOT_IMU_ICM42670P_AXIS_COUNT == 3U);
    check_near(ROBOT_IMU_ICM42670P_ACCEL_LSB_PER_G, 8192.0f, 0.0f);
    check_near(ROBOT_IMU_ICM42670P_GYRO_LSB_PER_DPS, 16.4f, 0.0f);
}

static void test_big_endian_signed_layout(void)
{
    const uint8_t register_bytes[ROBOT_IMU_ICM42670P_DATA_LENGTH_BYTES] = {
        0x12, 0x34, /* 0x0B/0x0C: accel X =  4660 */
        0xFE, 0xDC, /* 0x0D/0x0E: accel Y =  -292 */
        0x7F, 0xFF, /* 0x0F/0x10: accel Z = 32767 */
        0x80, 0x00, /* 0x11/0x12: gyro X = -32768 */
        0x00, 0x01, /* 0x13/0x14: gyro Y =      1 */
        0xFF, 0xFF, /* 0x15/0x16: gyro Z =     -1 */
    };
    robot_imu_icm42670p_raw_sample_t raw_sample;

    CHECK(robot_imu_icm42670p_parse_raw(
        register_bytes, sizeof(register_bytes), &raw_sample));
    CHECK(raw_sample.acceleration_raw[0] == 4660);
    CHECK(raw_sample.acceleration_raw[1] == -292);
    CHECK(raw_sample.acceleration_raw[2] == INT16_MAX);
    CHECK(raw_sample.angular_velocity_raw[0] == INT16_MIN);
    CHECK(raw_sample.angular_velocity_raw[1] == 1);
    CHECK(raw_sample.angular_velocity_raw[2] == -1);
}

static void test_parse_rejects_invalid_arguments(void)
{
    const uint8_t register_bytes[ROBOT_IMU_ICM42670P_DATA_LENGTH_BYTES] = {0};
    const robot_imu_icm42670p_raw_sample_t sentinel = {
        .acceleration_raw = {1, 2, 3},
        .angular_velocity_raw = {4, 5, 6},
    };
    robot_imu_icm42670p_raw_sample_t output = sentinel;

    CHECK(!robot_imu_icm42670p_parse_raw(
        register_bytes, sizeof(register_bytes) - 1U, &output));
    check_raw_sample_equal(&output, &sentinel);
    CHECK(!robot_imu_icm42670p_parse_raw(
        register_bytes, sizeof(register_bytes) + 1U, &output));
    check_raw_sample_equal(&output, &sentinel);
    CHECK(!robot_imu_icm42670p_parse_raw(
        NULL, sizeof(register_bytes), &output));
    CHECK(!robot_imu_icm42670p_parse_raw(
        register_bytes, sizeof(register_bytes), NULL));
}

static void test_six_axis_conversion(void)
{
    const robot_imu_icm42670p_raw_sample_t raw_sample = {
        .acceleration_raw = {8192, -8192, 4096},
        .angular_velocity_raw = {164, -164, 820},
    };
    robot_imu_icm42670p_si_sample_t si_sample;

    CHECK(robot_imu_icm42670p_convert_to_si(&raw_sample, &si_sample));
    check_near(si_sample.linear_acceleration_mps2[0], 9.80665f, 1.0e-6f);
    check_near(si_sample.linear_acceleration_mps2[1], -9.80665f, 1.0e-6f);
    check_near(si_sample.linear_acceleration_mps2[2], 4.903325f, 1.0e-6f);
    check_near(si_sample.angular_velocity_rad_s[0], 0.174532925f,
               1.0e-6f);
    check_near(si_sample.angular_velocity_rad_s[1], -0.174532925f,
               1.0e-6f);
    check_near(si_sample.angular_velocity_rad_s[2], 0.872664626f,
               1.0e-6f);
}

static void test_boundary_conversion(void)
{
    const robot_imu_icm42670p_raw_sample_t raw_sample = {
        .acceleration_raw = {INT16_MAX, INT16_MIN, 0},
        .angular_velocity_raw = {INT16_MAX, INT16_MIN, 0},
    };
    robot_imu_icm42670p_si_sample_t si_sample;

    CHECK(robot_imu_icm42670p_convert_to_si(&raw_sample, &si_sample));
    check_near(si_sample.linear_acceleration_mps2[0], 39.2254029f, 1.0e-5f);
    check_near(si_sample.linear_acceleration_mps2[1], -39.2266f, 1.0e-5f);
    check_near(si_sample.linear_acceleration_mps2[2], 0.0f, 0.0f);
    check_near(si_sample.angular_velocity_rad_s[0], 34.8714656f,
               1.0e-5f);
    check_near(si_sample.angular_velocity_rad_s[1], -34.8725298f,
               1.0e-5f);
    check_near(si_sample.angular_velocity_rad_s[2], 0.0f, 0.0f);
}

static void test_conversion_rejects_invalid_arguments(void)
{
    const robot_imu_icm42670p_raw_sample_t raw_sample = {0};
    const robot_imu_icm42670p_si_sample_t sentinel = {
        .linear_acceleration_mps2 = {1.0f, 2.0f, 3.0f},
        .angular_velocity_rad_s = {4.0f, 5.0f, 6.0f},
    };
    robot_imu_icm42670p_si_sample_t output = sentinel;

    CHECK(!robot_imu_icm42670p_convert_to_si(NULL, &output));
    check_si_sample_equal(&output, &sentinel);
    CHECK(!robot_imu_icm42670p_convert_to_si(&raw_sample, NULL));
}

int main(void)
{
    test_register_contract();
    test_big_endian_signed_layout();
    test_parse_rejects_invalid_arguments();
    test_six_axis_conversion();
    test_boundary_conversion();
    test_conversion_rejects_invalid_arguments();

    if (s_failures != 0U) {
        fprintf(stderr, "robot_imu_icm42670p_core: %u checks, %u failures\n",
                s_checks, s_failures);
        return 1;
    }

    printf("robot_imu_icm42670p_core: %u checks, 0 failures\n", s_checks);
    return 0;
}
