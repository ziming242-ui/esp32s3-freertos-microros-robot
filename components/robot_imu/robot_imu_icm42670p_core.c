#include "robot_imu_icm42670p_core.h"

#include <stdint.h>

#define ROBOT_IMU_DEGREES_TO_RADIANS 0.01745329251994329577f

static int16_t decode_big_endian_i16(const uint8_t *bytes)
{
    int32_t value = ((int32_t)bytes[0] << 8) | (int32_t)bytes[1];
    if (value >= 0x8000) {
        value -= 0x10000;
    }
    return (int16_t)value;
}

bool robot_imu_icm42670p_parse_raw(
    const uint8_t *register_bytes,
    size_t register_byte_count,
    robot_imu_icm42670p_raw_sample_t *out_sample)
{
    if (register_bytes == NULL || out_sample == NULL ||
        register_byte_count != ROBOT_IMU_ICM42670P_DATA_LENGTH_BYTES) {
        return false;
    }

    robot_imu_icm42670p_raw_sample_t candidate = {0};
    for (size_t axis = 0; axis < ROBOT_IMU_ICM42670P_AXIS_COUNT; ++axis) {
        const size_t acceleration_offset = axis * 2U;
        const size_t angular_velocity_offset = 6U + axis * 2U;
        candidate.acceleration_raw[axis] =
            decode_big_endian_i16(&register_bytes[acceleration_offset]);
        candidate.angular_velocity_raw[axis] =
            decode_big_endian_i16(&register_bytes[angular_velocity_offset]);
    }

    *out_sample = candidate;
    return true;
}

bool robot_imu_icm42670p_convert_to_si(
    const robot_imu_icm42670p_raw_sample_t *raw_sample,
    robot_imu_icm42670p_si_sample_t *out_sample)
{
    if (raw_sample == NULL || out_sample == NULL) {
        return false;
    }

    const float acceleration_scale =
        ROBOT_IMU_ICM42670P_STANDARD_GRAVITY_MPS2 /
        ROBOT_IMU_ICM42670P_ACCEL_LSB_PER_G;
    const float angular_velocity_scale =
        ROBOT_IMU_DEGREES_TO_RADIANS /
        ROBOT_IMU_ICM42670P_GYRO_LSB_PER_DPS;
    robot_imu_icm42670p_si_sample_t candidate = {0};

    for (size_t axis = 0; axis < ROBOT_IMU_ICM42670P_AXIS_COUNT; ++axis) {
        candidate.linear_acceleration_mps2[axis] =
            (float)raw_sample->acceleration_raw[axis] * acceleration_scale;
        candidate.angular_velocity_rad_s[axis] =
            (float)raw_sample->angular_velocity_raw[axis] *
            angular_velocity_scale;
    }

    *out_sample = candidate;
    return true;
}
