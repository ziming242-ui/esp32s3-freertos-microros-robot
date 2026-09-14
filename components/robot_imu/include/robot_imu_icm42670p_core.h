#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_IMU_ICM42670P_DATA_START_REGISTER UINT8_C(0x0B)
#define ROBOT_IMU_ICM42670P_DATA_LENGTH_BYTES 12U
#define ROBOT_IMU_ICM42670P_AXIS_COUNT 3U
#define ROBOT_IMU_ICM42670P_ACCEL_LSB_PER_G 8192.0f
#define ROBOT_IMU_ICM42670P_GYRO_LSB_PER_DPS 16.4f
#define ROBOT_IMU_ICM42670P_STANDARD_GRAVITY_MPS2 9.80665f

typedef struct {
    int16_t acceleration_raw[ROBOT_IMU_ICM42670P_AXIS_COUNT];
    int16_t angular_velocity_raw[ROBOT_IMU_ICM42670P_AXIS_COUNT];
} robot_imu_icm42670p_raw_sample_t;

typedef struct {
    float linear_acceleration_mps2[ROBOT_IMU_ICM42670P_AXIS_COUNT];
    float angular_velocity_rad_s[ROBOT_IMU_ICM42670P_AXIS_COUNT];
} robot_imu_icm42670p_si_sample_t;

/*
 * Parses one burst read covering registers 0x0B through 0x16. Each axis is a
 * signed, big-endian 16-bit value. The output is unchanged on failure.
 */
bool robot_imu_icm42670p_parse_raw(
    const uint8_t *register_bytes,
    size_t register_byte_count,
    robot_imu_icm42670p_raw_sample_t *out_sample);

/*
 * Converts the configured +/-4 g and +/-2000 dps raw ranges to SI units.
 * Values remain in sensor-native axis order; installation mapping belongs to
 * the physical backend. The output is unchanged on failure.
 */
bool robot_imu_icm42670p_convert_to_si(
    const robot_imu_icm42670p_raw_sample_t *raw_sample,
    robot_imu_icm42670p_si_sample_t *out_sample);

#ifdef __cplusplus
}
#endif
