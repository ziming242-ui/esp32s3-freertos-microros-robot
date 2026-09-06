#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * One coherent IMU snapshot. The vector names include their SI units so the
 * ROS layer must not apply a second conversion.
 */
typedef struct {
    float linear_acceleration_mps2[3];
    float angular_velocity_rad_s[3];
    int64_t sampled_at_us;
    uint32_t sample_sequence;
} robot_imu_sample_t;

typedef enum {
    ROBOT_IMU_SAMPLE_FRESH = 0,
    ROBOT_IMU_SAMPLE_NO_SAMPLE,
    ROBOT_IMU_SAMPLE_STALE,
    ROBOT_IMU_SAMPLE_NONFINITE,
    ROBOT_IMU_SAMPLE_CLOCK_INVALID,
    ROBOT_IMU_SAMPLE_ARGUMENT_INVALID,
} robot_imu_sample_state_t;

/* Pure-C helpers: no ESP-IDF or ROS dependency, so they can run on a PC. */
bool robot_imu_sample_values_are_finite(const robot_imu_sample_t *sample);

robot_imu_sample_state_t robot_imu_classify_sample(
    bool has_sample,
    const robot_imu_sample_t *sample,
    int64_t now_us,
    int64_t stale_timeout_us);

const char *robot_imu_sample_state_name(robot_imu_sample_state_t state);

#ifdef __cplusplus
}
#endif
