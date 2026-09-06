#include "robot_imu_policy.h"

#include <math.h>
#include <stddef.h>

bool robot_imu_sample_values_are_finite(const robot_imu_sample_t *sample)
{
    if (sample == NULL) {
        return false;
    }

    for (size_t axis = 0; axis < 3; ++axis) {
        if (!isfinite(sample->linear_acceleration_mps2[axis]) ||
            !isfinite(sample->angular_velocity_rad_s[axis])) {
            return false;
        }
    }

    return true;
}

robot_imu_sample_state_t robot_imu_classify_sample(
    bool has_sample,
    const robot_imu_sample_t *sample,
    int64_t now_us,
    int64_t stale_timeout_us)
{
    if (!has_sample) {
        return ROBOT_IMU_SAMPLE_NO_SAMPLE;
    }

    if (sample == NULL || stale_timeout_us < 0) {
        return ROBOT_IMU_SAMPLE_ARGUMENT_INVALID;
    }

    if (!robot_imu_sample_values_are_finite(sample)) {
        return ROBOT_IMU_SAMPLE_NONFINITE;
    }

    if (sample->sampled_at_us < 0 || now_us < sample->sampled_at_us) {
        return ROBOT_IMU_SAMPLE_CLOCK_INVALID;
    }

    if ((now_us - sample->sampled_at_us) > stale_timeout_us) {
        return ROBOT_IMU_SAMPLE_STALE;
    }

    return ROBOT_IMU_SAMPLE_FRESH;
}

const char *robot_imu_sample_state_name(robot_imu_sample_state_t state)
{
    switch (state) {
    case ROBOT_IMU_SAMPLE_FRESH:
        return "fresh";
    case ROBOT_IMU_SAMPLE_NO_SAMPLE:
        return "no_sample";
    case ROBOT_IMU_SAMPLE_STALE:
        return "stale";
    case ROBOT_IMU_SAMPLE_NONFINITE:
        return "nonfinite";
    case ROBOT_IMU_SAMPLE_CLOCK_INVALID:
        return "clock_invalid";
    case ROBOT_IMU_SAMPLE_ARGUMENT_INVALID:
        return "argument_invalid";
    default:
        return "unknown";
    }
}
