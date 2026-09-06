#include "robot_imu_policy.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    robot_imu_sample_t sample = {
        .linear_acceleration_mps2 = {0.0f, 0.0f, 9.80665f},
        .angular_velocity_rad_s = {0.0f, 0.0f, 0.0f},
        .sampled_at_us = 1000000,
        .sample_sequence = 1,
    };

    assert(robot_imu_classify_sample(false, NULL, 0, 100000) ==
           ROBOT_IMU_SAMPLE_NO_SAMPLE);
    assert(robot_imu_classify_sample(true, &sample, 1050000, 100000) ==
           ROBOT_IMU_SAMPLE_FRESH);
    assert(robot_imu_classify_sample(true, &sample, 1100000, 100000) ==
           ROBOT_IMU_SAMPLE_FRESH);
    assert(robot_imu_classify_sample(true, &sample, 1100001, 100000) ==
           ROBOT_IMU_SAMPLE_STALE);
    assert(robot_imu_classify_sample(true, &sample, 999999, 100000) ==
           ROBOT_IMU_SAMPLE_CLOCK_INVALID);

    sample.angular_velocity_rad_s[1] = NAN;
    assert(robot_imu_classify_sample(true, &sample, 1050000, 100000) ==
           ROBOT_IMU_SAMPLE_NONFINITE);

    puts("robot_imu_policy: PASS");
    return 0;
}
