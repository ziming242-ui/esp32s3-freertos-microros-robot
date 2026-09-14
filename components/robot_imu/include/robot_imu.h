#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "robot_imu_policy.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Starts the configured producer. The default backend is disabled. Selecting
 * the synthetic backend must be an explicit test configuration choice.
 */
esp_err_t robot_imu_start(void);

bool robot_imu_is_enabled(void);
const char *robot_imu_backend_name(void);
const char *robot_imu_frame_id(void);
int64_t robot_imu_stale_timeout_us(void);

/*
 * Producer-side entry point for an enabled backend. Call only after a sensor
 * read and parse have succeeded. The timestamp and sequence become visible
 * only if the complete snapshot is accepted by the length-one latest-sample
 * queue. The disabled backend deliberately has no producer queue.
 *
 * This is a task-context API, not an ISR API. Units are m/s^2 and rad/s.
 * Values must use the right-handed frame named by robot_imu_frame_id(). The
 * physical backend currently preserves the sensor-native axes; its mounting
 * transform to the robot base remains a separate hardware-validation gate.
 * The ROS publisher copies values as-is and never treats a frame label as an
 * axis conversion.
 */
esp_err_t robot_imu_submit_si_sample(
    const float linear_acceleration_mps2[3],
    const float angular_velocity_rad_s[3],
    int64_t sampled_at_us);

/* Copies one complete snapshot; false means that no valid sample exists. */
bool robot_imu_get_latest(robot_imu_sample_t *out_sample);

/*
 * Validation-only fault injection. While the physical backend is running,
 * pause acquisition without inventing samples or changing motor state.
 * This is accepted only for a bounded duration and is not a motion command.
 */
esp_err_t robot_imu_request_test_pause(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif
