#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts a serial-commanded, default-disarmed chassis diagnostic. */
esp_err_t robot_chassis_diag_start(void);

typedef struct {
    int64_t sampled_at_us;
    float x_m;
    float y_m;
    float yaw_rad;
    float linear_x_mps;
    float angular_z_radps;
} robot_chassis_odom_snapshot_t;

/* Thread-safe encoder-derived odometry snapshot for the micro-ROS task. */
bool robot_chassis_diag_get_odom_snapshot(
    robot_chassis_odom_snapshot_t *snapshot);

/* Thread-safe command ingress for a micro-ROS executor task.  Commands are
 * consumed only by the 10 ms chassis task; callers never touch motor state. */
bool robot_chassis_diag_remote_zero(void);
bool robot_chassis_diag_remote_arm(void);
bool robot_chassis_diag_remote_stop(void);
bool robot_chassis_diag_submit_cmd_vel(float linear_x_mps,
                                       float angular_z_radps);

#ifdef __cplusplus
}
#endif
