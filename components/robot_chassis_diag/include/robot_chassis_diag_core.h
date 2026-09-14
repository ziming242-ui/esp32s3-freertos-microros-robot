#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_CHASSIS_WHEEL_COUNT 4

typedef enum {
    ROBOT_CHASSIS_STATE_DISARMED = 0,
    ROBOT_CHASSIS_STATE_ARMED,
    ROBOT_CHASSIS_STATE_FAULT_LATCHED
} robot_chassis_state_t;

typedef enum {
    ROBOT_CHASSIS_STOP_STARTUP = 0,
    ROBOT_CHASSIS_STOP_NONE,
    ROBOT_CHASSIS_STOP_EXPLICIT,
    ROBOT_CHASSIS_STOP_TIMEOUT,
    ROBOT_CHASSIS_STOP_INVALID_TARGET,
    ROBOT_CHASSIS_STOP_ENCODER_STALL
} robot_chassis_stop_reason_t;

typedef struct {
    float kp;
    float ki_per_step;
    float integral_limit;
    int max_output;
    int max_target_counts;
    uint32_t command_timeout_ms;
    uint16_t stall_steps;
} robot_chassis_core_config_t;

typedef struct {
    robot_chassis_core_config_t config;
    robot_chassis_state_t state;
    robot_chassis_stop_reason_t stop_reason;
    int target_counts[ROBOT_CHASSIS_WHEEL_COUNT];
    int output[ROBOT_CHASSIS_WHEEL_COUNT];
    float integral[ROBOT_CHASSIS_WHEEL_COUNT];
    uint16_t stall_count[ROBOT_CHASSIS_WHEEL_COUNT];
    uint32_t last_command_ms;
    bool zero_seen;
} robot_chassis_core_t;

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;
    float linear_x_mps;
    float angular_z_radps;
} robot_chassis_odom_t;

bool robot_chassis_core_init(robot_chassis_core_t *core,
                             const robot_chassis_core_config_t *config);
bool robot_chassis_core_update_gains(robot_chassis_core_t *core,
                                     float kp,
                                     float ki_per_step);
bool robot_chassis_core_twist_to_targets(
    float linear_x_mps,
    float angular_z_radps,
    float turn_geometry_m,
    float counts_per_metre_per_step,
    int max_target_counts,
    int target_counts[ROBOT_CHASSIS_WHEEL_COUNT],
    bool *limited);
/* Convert one bounded PCNT sample pair into a signed delta.
 * count_limit is the configured positive PCNT boundary. */
int robot_chassis_safe_count_delta(int current_count,
                                   int previous_count,
                                   int count_limit);
/* Integrate one four-wheel encoder sample as a differential-drive step.
 * turn_geometry_m is the distance from the centreline to either side. */
bool robot_chassis_odom_update(
    robot_chassis_odom_t *odom,
    const int count_delta[ROBOT_CHASSIS_WHEEL_COUNT],
    float dt_s,
    float metres_per_count,
    float turn_geometry_m);
void robot_chassis_core_accept_zero(robot_chassis_core_t *core,
                                    uint32_t now_ms);
bool robot_chassis_core_arm(robot_chassis_core_t *core, uint32_t now_ms);
bool robot_chassis_core_submit(robot_chassis_core_t *core,
                               const int target_counts[ROBOT_CHASSIS_WHEEL_COUNT],
                               uint32_t now_ms);
void robot_chassis_core_stop(robot_chassis_core_t *core,
                             robot_chassis_stop_reason_t reason);
bool robot_chassis_core_clear_fault(robot_chassis_core_t *core);
void robot_chassis_core_step(robot_chassis_core_t *core,
                             const int count_delta[ROBOT_CHASSIS_WHEEL_COUNT],
                             uint32_t now_ms);
const char *robot_chassis_state_string(robot_chassis_state_t state);
const char *robot_chassis_stop_reason_string(robot_chassis_stop_reason_t reason);

#ifdef __cplusplus
}
#endif
