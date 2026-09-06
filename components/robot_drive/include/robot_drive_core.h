#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROBOT_DRIVE_WHEEL_LF = 0,
    ROBOT_DRIVE_WHEEL_LR,
    ROBOT_DRIVE_WHEEL_RF,
    ROBOT_DRIVE_WHEEL_RR,
    ROBOT_DRIVE_WHEEL_COUNT
} robot_drive_wheel_t;

typedef struct {
    /* Effective turn coefficient; not a measured chassis half-track. */
    float turn_geometry_m;
    float max_target_mps;
} robot_drive_core_config_t;

typedef struct {
    float linear_x_mps;
    float angular_z_radps;
} robot_drive_core_input_t;

typedef struct {
    float target_mps[ROBOT_DRIVE_WHEEL_COUNT];
    bool limited;
} robot_drive_core_output_t;

typedef enum {
    ROBOT_DRIVE_CORE_OK = 0,
    ROBOT_DRIVE_CORE_INVALID_ARGUMENT,
    ROBOT_DRIVE_CORE_INVALID_CONFIG,
    ROBOT_DRIVE_CORE_INVALID_COMMAND,
    ROBOT_DRIVE_CORE_NUMERIC_ERROR
} robot_drive_core_status_t;

/* Pure C: no ROS, FreeRTOS, ESP-IDF peripheral, motor or encoder dependency. */
robot_drive_core_status_t robot_drive_core_compute_targets(
    const robot_drive_core_config_t *config,
    const robot_drive_core_input_t *input,
    robot_drive_core_output_t *output);

void robot_drive_core_zero_output(robot_drive_core_output_t *output);

#ifdef __cplusplus
}
#endif
