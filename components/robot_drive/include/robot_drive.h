#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "robot_drive_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROBOT_DRIVE_STOP_NONE = 0,
    ROBOT_DRIVE_STOP_STARTUP,
    ROBOT_DRIVE_STOP_COMMAND_TIMEOUT,
    ROBOT_DRIVE_STOP_INVALID_COMMAND,
    ROBOT_DRIVE_STOP_EXTERNAL_REQUEST,
    ROBOT_DRIVE_STOP_CORE_ERROR
} robot_drive_stop_reason_t;

typedef struct {
    float target_mps[ROBOT_DRIVE_WHEEL_COUNT];
    int64_t command_received_at_us;
    int64_t evaluated_at_us;
    uint32_t command_sequence;
    robot_drive_stop_reason_t stop_reason;
    bool stopped;
    bool input_fault;
    bool target_limited;
    bool dry_run;
} robot_drive_snapshot_t;

/* Starts an algorithm-only task. It never configures or writes motor GPIO. */
esp_err_t robot_drive_start(void);

/* Timestamp is captured inside this function. Invalid values enqueue a stop
 * request and return false, so a previous command is not kept as motion input. */
bool robot_drive_submit_cmd_vel(float linear_x_mps, float angular_z_radps);

/* ROBOT_DRIVE_STOP_NONE is rejected. A later valid command may resume dry-run. */
bool robot_drive_request_stop(robot_drive_stop_reason_t reason);

/* Returns the newest whole snapshot from a FreeRTOS length-one queue. */
bool robot_drive_get_snapshot(robot_drive_snapshot_t *snapshot);

const char *robot_drive_stop_reason_string(robot_drive_stop_reason_t reason);

#ifdef __cplusplus
}
#endif
