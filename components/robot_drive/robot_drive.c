#include "robot_drive.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "robot_drive";

typedef enum {
    ROBOT_DRIVE_EVENT_COMMAND = 0,
    ROBOT_DRIVE_EVENT_STOP
} robot_drive_event_type_t;

typedef struct {
    robot_drive_event_type_t type;
    robot_drive_core_input_t input;
    int64_t received_at_us;
    uint32_t sequence;
    robot_drive_stop_reason_t stop_reason;
    bool valid;
} robot_drive_event_t;

static QueueHandle_t s_event_queue;
static QueueHandle_t s_snapshot_queue;
static TaskHandle_t s_task;
static portMUX_TYPE s_sequence_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_next_sequence;

static uint32_t next_sequence(void)
{
    portENTER_CRITICAL(&s_sequence_lock);
    ++s_next_sequence;
    if (s_next_sequence == 0U) {
        ++s_next_sequence;
    }
    const uint32_t value = s_next_sequence;
    portEXIT_CRITICAL(&s_sequence_lock);
    return value;
}

static bool is_input_fault(robot_drive_stop_reason_t reason)
{
    return (reason == ROBOT_DRIVE_STOP_INVALID_COMMAND) ||
           (reason == ROBOT_DRIVE_STOP_CORE_ERROR);
}

static void make_stopped_snapshot(robot_drive_snapshot_t *snapshot,
                                  robot_drive_stop_reason_t reason,
                                  int64_t evaluated_at_us,
                                  int64_t command_received_at_us,
                                  uint32_t sequence)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->command_received_at_us = command_received_at_us;
    snapshot->evaluated_at_us = evaluated_at_us;
    snapshot->command_sequence = sequence;
    snapshot->stop_reason = reason;
    snapshot->stopped = true;
    snapshot->input_fault = is_input_fault(reason);
    snapshot->dry_run = true;
}

static void publish_snapshot(const robot_drive_snapshot_t *snapshot)
{
    if (xQueueOverwrite(s_snapshot_queue, snapshot) != pdPASS) {
        ESP_LOGE(TAG, "failed to overwrite dry-run snapshot");
    }
}

static void robot_drive_task(void *arg)
{
    (void)arg;

    const robot_drive_core_config_t config = {
        .turn_geometry_m = (float)CONFIG_ROBOT_DRIVE_TURN_GEOMETRY_MM / 1000.0f,
        .max_target_mps = (float)CONFIG_ROBOT_DRIVE_MAX_TARGET_MMPS / 1000.0f,
    };
    const int64_t timeout_us =
        (int64_t)CONFIG_ROBOT_DRIVE_COMMAND_TIMEOUT_MS * 1000;
    TickType_t period_ticks =
        pdMS_TO_TICKS(CONFIG_ROBOT_DRIVE_CONTROL_PERIOD_MS);
    if (period_ticks == 0) {
        period_ticks = 1;
    }
    TickType_t last_wake = xTaskGetTickCount();
    robot_drive_event_t active = {0};
    bool has_active_command = false;
    robot_drive_stop_reason_t stop_reason = ROBOT_DRIVE_STOP_STARTUP;
    robot_drive_stop_reason_t last_logged_reason = ROBOT_DRIVE_STOP_NONE;

    while (true) {
        robot_drive_event_t incoming;
        if (xQueueReceive(s_event_queue, &incoming, 0) == pdPASS) {
            active = incoming;
            if ((incoming.type == ROBOT_DRIVE_EVENT_COMMAND) && incoming.valid) {
                has_active_command = true;
                stop_reason = ROBOT_DRIVE_STOP_NONE;
            } else {
                has_active_command = false;
                stop_reason = (incoming.type == ROBOT_DRIVE_EVENT_STOP)
                    ? incoming.stop_reason : ROBOT_DRIVE_STOP_INVALID_COMMAND;
            }
        }

        const int64_t now_us = esp_timer_get_time();
        robot_drive_snapshot_t snapshot;
        if (!has_active_command) {
            make_stopped_snapshot(&snapshot, stop_reason, now_us,
                                  active.received_at_us, active.sequence);
        } else {
            const int64_t age_us = now_us - active.received_at_us;
            if ((age_us < 0) || (age_us > timeout_us)) {
                has_active_command = false;
                stop_reason = (age_us < 0)
                    ? ROBOT_DRIVE_STOP_CORE_ERROR
                    : ROBOT_DRIVE_STOP_COMMAND_TIMEOUT;
                make_stopped_snapshot(&snapshot, stop_reason, now_us,
                                      active.received_at_us, active.sequence);
            } else {
                robot_drive_core_output_t output;
                const robot_drive_core_status_t status =
                    robot_drive_core_compute_targets(&config, &active.input,
                                                     &output);
                if (status != ROBOT_DRIVE_CORE_OK) {
                    has_active_command = false;
                    stop_reason = (status == ROBOT_DRIVE_CORE_INVALID_COMMAND)
                        ? ROBOT_DRIVE_STOP_INVALID_COMMAND
                        : ROBOT_DRIVE_STOP_CORE_ERROR;
                    make_stopped_snapshot(&snapshot, stop_reason, now_us,
                                          active.received_at_us,
                                          active.sequence);
                } else {
                    memset(&snapshot, 0, sizeof(snapshot));
                    memcpy(snapshot.target_mps, output.target_mps,
                           sizeof(snapshot.target_mps));
                    snapshot.command_received_at_us = active.received_at_us;
                    snapshot.evaluated_at_us = now_us;
                    snapshot.command_sequence = active.sequence;
                    snapshot.stop_reason = ROBOT_DRIVE_STOP_NONE;
                    snapshot.target_limited = output.limited;
                    snapshot.dry_run = true;
                }
            }
        }
        publish_snapshot(&snapshot);

        if (snapshot.stop_reason != last_logged_reason) {
            ESP_LOGI(TAG, "dry-run stop state: %s",
                     robot_drive_stop_reason_string(snapshot.stop_reason));
            last_logged_reason = snapshot.stop_reason;
        }
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

esp_err_t robot_drive_start(void)
{
    if ((s_task != NULL) || (s_event_queue != NULL) ||
        (s_snapshot_queue != NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    s_event_queue = xQueueCreate(1, sizeof(robot_drive_event_t));
    s_snapshot_queue = xQueueCreate(1, sizeof(robot_drive_snapshot_t));
    if ((s_event_queue == NULL) || (s_snapshot_queue == NULL)) {
        if (s_event_queue != NULL) {
            vQueueDelete(s_event_queue);
        }
        if (s_snapshot_queue != NULL) {
            vQueueDelete(s_snapshot_queue);
        }
        s_event_queue = NULL;
        s_snapshot_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    robot_drive_snapshot_t initial;
    make_stopped_snapshot(&initial, ROBOT_DRIVE_STOP_STARTUP,
                          esp_timer_get_time(), 0, 0);
    publish_snapshot(&initial);

    const BaseType_t created = xTaskCreate(
        robot_drive_task,
        "robot_drive_dry",
        CONFIG_ROBOT_DRIVE_TASK_STACK,
        NULL,
        CONFIG_ROBOT_DRIVE_TASK_PRIORITY,
        &s_task);
    if (created != pdPASS) {
        vQueueDelete(s_event_queue);
        vQueueDelete(s_snapshot_queue);
        s_event_queue = NULL;
        s_snapshot_queue = NULL;
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGW(TAG,
             "dry-run only: no PWM, motor, encoder, PID or physical stop");
    return ESP_OK;
}

bool robot_drive_submit_cmd_vel(float linear_x_mps, float angular_z_radps)
{
    if (s_event_queue == NULL) {
        return false;
    }

    const bool valid = isfinite(linear_x_mps) &&
                       isfinite(angular_z_radps);
    const robot_drive_event_t event = {
        .type = ROBOT_DRIVE_EVENT_COMMAND,
        .input = {
            .linear_x_mps = linear_x_mps,
            .angular_z_radps = angular_z_radps,
        },
        .received_at_us = esp_timer_get_time(),
        .sequence = next_sequence(),
        .stop_reason = valid ? ROBOT_DRIVE_STOP_NONE
                             : ROBOT_DRIVE_STOP_INVALID_COMMAND,
        .valid = valid,
    };

    const bool queued = xQueueOverwrite(s_event_queue, &event) == pdPASS;
    return queued && valid;
}

bool robot_drive_request_stop(robot_drive_stop_reason_t reason)
{
    if ((s_event_queue == NULL) || (reason == ROBOT_DRIVE_STOP_NONE)) {
        return false;
    }

    const robot_drive_event_t event = {
        .type = ROBOT_DRIVE_EVENT_STOP,
        .received_at_us = esp_timer_get_time(),
        .sequence = next_sequence(),
        .stop_reason = reason,
        .valid = false,
    };
    return xQueueOverwrite(s_event_queue, &event) == pdPASS;
}

bool robot_drive_get_snapshot(robot_drive_snapshot_t *snapshot)
{
    if ((snapshot == NULL) || (s_snapshot_queue == NULL)) {
        return false;
    }
    return xQueuePeek(s_snapshot_queue, snapshot, 0) == pdPASS;
}

const char *robot_drive_stop_reason_string(robot_drive_stop_reason_t reason)
{
    switch (reason) {
    case ROBOT_DRIVE_STOP_NONE:
        return "none";
    case ROBOT_DRIVE_STOP_STARTUP:
        return "startup";
    case ROBOT_DRIVE_STOP_COMMAND_TIMEOUT:
        return "command-timeout";
    case ROBOT_DRIVE_STOP_INVALID_COMMAND:
        return "invalid-command";
    case ROBOT_DRIVE_STOP_EXTERNAL_REQUEST:
        return "external-request";
    case ROBOT_DRIVE_STOP_CORE_ERROR:
        return "core-error";
    default:
        return "unknown";
    }
}
