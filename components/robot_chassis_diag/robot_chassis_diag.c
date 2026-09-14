#include "robot_chassis_diag.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "robot_chassis_diag_core.h"
#include "robot_imu.h"

#define CONTROL_PERIOD_MS 10
#define TELEMETRY_PERIOD_STEPS 10
#define TIMING_REPORT_STEPS 100
#define ENCODER_COUNT_LIMIT 30000
#define PWM_RESOLUTION_HZ 10000000
#define PWM_FREQUENCY_HZ 25000
#define PWM_PERIOD_TICKS (PWM_RESOLUTION_HZ / PWM_FREQUENCY_HZ)
#define PWM_VENDOR_DEAD_ZONE 200
#define PWM_EXTRA_HARD_LIMIT 100
#define TARGET_COUNT_HARD_LIMIT 20
#define COMMAND_TIMEOUT_MS 500
#define ENCODER_STALL_STEPS 100
#define GAIN_KP_HARD_LIMIT 10.0f
#define GAIN_KI_HARD_LIMIT 1.0f
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12
#define BATTERY_DIVIDER_RATIO 4.03f
#define BATTERY_ARM_MIN_V 6.0f
#define BATTERY_ARM_MAX_V 12.0f
#define COUNTS_PER_REV_CANDIDATE 1040.0f
#define WHEEL_CIRCUMFERENCE_M_CANDIDATE 0.1508f
#define TURN_GEOMETRY_M_CANDIDATE 0.115f
#define REMOTE_QUEUE_LENGTH 8

typedef struct {
    int edge_gpio;
    int level_gpio;
} encoder_pins_t;

typedef struct {
    int pwma_gpio;
    int pwmb_gpio;
    int group_id;
} motor_pins_t;

/* M3/M4 PCNT edge inputs are swapped to preserve the verified vendor sign. */
static const encoder_pins_t s_encoder_pins[ROBOT_CHASSIS_WHEEL_COUNT] = {
    {6, 7}, {47, 48}, {12, 11}, {2, 1},
};

/* Same MCPWM pin ordering as the vendor twist_subscriber sample. */
static const motor_pins_t s_motor_pins[ROBOT_CHASSIS_WHEEL_COUNT] = {
    {5, 4, 0}, {16, 15, 0}, {9, 10, 0}, {13, 14, 1},
};

static const char *TAG = "chassis_diag";
static pcnt_unit_handle_t s_encoder_units[ROBOT_CHASSIS_WHEEL_COUNT];
static bdc_motor_handle_t s_motors[ROBOT_CHASSIS_WHEEL_COUNT];
static robot_chassis_core_t s_core;
static int s_last_counts[ROBOT_CHASSIS_WHEEL_COUNT];
static portMUX_TYPE s_odom_lock = portMUX_INITIALIZER_UNLOCKED;
static robot_chassis_odom_t s_odom;
static int64_t s_odom_sampled_at_us;
static adc_oneshot_unit_handle_t s_battery_adc;
static adc_cali_handle_t s_battery_cali;
static bool s_battery_calibrated;
static QueueHandle_t s_remote_queue;

typedef enum {
    REMOTE_EVENT_ZERO = 0,
    REMOTE_EVENT_ARM,
    REMOTE_EVENT_STOP,
    REMOTE_EVENT_TARGETS,
} remote_event_type_t;

typedef struct {
    remote_event_type_t type;
    int targets[ROBOT_CHASSIS_WHEEL_COUNT];
    bool limited;
} remote_event_t;

typedef struct {
    int raw;
    int adc_mv;
    float battery_v;
    bool calibrated;
} battery_sample_t;

static esp_err_t init_battery_adc(void)
{
    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_config, &s_battery_adc);
    if (err != ESP_OK) return err;

    const adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = BATTERY_ADC_ATTEN,
    };
    err = adc_oneshot_config_channel(s_battery_adc, BATTERY_ADC_CHANNEL,
                                     &channel_config);
    if (err != ESP_OK) return err;

    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&calibration_config,
                                                &s_battery_cali);
    if (err == ESP_OK) {
        s_battery_calibrated = true;
        return ESP_OK;
    }
    s_battery_cali = NULL;
    s_battery_calibrated = false;
    ESP_LOGW(TAG, "battery ADC calibration unavailable: %s",
             esp_err_to_name(err));
    return ESP_OK;
}

static esp_err_t read_battery(battery_sample_t *sample)
{
    if ((sample == NULL) || (s_battery_adc == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }
    sample->raw = -1;
    sample->adc_mv = -1;
    sample->battery_v = -1.0f;
    sample->calibrated = s_battery_calibrated;

    esp_err_t err = adc_oneshot_read(s_battery_adc, BATTERY_ADC_CHANNEL,
                                     &sample->raw);
    if (err != ESP_OK) return err;
    if (!s_battery_calibrated) return ESP_OK;

    err = adc_cali_raw_to_voltage(s_battery_cali, sample->raw,
                                  &sample->adc_mv);
    if (err != ESP_OK) return err;
    sample->battery_v = ((float)sample->adc_mv / 1000.0f) *
                        BATTERY_DIVIDER_RATIO;
    return ESP_OK;
}

static bool print_battery_status(void)
{
    battery_sample_t sample;
    const esp_err_t err = read_battery(&sample);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BAT result=ERROR err=%s", esp_err_to_name(err));
        return false;
    }
    const bool arm_safe = sample.calibrated &&
                          (sample.battery_v >= BATTERY_ARM_MIN_V) &&
                          (sample.battery_v <= BATTERY_ARM_MAX_V);
    ESP_LOGW(TAG,
        "BAT raw=%d adc_mv=%d battery_v=%.2f calibrated=%d arm_safe=%d",
        sample.raw, sample.adc_mv, (double)sample.battery_v,
        sample.calibrated ? 1 : 0, arm_safe ? 1 : 0);
    return arm_safe;
}

static esp_err_t init_encoder(size_t index)
{
    const pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_COUNT_LIMIT,
        .low_limit = -ENCODER_COUNT_LIMIT,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &s_encoder_units[index]);
    if (err != ESP_OK) return err;

    const pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    err = pcnt_unit_set_glitch_filter(s_encoder_units[index], &filter);
    if (err != ESP_OK) return err;

    const pcnt_chan_config_t a_config = {
        .edge_gpio_num = s_encoder_pins[index].edge_gpio,
        .level_gpio_num = s_encoder_pins[index].level_gpio,
    };
    pcnt_channel_handle_t channel_a = NULL;
    err = pcnt_new_channel(s_encoder_units[index], &a_config, &channel_a);
    if (err != ESP_OK) return err;

    const pcnt_chan_config_t b_config = {
        .edge_gpio_num = s_encoder_pins[index].level_gpio,
        .level_gpio_num = s_encoder_pins[index].edge_gpio,
    };
    pcnt_channel_handle_t channel_b = NULL;
    err = pcnt_new_channel(s_encoder_units[index], &b_config, &channel_b);
    if (err != ESP_OK) return err;

    err = pcnt_channel_set_edge_action(channel_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK) return err;
    err = pcnt_channel_set_level_action(channel_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK) return err;
    err = pcnt_channel_set_edge_action(channel_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (err != ESP_OK) return err;
    err = pcnt_channel_set_level_action(channel_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK) return err;
    err = pcnt_unit_enable(s_encoder_units[index]);
    if (err != ESP_OK) return err;
    err = pcnt_unit_clear_count(s_encoder_units[index]);
    if (err != ESP_OK) return err;
    return pcnt_unit_start(s_encoder_units[index]);
}

static esp_err_t init_motor(size_t index)
{
    const bdc_motor_config_t config = {
        .pwm_freq_hz = PWM_FREQUENCY_HZ,
        .pwma_gpio_num = s_motor_pins[index].pwma_gpio,
        .pwmb_gpio_num = s_motor_pins[index].pwmb_gpio,
    };
    const bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = s_motor_pins[index].group_id,
        .resolution_hz = PWM_RESOLUTION_HZ,
    };
    esp_err_t err = bdc_motor_new_mcpwm_device(
        &config, &mcpwm_config, &s_motors[index]);
    if (err != ESP_OK) return err;
    err = bdc_motor_enable(s_motors[index]);
    if (err != ESP_OK) return err;
    return bdc_motor_coast(s_motors[index]);
}

static void stop_all(bool brake)
{
    for (size_t i = 0; i < ROBOT_CHASSIS_WHEEL_COUNT; ++i) {
        if (s_motors[i] != NULL) {
            const esp_err_t err = brake ? bdc_motor_brake(s_motors[i])
                                        : bdc_motor_coast(s_motors[i]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "M%u stop failed: %s", (unsigned)(i + 1),
                         esp_err_to_name(err));
            }
        }
    }
}

static int command_to_duty(int command)
{
    if (command == 0) return 0;
    int duty = PWM_VENDOR_DEAD_ZONE + abs(command);
    if (duty > PWM_VENDOR_DEAD_ZONE + PWM_EXTRA_HARD_LIMIT) {
        duty = PWM_VENDOR_DEAD_ZONE + PWM_EXTRA_HARD_LIMIT;
    }
    if (duty >= PWM_PERIOD_TICKS) duty = PWM_PERIOD_TICKS - 1;
    return duty;
}

static bool control_setpoints_are_zero(void)
{
    for (size_t i = 0; i < ROBOT_CHASSIS_WHEEL_COUNT; ++i) {
        if ((s_core.target_counts[i] != 0) || (s_core.output[i] != 0)) {
            return false;
        }
    }
    return true;
}

static void apply_outputs(void)
{
    for (size_t i = 0; i < ROBOT_CHASSIS_WHEEL_COUNT; ++i) {
        const int command = s_core.output[i];
        if ((s_core.state != ROBOT_CHASSIS_STATE_ARMED) ||
            (s_core.target_counts[i] == 0) || (command == 0)) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(bdc_motor_coast(s_motors[i]));
            continue;
        }
        const int duty = command_to_duty(command);
        if (command > 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(bdc_motor_forward(s_motors[i]));
        } else {
            ESP_ERROR_CHECK_WITHOUT_ABORT(bdc_motor_reverse(s_motors[i]));
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(bdc_motor_set_speed(s_motors[i], duty));
    }
}

static void print_status(const int counts[4], const int delta[4])
{
    const int duty[4] = {
        command_to_duty(s_core.output[0]),
        command_to_duty(s_core.output[1]),
        command_to_duty(s_core.output[2]),
        command_to_duty(s_core.output[3]),
    };
    robot_chassis_odom_snapshot_t odom = {0};
    (void)robot_chassis_diag_get_odom_snapshot(&odom);
    ESP_LOGI(TAG,
        "CTRL state=%s reason=%s target=%d,%d,%d,%d delta=%d,%d,%d,%d "
        "pwm=%d,%d,%d,%d duty=%d,%d,%d,%d "
        "count=%d,%d,%d,%d odom_x=%.4f odom_y=%.4f yaw=%.4f "
        "linear_x=%.4f angular_z=%.4f",
        robot_chassis_state_string(s_core.state),
        robot_chassis_stop_reason_string(s_core.stop_reason),
        s_core.target_counts[0], s_core.target_counts[1],
        s_core.target_counts[2], s_core.target_counts[3],
        delta[0], delta[1], delta[2], delta[3],
        s_core.output[0], s_core.output[1], s_core.output[2], s_core.output[3],
        duty[0], duty[1], duty[2], duty[3],
        counts[0], counts[1], counts[2], counts[3],
        (double)odom.x_m, (double)odom.y_m, (double)odom.yaw_rad,
        (double)odom.linear_x_mps, (double)odom.angular_z_radps);
}

static void print_config(void)
{
    ESP_LOGW(TAG,
        "CONFIG kp=%.3f ki_per_step=%.3f target_limit=%d "
        "pwm_extra_limit=%d duty_limit=%d/%d timeout_ms=%lu stall_steps=%u",
        (double)s_core.config.kp, (double)s_core.config.ki_per_step,
        s_core.config.max_target_counts, s_core.config.max_output,
        PWM_VENDOR_DEAD_ZONE + s_core.config.max_output, PWM_PERIOD_TICKS,
        (unsigned long)s_core.config.command_timeout_ms,
        (unsigned)s_core.config.stall_steps);
}

static void process_command(char *line, uint32_t now_ms)
{
    while ((*line == ' ') || (*line == '\t')) ++line;
    if ((strcmp(line, "ZERO") == 0) || (strcmp(line, "STOP") == 0)) {
        robot_chassis_core_accept_zero(&s_core, now_ms);
        stop_all(false);
        ESP_LOGW(TAG, "ACK ZERO outputs=0");
        return;
    }
    if (strcmp(line, "ARM") == 0) {
        const bool battery_ok = print_battery_status();
        const bool ok = battery_ok && robot_chassis_core_arm(&s_core, now_ms);
        ESP_LOGW(TAG, "ACK ARM result=%s state=%s", ok ? "PASS" : "REJECT",
                 robot_chassis_state_string(s_core.state));
        return;
    }
    if (strcmp(line, "CLEAR") == 0) {
        const bool ok = robot_chassis_core_clear_fault(&s_core);
        ESP_LOGW(TAG, "ACK CLEAR result=%s state=%s", ok ? "PASS" : "REJECT",
                 robot_chassis_state_string(s_core.state));
        return;
    }
    if (strcmp(line, "STATUS") == 0) {
        int counts[4] = {0};
        int delta[4] = {0};
        for (size_t i = 0; i < 4; ++i) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                pcnt_unit_get_count(s_encoder_units[i], &counts[i]));
        }
        print_status(counts, delta);
        print_config();
        print_battery_status();
        return;
    }
    if (strcmp(line, "BAT") == 0) {
        print_battery_status();
        return;
    }
    if (strcmp(line, "ODOMZERO") == 0) {
        const bool ok = s_core.state == ROBOT_CHASSIS_STATE_DISARMED;
        if (ok) {
            taskENTER_CRITICAL(&s_odom_lock);
            s_odom = (robot_chassis_odom_t){0};
            s_odom_sampled_at_us = esp_timer_get_time();
            taskEXIT_CRITICAL(&s_odom_lock);
        }
        ESP_LOGW(TAG, "ACK ODOMZERO result=%s state=%s",
                 ok ? "PASS" : "REJECT",
                 robot_chassis_state_string(s_core.state));
        return;
    }

    unsigned int imu_pause_ms = 0;
    char imu_pause_extra = '\0';
    if (sscanf(line, "IMUPAUSE %u %c",
               &imu_pause_ms, &imu_pause_extra) == 1) {
        const bool safe_state =
            (s_core.state == ROBOT_CHASSIS_STATE_DISARMED) &&
            control_setpoints_are_zero();
        if (safe_state) {
            /* Reassert the already-safe physical output state before pausing. */
            stop_all(false);
        }
        const esp_err_t pause_err = safe_state
            ? robot_imu_request_test_pause((uint32_t)imu_pause_ms)
            : ESP_ERR_INVALID_STATE;
        ESP_LOGW(TAG,
                 "ACK IMUPAUSE duration_ms=%u result=%s state=%s "
                 "target=%d,%d,%d,%d pwm=%d,%d,%d,%d duty=%d,%d,%d,%d "
                 "coast_reasserted=%d",
                 imu_pause_ms,
                 pause_err == ESP_OK ? "PASS" : esp_err_to_name(pause_err),
                 robot_chassis_state_string(s_core.state),
                 s_core.target_counts[0], s_core.target_counts[1],
                 s_core.target_counts[2], s_core.target_counts[3],
                 s_core.output[0], s_core.output[1],
                 s_core.output[2], s_core.output[3],
                 command_to_duty(s_core.output[0]),
                 command_to_duty(s_core.output[1]),
                 command_to_duty(s_core.output[2]),
                 command_to_duty(s_core.output[3]),
                 safe_state ? 1 : 0);
        return;
    }

    float kp = -1.0f;
    float ki = -1.0f;
    char gain_extra = '\0';
    if (sscanf(line, "GAIN %f %f %c", &kp, &ki, &gain_extra) == 2) {
        const bool within_limits = (kp >= 0.0f) &&
                                   (kp <= GAIN_KP_HARD_LIMIT) &&
                                   (ki >= 0.0f) &&
                                   (ki <= GAIN_KI_HARD_LIMIT);
        const bool ok = within_limits &&
                        robot_chassis_core_update_gains(&s_core, kp, ki);
        ESP_LOGW(TAG, "ACK GAIN result=%s state=%s kp=%.3f ki=%.3f",
                 ok ? "PASS" : "REJECT",
                 robot_chassis_state_string(s_core.state),
                 (double)s_core.config.kp,
                 (double)s_core.config.ki_per_step);
        return;
    }

    int wheel = -1;
    int target = INT_MAX;
    char extra = '\0';
    if ((sscanf(line, "RUN %d %d %c", &wheel, &target, &extra) == 2) &&
        (wheel >= 0) && (wheel <= 4)) {
        int targets[4] = {0};
        if (wheel == 0) {
            for (size_t i = 0; i < 4; ++i) targets[i] = target;
        } else {
            targets[wheel - 1] = target;
        }
        const bool ok = robot_chassis_core_submit(&s_core, targets, now_ms);
        if (!ok) stop_all(true);
        ESP_LOGW(TAG, "ACK RUN wheel=%d target=%d result=%s state=%s",
                 wheel, target, ok ? "PASS" : "REJECT",
                 robot_chassis_state_string(s_core.state));
        return;
    }

    robot_chassis_core_stop(&s_core, ROBOT_CHASSIS_STOP_INVALID_TARGET);
    stop_all(true);
    ESP_LOGE(TAG, "ACK INVALID result=FAULT_LATCHED");
}

static void poll_uart(uint32_t now_ms)
{
    static char line[64];
    static size_t used;
    uint8_t data[32];
    const int received = uart_read_bytes(UART_NUM_0, data, sizeof(data), 0);
    for (int i = 0; i < received; ++i) {
        const char ch = (char)data[i];
        if ((ch == '\r') || (ch == '\n')) {
            if (used > 0) {
                line[used] = '\0';
                process_command(line, now_ms);
                used = 0;
            }
        } else if (used + 1 < sizeof(line)) {
            line[used++] = ch;
        } else {
            used = 0;
            robot_chassis_core_stop(&s_core,
                                    ROBOT_CHASSIS_STOP_INVALID_TARGET);
            stop_all(true);
            ESP_LOGE(TAG, "UART command overflow -> FAULT_LATCHED");
        }
    }
}

static void process_remote_events(uint32_t now_ms)
{
    remote_event_t event;
    while ((s_remote_queue != NULL) &&
           (xQueueReceive(s_remote_queue, &event, 0) == pdPASS)) {
        switch (event.type) {
        case REMOTE_EVENT_ZERO:
            robot_chassis_core_accept_zero(&s_core, now_ms);
            stop_all(false);
            ESP_LOGW(TAG, "ACK REMOTE ZERO state=%s",
                     robot_chassis_state_string(s_core.state));
            break;
        case REMOTE_EVENT_ARM: {
            const bool battery_ok = print_battery_status();
            const bool ok = battery_ok &&
                            robot_chassis_core_arm(&s_core, now_ms);
            ESP_LOGW(TAG, "ACK REMOTE ARM result=%s state=%s",
                     ok ? "PASS" : "REJECT",
                     robot_chassis_state_string(s_core.state));
            break;
        }
        case REMOTE_EVENT_STOP:
            robot_chassis_core_stop(&s_core, ROBOT_CHASSIS_STOP_EXPLICIT);
            stop_all(true);
            ESP_LOGW(TAG, "ACK REMOTE STOP state=%s",
                     robot_chassis_state_string(s_core.state));
            break;
        case REMOTE_EVENT_TARGETS: {
            const bool ok = robot_chassis_core_submit(
                &s_core, event.targets, now_ms);
            if (!ok) stop_all(true);
            ESP_LOGW(TAG,
                "ACK REMOTE CMD_VEL result=%s state=%s limited=%d "
                "target=%d,%d,%d,%d",
                ok ? "PASS" : "REJECT",
                robot_chassis_state_string(s_core.state),
                event.limited ? 1 : 0,
                event.targets[0], event.targets[1],
                event.targets[2], event.targets[3]);
            break;
        }
        default:
            robot_chassis_core_stop(
                &s_core, ROBOT_CHASSIS_STOP_INVALID_TARGET);
            stop_all(true);
            ESP_LOGE(TAG, "invalid remote event -> FAULT_LATCHED");
            break;
        }
    }
}

static void control_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    int64_t prior_us = 0;
    int64_t sum_period_us = 0;
    int32_t min_period_us = INT32_MAX;
    int32_t max_period_us = 0;
    uint32_t timing_samples = 0;
    uint32_t step = 0;

    for (;;) {
        const int64_t now_us = esp_timer_get_time();
        const uint32_t now_ms = (uint32_t)(now_us / 1000);
        int32_t period_us = CONTROL_PERIOD_MS * 1000;
        if (prior_us != 0) {
            period_us = (int32_t)(now_us - prior_us);
            if (period_us < min_period_us) min_period_us = period_us;
            if (period_us > max_period_us) max_period_us = period_us;
            sum_period_us += period_us;
            ++timing_samples;
        }
        prior_us = now_us;

        poll_uart(now_ms);
        process_remote_events(now_ms);
        int counts[4] = {0};
        int delta[4] = {0};
        bool encoder_ok = true;
        for (size_t i = 0; i < 4; ++i) {
            if (pcnt_unit_get_count(s_encoder_units[i], &counts[i]) != ESP_OK) {
                encoder_ok = false;
                break;
            }
            delta[i] = robot_chassis_safe_count_delta(
                counts[i], s_last_counts[i], ENCODER_COUNT_LIMIT);
            s_last_counts[i] = counts[i];
        }
        if (!encoder_ok) {
            robot_chassis_core_stop(&s_core,
                                    ROBOT_CHASSIS_STOP_ENCODER_STALL);
            stop_all(true);
            ESP_LOGE(TAG, "PCNT read error -> FAULT_LATCHED");
        } else {
            robot_chassis_core_step(&s_core, delta, now_ms);
            if ((s_core.state == ROBOT_CHASSIS_STATE_FAULT_LATCHED) &&
                (s_core.stop_reason == ROBOT_CHASSIS_STOP_ENCODER_STALL)) {
                stop_all(true);
            } else {
                apply_outputs();
            }
            robot_chassis_odom_t next_odom = s_odom;
            const float metres_per_count = WHEEL_CIRCUMFERENCE_M_CANDIDATE /
                                           COUNTS_PER_REV_CANDIDATE;
            if (robot_chassis_odom_update(
                    &next_odom, delta, (float)period_us / 1000000.0f,
                    metres_per_count, TURN_GEOMETRY_M_CANDIDATE)) {
                taskENTER_CRITICAL(&s_odom_lock);
                s_odom = next_odom;
                s_odom_sampled_at_us = now_us;
                taskEXIT_CRITICAL(&s_odom_lock);
            }
        }

        ++step;
        if ((step % TELEMETRY_PERIOD_STEPS) == 0) {
            print_status(counts, delta);
        }
        if ((step % TIMING_REPORT_STEPS) == 0 && timing_samples > 0) {
            const int64_t mean = sum_period_us / timing_samples;
            ESP_LOGI(TAG,
                "TIMING samples=%lu target_us=10000 min_us=%ld mean_us=%lld max_us=%ld",
                (unsigned long)timing_samples, (long)min_period_us,
                (long long)mean, (long)max_period_us);
            sum_period_us = 0;
            min_period_us = INT32_MAX;
            max_period_us = 0;
            timing_samples = 0;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

esp_err_t robot_chassis_diag_start(void)
{
    const robot_chassis_core_config_t config = {
        .kp = 5.0f,
        .ki_per_step = 0.25f,
        .integral_limit = 40.0f,
        .max_output = PWM_EXTRA_HARD_LIMIT,
        .max_target_counts = TARGET_COUNT_HARD_LIMIT,
        .command_timeout_ms = COMMAND_TIMEOUT_MS,
        .stall_steps = ENCODER_STALL_STEPS,
    };
    if (!robot_chassis_core_init(&s_core, &config)) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_odom_lock);
    s_odom = (robot_chassis_odom_t){0};
    s_odom_sampled_at_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_odom_lock);

    s_remote_queue = xQueueCreate(REMOTE_QUEUE_LENGTH, sizeof(remote_event_t));
    if (s_remote_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = init_battery_adc();
    if (err != ESP_OK) return err;

    for (size_t i = 0; i < ROBOT_CHASSIS_WHEEL_COUNT; ++i) {
        err = init_encoder(i);
        if (err != ESP_OK) {
            stop_all(false);
            return err;
        }
    }
    for (size_t i = 0; i < ROBOT_CHASSIS_WHEEL_COUNT; ++i) {
        err = init_motor(i);
        if (err != ESP_OK) {
            stop_all(false);
            return err;
        }
    }
    stop_all(false);

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(UART_NUM_0, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) return err;

    ESP_LOGW(TAG, "CHASSIS_DIAG READY default=DISARMED outputs=0");
    ESP_LOGW(TAG,
        "HARD_LIMIT target=+-%d counts/10ms pwm_extra=+-%d actual_tick<=%d/%d timeout=%dms",
        TARGET_COUNT_HARD_LIMIT, PWM_EXTRA_HARD_LIMIT,
        PWM_VENDOR_DEAD_ZONE + PWM_EXTRA_HARD_LIMIT, PWM_PERIOD_TICKS,
        COMMAND_TIMEOUT_MS);
    print_config();
    print_battery_status();
    ESP_LOGW(TAG,
        "COMMANDS: ZERO | ARM | RUN <0..4> <-20..20> | STOP | CLEAR | "
        "STATUS | BAT | GAIN <0..10> <0..1> | ODOMZERO | "
        "IMUPAUSE <101..2000>");

    const BaseType_t created = xTaskCreate(
        control_task, "chassis_diag", 6144, NULL, 8, NULL);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

bool robot_chassis_diag_get_odom_snapshot(
    robot_chassis_odom_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    taskENTER_CRITICAL(&s_odom_lock);
    snapshot->sampled_at_us = s_odom_sampled_at_us;
    snapshot->x_m = s_odom.x_m;
    snapshot->y_m = s_odom.y_m;
    snapshot->yaw_rad = s_odom.yaw_rad;
    snapshot->linear_x_mps = s_odom.linear_x_mps;
    snapshot->angular_z_radps = s_odom.angular_z_radps;
    taskEXIT_CRITICAL(&s_odom_lock);
    return snapshot->sampled_at_us > 0;
}

static bool queue_remote_event(const remote_event_t *event)
{
    return (event != NULL) && (s_remote_queue != NULL) &&
           (xQueueSend(s_remote_queue, event, 0) == pdPASS);
}

bool robot_chassis_diag_remote_zero(void)
{
    const remote_event_t event = {.type = REMOTE_EVENT_ZERO};
    return queue_remote_event(&event);
}

bool robot_chassis_diag_remote_arm(void)
{
    const remote_event_t event = {.type = REMOTE_EVENT_ARM};
    return queue_remote_event(&event);
}

bool robot_chassis_diag_remote_stop(void)
{
    const remote_event_t event = {.type = REMOTE_EVENT_STOP};
    return queue_remote_event(&event);
}

bool robot_chassis_diag_submit_cmd_vel(float linear_x_mps,
                                       float angular_z_radps)
{
    remote_event_t event = {.type = REMOTE_EVENT_TARGETS};
    const float counts_per_metre_per_step =
        COUNTS_PER_REV_CANDIDATE * ((float)CONTROL_PERIOD_MS / 1000.0f) /
        WHEEL_CIRCUMFERENCE_M_CANDIDATE;
    if (!robot_chassis_core_twist_to_targets(
            linear_x_mps, angular_z_radps,
            TURN_GEOMETRY_M_CANDIDATE, counts_per_metre_per_step,
            TARGET_COUNT_HARD_LIMIT, event.targets, &event.limited)) {
        return false;
    }
    return queue_remote_event(&event);
}
