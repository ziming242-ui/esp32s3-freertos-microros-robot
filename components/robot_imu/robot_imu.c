#include "robot_imu.h"

#include <stddef.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if defined(CONFIG_ROBOT_IMU_BACKEND_ICM42670P)
#include "robot_imu_icm42670p_backend.h"
#endif

static const char *TAG = "robot_imu";

/* A length-one queue gives the pipeline coherent "latest sample" semantics. */
static QueueHandle_t s_latest_sample_queue;
static uint32_t s_sample_sequence;
static bool s_started;

#if defined(CONFIG_ROBOT_IMU_BACKEND_SYNTHETIC)
static void synthetic_imu_task(void *arg)
{
    (void)arg;

    /* Deterministic SI-unit test data; this is not a physical measurement. */
    const float acceleration_mps2[3] = {0.0f, 0.0f, 9.80665f};
    const float angular_velocity_rad_s[3] = {0.0f, 0.0f, 0.0f};
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t period_ticks =
        pdMS_TO_TICKS(CONFIG_ROBOT_IMU_SYNTHETIC_PERIOD_MS);

    if (period_ticks == 0) {
        period_ticks = 1;
    }

    while (true) {
        const esp_err_t rc = robot_imu_submit_si_sample(
            acceleration_mps2,
            angular_velocity_rad_s,
            esp_timer_get_time());
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "synthetic sample submission failed: %s",
                     esp_err_to_name(rc));
            break;
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }

    vTaskDelete(NULL);
}
#endif

esp_err_t robot_imu_start(void)
{
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }

#if defined(CONFIG_ROBOT_IMU_BACKEND_SYNTHETIC) || \
    defined(CONFIG_ROBOT_IMU_BACKEND_ICM42670P)
    s_latest_sample_queue = xQueueCreate(1, sizeof(robot_imu_sample_t));
    if (s_latest_sample_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
#endif

#if defined(CONFIG_ROBOT_IMU_BACKEND_SYNTHETIC)
    const BaseType_t created = xTaskCreate(
        synthetic_imu_task,
        "imu_synthetic",
        3072,
        NULL,
        4,
        NULL);
    if (created != pdPASS) {
        vQueueDelete(s_latest_sample_queue);
        s_latest_sample_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGW(TAG,
             "synthetic IMU enabled: period=%d ms; not sensor evidence",
             CONFIG_ROBOT_IMU_SYNTHETIC_PERIOD_MS);
#elif defined(CONFIG_ROBOT_IMU_BACKEND_ICM42670P)
    const esp_err_t err = robot_imu_icm42670p_backend_start();
    if (err != ESP_OK) {
        vQueueDelete(s_latest_sample_queue);
        s_latest_sample_queue = NULL;
        return err;
    }
    s_started = true;
    ESP_LOGI(TAG, "physical IMU backend enabled: icm42670p");
#else
    s_started = true;
    ESP_LOGI(TAG, "IMU backend disabled");
#endif

    return ESP_OK;
}

bool robot_imu_is_enabled(void)
{
#if defined(CONFIG_ROBOT_IMU_BACKEND_SYNTHETIC) || \
    defined(CONFIG_ROBOT_IMU_BACKEND_ICM42670P)
    return true;
#else
    return false;
#endif
}

const char *robot_imu_backend_name(void)
{
#if defined(CONFIG_ROBOT_IMU_BACKEND_SYNTHETIC)
    return "synthetic";
#elif defined(CONFIG_ROBOT_IMU_BACKEND_ICM42670P)
    return "icm42670p";
#else
    return "disabled";
#endif
}

const char *robot_imu_frame_id(void)
{
#if defined(CONFIG_ROBOT_IMU_BACKEND_ICM42670P)
    return "icm42670p_link";
#else
    return "imu_frame";
#endif
}

int64_t robot_imu_stale_timeout_us(void)
{
    return (int64_t)CONFIG_ROBOT_IMU_STALE_TIMEOUT_MS * 1000;
}

esp_err_t robot_imu_submit_si_sample(
    const float linear_acceleration_mps2[3],
    const float angular_velocity_rad_s[3],
    int64_t sampled_at_us)
{
    if (linear_acceleration_mps2 == NULL ||
        angular_velocity_rad_s == NULL ||
        sampled_at_us < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_latest_sample_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t candidate_sequence = s_sample_sequence + 1U;
    if (candidate_sequence == 0U) {
        candidate_sequence = 1U;
    }

    robot_imu_sample_t candidate = {
        .sampled_at_us = sampled_at_us,
        .sample_sequence = candidate_sequence,
    };

    for (size_t axis = 0; axis < 3; ++axis) {
        candidate.linear_acceleration_mps2[axis] =
            linear_acceleration_mps2[axis];
        candidate.angular_velocity_rad_s[axis] =
            angular_velocity_rad_s[axis];
    }

    if (!robot_imu_sample_values_are_finite(&candidate)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueOverwrite(s_latest_sample_queue, &candidate) != pdPASS) {
        return ESP_FAIL;
    }

    /* Commit the producer state only after the complete snapshot is visible. */
    s_sample_sequence = candidate.sample_sequence;
    return ESP_OK;
}

bool robot_imu_get_latest(robot_imu_sample_t *out_sample)
{
    if (out_sample == NULL || s_latest_sample_queue == NULL) {
        return false;
    }

    return xQueuePeek(s_latest_sample_queue, out_sample, 0) == pdPASS;
}

esp_err_t robot_imu_request_test_pause(uint32_t duration_ms)
{
#if defined(CONFIG_ROBOT_IMU_BACKEND_ICM42670P)
    return robot_imu_icm42670p_backend_request_pause(duration_ms);
#else
    (void)duration_ms;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
