#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#if defined(CONFIG_ROBOT_APP_MODE_CHASSIS_DIAG)
#include "robot_chassis_diag.h"
#elif defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
#include "robot_chassis_diag.h"
#include "robot_imu.h"
#include "robot_microros.h"
#include "robot_wifi.h"
#elif defined(CONFIG_ROBOT_APP_MODE_ENCODER_PROBE)
#include "robot_encoder_probe.h"
#else
#include "robot_drive.h"
#include "robot_imu.h"
#include "robot_microros.h"
#include "robot_wifi.h"
#endif

static const char *TAG = "main";

/* Yahboom MicroROS Robot board: GPIO46 drives the active-high buzzer. */
static esp_err_t board_buzzer_force_off(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << GPIO_NUM_46,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    /* Preload the output latch before enabling the output driver, then assert
     * the safe level again after configuration. */
    esp_err_t err = gpio_set_level(GPIO_NUM_46, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_config(&config);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_set_level(GPIO_NUM_46, 0);
}

#if !defined(CONFIG_ROBOT_APP_MODE_ENCODER_PROBE) && \
    !defined(CONFIG_ROBOT_APP_MODE_CHASSIS_DIAG)
static void stop_after_startup_failure(const char *stage, esp_err_t err)
{
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    const bool stop_queued = robot_chassis_diag_remote_stop();
#else
    const bool stop_queued = robot_drive_request_stop(
        ROBOT_DRIVE_STOP_EXTERNAL_REQUEST);
#endif
    ESP_LOGE(TAG,
             "%s failed: %s; motion stop request %s",
             stage,
             esp_err_to_name(err),
             stop_queued ? "queued" : "not queued");
}
#endif

void app_main(void)
{
    esp_err_t err = board_buzzer_force_off();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to force buzzer off: %s; startup halted",
                 esp_err_to_name(err));
        return;
    }

#if defined(CONFIG_ROBOT_APP_MODE_CHASSIS_DIAG)
    ESP_LOGW(TAG, "starting default-disarmed chassis diagnostic");
    err = robot_chassis_diag_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "chassis diagnostic start failed: %s",
                 esp_err_to_name(err));
    }
    return;
#elif defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    ESP_LOGW(TAG,
             "starting default-disarmed micro-ROS chassis integration");
    err = robot_chassis_diag_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "chassis backend start failed: %s",
                 esp_err_to_name(err));
        return;
    }

    err = robot_imu_start();
    if (err != ESP_OK) {
        stop_after_startup_failure("IMU start", err);
        return;
    }
    err = robot_wifi_connect();
    if (err != ESP_OK) {
        stop_after_startup_failure("Wi-Fi connection", err);
        return;
    }
    ESP_LOGI(TAG, "network ready; starting micro-ROS motion bridge");
    err = robot_microros_start();
    if (err != ESP_OK) {
        stop_after_startup_failure("micro-ROS start", err);
    }
    return;
#elif defined(CONFIG_ROBOT_APP_MODE_ENCODER_PROBE)
    ESP_LOGW(TAG, "starting read-only encoder probe; motor power must remain disconnected");
    err = robot_encoder_probe_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "encoder probe start failed: %s", esp_err_to_name(err));
    }
    return;
#else
    ESP_LOGI(TAG, "starting drive dry-run service");
    err = robot_drive_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "drive dry-run start failed: %s; startup halted",
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "starting IMU pipeline");
    err = robot_imu_start();
    if (err != ESP_OK) {
        stop_after_startup_failure("IMU start", err);
        return;
    }

    ESP_LOGI(TAG, "starting Wi-Fi station");
    err = robot_wifi_connect();
    if (err != ESP_OK) {
        stop_after_startup_failure("Wi-Fi connection", err);
        return;
    }
    ESP_LOGI(TAG, "network ready");

    ESP_LOGI(TAG, "starting micro-ROS publisher");
    err = robot_microros_start();
    if (err != ESP_OK) {
        stop_after_startup_failure("micro-ROS start", err);
        return;
    }
#endif
}
