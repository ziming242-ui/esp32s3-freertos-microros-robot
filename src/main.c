#include "esp_log.h"
#include "robot_drive.h"
#include "robot_imu.h"
#include "robot_microros.h"
#include "robot_wifi.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "starting drive dry-run service");
    ESP_ERROR_CHECK(robot_drive_start());

    ESP_LOGI(TAG, "starting IMU pipeline");
    ESP_ERROR_CHECK(robot_imu_start());

    ESP_LOGI(TAG, "starting Wi-Fi station");
    ESP_ERROR_CHECK(robot_wifi_connect());
    ESP_LOGI(TAG, "network ready");

    ESP_LOGI(TAG, "starting micro-ROS publisher");
    ESP_ERROR_CHECK(robot_microros_start());
}
