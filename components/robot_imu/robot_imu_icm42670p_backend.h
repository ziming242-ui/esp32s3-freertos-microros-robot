#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t robot_imu_icm42670p_backend_start(void);
esp_err_t robot_imu_icm42670p_backend_request_pause(uint32_t duration_ms);
