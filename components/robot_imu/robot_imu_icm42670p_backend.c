#include "robot_imu_icm42670p_backend.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "robot_imu.h"
#include "robot_imu_icm42670p_core.h"

static const char *TAG = "imu_icm42670p";

enum {
    ICM42670P_SCL_GPIO = 39,
    ICM42670P_SDA_GPIO = 40,
    ICM42670P_I2C_CLOCK_HZ = 400000,
    ICM42670P_I2C_ADDRESS = 0x68,
    ICM42670P_I2C_TIMEOUT_MS = 100,

    ICM42670P_SIGNAL_PATH_RESET_REG = 0x02,
    ICM42670P_PWR_MGMT0_REG = 0x1F,
    ICM42670P_GYRO_CONFIG0_REG = 0x20,
    ICM42670P_ACCEL_CONFIG0_REG = 0x21,
    ICM42670P_INT_SOURCE0_REG = 0x2B,
    ICM42670P_INTF_CONFIG0_REG = 0x35,
    ICM42670P_INT_STATUS_DRDY_REG = 0x39,
    ICM42670P_INT_STATUS_REG = 0x3A,
    ICM42670P_WHO_AM_I_REG = 0x75,

    ICM42670P_SOFT_RESET_COMMAND = 0x10,
    ICM42670P_RESET_DONE_MASK = 0x10,
    ICM42670P_DRDY_INT1_ENABLE_MASK = 0x08,
    ICM42670P_SENSOR_DATA_BIG_ENDIAN_MASK = 0x10,
    ICM42670P_DATA_READY_MASK = 0x01,
    ICM42670P_WHO_AM_I_VALUE = 0x67,

    ICM42670P_GYRO_CONFIG0_VALUE = 0x07,
    ICM42670P_ACCEL_CONFIG0_VALUE = 0x47,
    ICM42670P_PWR_MGMT0_VALUE = 0x0F,

    ICM42670P_STARTUP_WAIT_MS = 60,
    ICM42670P_DATA_READY_TIMEOUT_MS = 100,
    ICM42670P_RUNTIME_LOG_PERIOD_MS = 1000,
    ICM42670P_TASK_STACK_BYTES = 4096,
    ICM42670P_TASK_PRIORITY = 6,
    ICM42670P_TEST_PAUSE_MIN_MS = 101,
    ICM42670P_TEST_PAUSE_MAX_MS = 2000,
};

typedef struct {
    uint32_t samples_ok;
    uint32_t i2c_errors;
    uint32_t protocol_errors;
    uint32_t data_ready_timeouts;
    uint32_t parse_errors;
    uint32_t submit_errors;
    uint32_t test_pauses;
} icm42670p_runtime_counters_t;

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_device;
static QueueHandle_t s_test_pause_queue;
static bool s_started;
static icm42670p_runtime_counters_t s_counters;

static esp_err_t read_registers(uint8_t start_register,
                                uint8_t *data,
                                size_t data_length)
{
    const esp_err_t err = i2c_master_transmit_receive(
        s_device,
        &start_register,
        sizeof(start_register),
        data,
        data_length,
        ICM42670P_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        s_counters.i2c_errors++;
    }
    return err;
}

static esp_err_t read_register(uint8_t register_address, uint8_t *value)
{
    return read_registers(register_address, value, 1);
}

static esp_err_t write_register(uint8_t register_address, uint8_t value)
{
    const uint8_t transaction[] = {register_address, value};
    const esp_err_t err = i2c_master_transmit(
        s_device,
        transaction,
        sizeof(transaction),
        ICM42670P_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        s_counters.i2c_errors++;
    }
    return err;
}

static bool write_and_verify(uint8_t register_address, uint8_t expected_value)
{
    uint8_t actual_value = 0;
    esp_err_t err = write_register(register_address, expected_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write failed reg=0x%02x error=%s",
                 register_address, esp_err_to_name(err));
        return false;
    }
    err = read_register(register_address, &actual_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "readback failed reg=0x%02x error=%s",
                 register_address, esp_err_to_name(err));
        return false;
    }
    if (actual_value != expected_value) {
        s_counters.protocol_errors++;
        ESP_LOGE(TAG,
                 "register verify FAIL reg=0x%02x wrote=0x%02x read=0x%02x",
                 register_address, expected_value, actual_value);
        return false;
    }

    ESP_LOGI(TAG, "register verify PASS reg=0x%02x value=0x%02x",
             register_address, actual_value);
    return true;
}

static bool verify_who_am_i(const char *stage)
{
    uint8_t who_am_i = 0;
    const esp_err_t err = read_register(ICM42670P_WHO_AM_I_REG, &who_am_i);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I %s read failed: %s",
                 stage, esp_err_to_name(err));
        return false;
    }
    if (who_am_i != ICM42670P_WHO_AM_I_VALUE) {
        s_counters.protocol_errors++;
        ESP_LOGE(TAG, "WHO_AM_I %s FAIL read=0x%02x expected=0x%02x",
                 stage, who_am_i, ICM42670P_WHO_AM_I_VALUE);
        return false;
    }

    ESP_LOGI(TAG, "WHO_AM_I %s PASS address=0x%02x value=0x%02x",
             stage, ICM42670P_I2C_ADDRESS, who_am_i);
    return true;
}

static bool soft_reset_and_verify(void)
{
    esp_err_t err = write_register(
        ICM42670P_SIGNAL_PATH_RESET_REG, ICM42670P_SOFT_RESET_COMMAND);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "soft reset write failed: %s", esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t reset_status = 0;
    err = read_register(ICM42670P_INT_STATUS_REG, &reset_status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "soft reset status read failed: %s",
                 esp_err_to_name(err));
        return false;
    }
    if ((reset_status & ICM42670P_RESET_DONE_MASK) == 0) {
        s_counters.protocol_errors++;
        ESP_LOGE(TAG, "soft reset FAIL INT_STATUS=0x%02x reset_done=0",
                 reset_status);
        return false;
    }

    ESP_LOGI(TAG, "soft reset PASS INT_STATUS=0x%02x reset_done=1",
             reset_status);
    return verify_who_am_i("post-reset");
}

static bool configure_sensor(void)
{
    uint8_t interface_config = 0;
    esp_err_t err = read_register(ICM42670P_INTF_CONFIG0_REG,
                                  &interface_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INTF_CONFIG0 read failed: %s", esp_err_to_name(err));
        return false;
    }
    if ((interface_config & ICM42670P_SENSOR_DATA_BIG_ENDIAN_MASK) == 0) {
        s_counters.protocol_errors++;
        ESP_LOGE(TAG, "byte order FAIL INTF_CONFIG0=0x%02x expected=big",
                 interface_config);
        return false;
    }
    ESP_LOGI(TAG, "sensor byte order PASS INTF_CONFIG0=0x%02x endian=big",
             interface_config);

    if (!write_and_verify(ICM42670P_GYRO_CONFIG0_REG,
                          ICM42670P_GYRO_CONFIG0_VALUE) ||
        !write_and_verify(ICM42670P_ACCEL_CONFIG0_REG,
                          ICM42670P_ACCEL_CONFIG0_VALUE)) {
        return false;
    }

    uint8_t interrupt_source = 0;
    err = read_register(ICM42670P_INT_SOURCE0_REG, &interrupt_source);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INT_SOURCE0 read failed: %s", esp_err_to_name(err));
        return false;
    }
    interrupt_source |= ICM42670P_DRDY_INT1_ENABLE_MASK;
    if (!write_and_verify(ICM42670P_INT_SOURCE0_REG, interrupt_source) ||
        !write_and_verify(ICM42670P_PWR_MGMT0_REG,
                          ICM42670P_PWR_MGMT0_VALUE)) {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ICM42670P_STARTUP_WAIT_MS));
    return true;
}

static bool wait_for_data_ready(void)
{
    const int64_t started_at_us = esp_timer_get_time();
    const int64_t timeout_us =
        (int64_t)ICM42670P_DATA_READY_TIMEOUT_MS * 1000;

    do {
        uint8_t status = 0;
        if (read_register(ICM42670P_INT_STATUS_DRDY_REG, &status) == ESP_OK &&
            (status & ICM42670P_DATA_READY_MASK) != 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((esp_timer_get_time() - started_at_us) < timeout_us);

    s_counters.data_ready_timeouts++;
    return false;
}

static void log_runtime_summary(
    const robot_imu_icm42670p_si_sample_t *latest_sample,
    bool has_latest_sample)
{
    if (has_latest_sample) {
        const float ax = latest_sample->linear_acceleration_mps2[0];
        const float ay = latest_sample->linear_acceleration_mps2[1];
        const float az = latest_sample->linear_acceleration_mps2[2];
        const float gx = latest_sample->angular_velocity_rad_s[0];
        const float gy = latest_sample->angular_velocity_rad_s[1];
        const float gz = latest_sample->angular_velocity_rad_s[2];
        const float acceleration_norm = sqrtf(ax * ax + ay * ay + az * az);
        const float angular_velocity_norm = sqrtf(gx * gx + gy * gy + gz * gz);
        ESP_LOGI(TAG,
                 "ICM_RUNTIME samples=%" PRIu32
                 " i2c_errors=%" PRIu32 " protocol_errors=%" PRIu32
                 " drdy_timeouts=%" PRIu32 " parse_errors=%" PRIu32
                 " submit_errors=%" PRIu32 " test_pauses=%" PRIu32
                 " accel_mps2=[%+.4f,%+.4f,%+.4f] |a|=%.4f"
                 " gyro_rad_s=[%+.4f,%+.4f,%+.4f] |g|=%.4f",
                 s_counters.samples_ok,
                 s_counters.i2c_errors,
                 s_counters.protocol_errors,
                 s_counters.data_ready_timeouts,
                 s_counters.parse_errors,
                 s_counters.submit_errors,
                 s_counters.test_pauses,
                 (double)ax, (double)ay, (double)az,
                 (double)acceleration_norm,
                 (double)gx, (double)gy, (double)gz,
                 (double)angular_velocity_norm);
    } else {
        ESP_LOGW(TAG,
                 "ICM_RUNTIME samples=0 i2c_errors=%" PRIu32
                 " protocol_errors=%" PRIu32
                 " drdy_timeouts=%" PRIu32 " parse_errors=%" PRIu32
                 " submit_errors=%" PRIu32 " test_pauses=%" PRIu32,
                 s_counters.i2c_errors,
                 s_counters.protocol_errors,
                 s_counters.data_ready_timeouts,
                 s_counters.parse_errors,
                 s_counters.submit_errors,
                 s_counters.test_pauses);
    }
}

static void icm42670p_task(void *argument)
{
    (void)argument;
    int64_t next_log_at_us = esp_timer_get_time() +
        (int64_t)ICM42670P_RUNTIME_LOG_PERIOD_MS * 1000;
    robot_imu_icm42670p_si_sample_t latest_sample = {0};
    bool has_latest_sample = false;

    while (true) {
        uint32_t pause_ms = 0;
        if (xQueueReceive(s_test_pause_queue, &pause_ms, 0) == pdPASS) {
            ESP_LOGW(TAG,
                     "IMU_TEST_PAUSE start duration_ms=%" PRIu32
                     " sample submission disabled; motors unchanged",
                     pause_ms);
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
            s_counters.test_pauses++;
            ESP_LOGW(TAG,
                     "IMU_TEST_PAUSE complete duration_ms=%" PRIu32
                     " acquisition resumed",
                     pause_ms);
        }

        if (wait_for_data_ready()) {
            const int64_t sampled_at_us = esp_timer_get_time();
            uint8_t register_bytes[ROBOT_IMU_ICM42670P_DATA_LENGTH_BYTES] = {0};
            const esp_err_t read_err = read_registers(
                ROBOT_IMU_ICM42670P_DATA_START_REGISTER,
                register_bytes,
                sizeof(register_bytes));
            if (read_err == ESP_OK) {
                robot_imu_icm42670p_raw_sample_t raw_sample = {0};
                robot_imu_icm42670p_si_sample_t si_sample = {0};
                if (!robot_imu_icm42670p_parse_raw(register_bytes,
                                                    sizeof(register_bytes),
                                                    &raw_sample) ||
                    !robot_imu_icm42670p_convert_to_si(&raw_sample,
                                                       &si_sample)) {
                    s_counters.parse_errors++;
                } else {
                    const esp_err_t submit_err = robot_imu_submit_si_sample(
                        si_sample.linear_acceleration_mps2,
                        si_sample.angular_velocity_rad_s,
                        sampled_at_us);
                    if (submit_err == ESP_OK) {
                        s_counters.samples_ok++;
                        latest_sample = si_sample;
                        has_latest_sample = true;
                    } else {
                        s_counters.submit_errors++;
                    }
                }
            }
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_log_at_us) {
            log_runtime_summary(&latest_sample, has_latest_sample);
            next_log_at_us = now_us +
                (int64_t)ICM42670P_RUNTIME_LOG_PERIOD_MS * 1000;
        }
    }
}

static void cleanup_start_failure(void)
{
    if (s_device != NULL) {
        (void)i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    if (s_bus != NULL) {
        (void)i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    if (s_test_pause_queue != NULL) {
        vQueueDelete(s_test_pause_queue);
        s_test_pause_queue = NULL;
    }
}

esp_err_t robot_imu_icm42670p_backend_start(void)
{
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_counters, 0, sizeof(s_counters));
    s_test_pause_queue = xQueueCreate(1, sizeof(uint32_t));
    if (s_test_pause_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = ICM42670P_SDA_GPIO,
        .scl_io_num = ICM42670P_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C0 bus init failed: %s", esp_err_to_name(err));
        cleanup_start_failure();
        return err;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM42670P_I2C_ADDRESS,
        .scl_speed_hz = ICM42670P_I2C_CLOCK_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &device_config, &s_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(err));
        cleanup_start_failure();
        return err;
    }

    if (!verify_who_am_i("pre-reset") ||
        !soft_reset_and_verify() ||
        !configure_sensor()) {
        cleanup_start_failure();
        return ESP_ERR_INVALID_RESPONSE;
    }

    const BaseType_t created = xTaskCreate(
        icm42670p_task,
        "imu_icm42670p",
        ICM42670P_TASK_STACK_BYTES,
        NULL,
        ICM42670P_TASK_PRIORITY,
        NULL);
    if (created != pdPASS) {
        cleanup_start_failure();
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGW(TAG,
             "ICM42670P READY I2C0 SCL=GPIO%d SDA=GPIO%d clock=%dHz"
             " address=0x%02x frame=icm42670p_link"
             " axis=sensor-native installation_mapping=UNVERIFIED",
             ICM42670P_SCL_GPIO,
             ICM42670P_SDA_GPIO,
             ICM42670P_I2C_CLOCK_HZ,
             ICM42670P_I2C_ADDRESS);
    return ESP_OK;
}

esp_err_t robot_imu_icm42670p_backend_request_pause(uint32_t duration_ms)
{
    if (!s_started || s_test_pause_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (duration_ms < ICM42670P_TEST_PAUSE_MIN_MS ||
        duration_ms > ICM42670P_TEST_PAUSE_MAX_MS) {
        return ESP_ERR_INVALID_ARG;
    }
    return xQueueOverwrite(s_test_pause_queue, &duration_ms) == pdPASS
        ? ESP_OK
        : ESP_FAIL;
}
