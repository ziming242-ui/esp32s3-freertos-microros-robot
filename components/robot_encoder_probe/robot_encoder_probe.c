#include "robot_encoder_probe.h"

#include <stddef.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ROBOT_ENCODER_COUNT_LIMIT 30000
#define ROBOT_ENCODER_SAMPLE_MS 200

typedef struct {
    int edge_a_gpio;
    int level_b_gpio;
} robot_encoder_pin_pair_t;

/* These pairs follow the Yahboom encoder sample. M3/M4 intentionally swap
 * A/B at the PCNT edge input so the vendor's sign convention is preserved. */
static const robot_encoder_pin_pair_t s_pins[4] = {
    {6, 7},
    {47, 48},
    {12, 11},
    {2, 1},
};

static pcnt_unit_handle_t s_units[4];
static const char *TAG = "encoder_probe";

static esp_err_t init_quadrature_unit(size_t index)
{
    const pcnt_unit_config_t unit_config = {
        .high_limit = ROBOT_ENCODER_COUNT_LIMIT,
        .low_limit = -ROBOT_ENCODER_COUNT_LIMIT,
    };
    esp_err_t err = pcnt_new_unit(&unit_config, &s_units[index]);
    if (err != ESP_OK) {
        return err;
    }

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    err = pcnt_unit_set_glitch_filter(s_units[index], &filter_config);
    if (err != ESP_OK) {
        return err;
    }

    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = s_pins[index].edge_a_gpio,
        .level_gpio_num = s_pins[index].level_b_gpio,
    };
    pcnt_channel_handle_t channel_a = NULL;
    err = pcnt_new_channel(s_units[index], &channel_a_config, &channel_a);
    if (err != ESP_OK) {
        return err;
    }

    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = s_pins[index].level_b_gpio,
        .level_gpio_num = s_pins[index].edge_a_gpio,
    };
    pcnt_channel_handle_t channel_b = NULL;
    err = pcnt_new_channel(s_units[index], &channel_b_config, &channel_b);
    if (err != ESP_OK) {
        return err;
    }

    err = pcnt_channel_set_edge_action(
        channel_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_channel_set_level_action(
        channel_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_channel_set_edge_action(
        channel_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_channel_set_level_action(
        channel_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK) {
        return err;
    }

    err = pcnt_unit_enable(s_units[index]);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_unit_clear_count(s_units[index]);
    if (err != ESP_OK) {
        return err;
    }
    return pcnt_unit_start(s_units[index]);
}

static void encoder_probe_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        int counts[4] = {0};
        bool read_ok = true;
        for (size_t i = 0; i < 4; ++i) {
            if (pcnt_unit_get_count(s_units[i], &counts[i]) != ESP_OK) {
                read_ok = false;
            }
        }

        if (read_ok) {
            ESP_LOGI(TAG,
                     "ENCODER_COUNTS M1=%d M2=%d M3=%d M4=%d",
                     counts[0], counts[1], counts[2], counts[3]);
        } else {
            ESP_LOGE(TAG, "PCNT read failed");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_ENCODER_SAMPLE_MS));
    }
}

esp_err_t robot_encoder_probe_start(void)
{
    ESP_LOGW(TAG, "READ-ONLY MODE: PWM and motor outputs are not configured");
    ESP_LOGI(TAG,
             "vendor PCNT pins: M1=6/7 M2=47/48 M3=11/12 M4=1/2");

    for (size_t i = 0; i < 4; ++i) {
        esp_err_t err = init_quadrature_unit(i);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "M%u PCNT init failed: %s",
                     (unsigned)(i + 1), esp_err_to_name(err));
            return err;
        }
    }

    BaseType_t created = xTaskCreate(
        encoder_probe_task,
        "encoder_probe",
        3072,
        NULL,
        5,
        NULL);
    return created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
