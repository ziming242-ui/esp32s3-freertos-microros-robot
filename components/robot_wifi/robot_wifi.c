#include "robot_wifi.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT    BIT1

static const char *TAG = "robot_wifi";

static EventGroupHandle_t s_wifi_events;
static int s_retry_count;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
        return;
    }

    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event =
            (const wifi_event_sta_disconnected_t *)event_data;
        const int reason = (event != NULL) ? event->reason : -1;

        if (s_retry_count < CONFIG_ROBOT_WIFI_MAXIMUM_RETRY) {
            s_retry_count++;
            ESP_LOGW(TAG,
                     "disconnected, reason=%d; reconnecting (%d/%d)",
                     reason,
                     s_retry_count,
                     CONFIG_ROBOT_WIFI_MAXIMUM_RETRY);
            ESP_ERROR_CHECK(esp_wifi_connect());
        } else {
            ESP_LOGE(TAG,
                     "connection failed after %d retries, last reason=%d",
                     CONFIG_ROBOT_WIFI_MAXIMUM_RETRY,
                     reason);
            xEventGroupSetBits(s_wifi_events, WIFI_FAILED_BIT);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event =
            (const ip_event_got_ip_t *)event_data;

        s_retry_count = 0;
        ESP_LOGI(TAG, "got IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t initialise_nvs(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_flash_init();
    }

    return err;
}

esp_err_t robot_wifi_connect(void)
{
    if (CONFIG_ROBOT_WIFI_SSID[0] == '\0') {
        ESP_LOGE(TAG, "Wi-Fi SSID is not configured");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = initialise_nvs();
    if (err != ESP_OK) {
        return err;
    }

    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        return err;
    }

    if (esp_netif_create_default_wifi_sta() == NULL) {
        return ESP_FAIL;
    }

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_register(WIFI_EVENT,
                                     ESP_EVENT_ANY_ID,
                                     wifi_event_handler,
                                     NULL);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_register(IP_EVENT,
                                     IP_EVENT_STA_GOT_IP,
                                     wifi_event_handler,
                                     NULL);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid,
             sizeof(wifi_config.sta.ssid),
             "%s", CONFIG_ROBOT_WIFI_SSID);
    snprintf((char *)wifi_config.sta.password,
             sizeof(wifi_config.sta.password),
             "%s", CONFIG_ROBOT_WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    const EventBits_t result = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    return (result & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}
