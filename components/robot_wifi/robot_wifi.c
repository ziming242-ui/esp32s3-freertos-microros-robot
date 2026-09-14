#include "robot_wifi.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#ifdef ROBOT_WIFI_DIAGNOSTIC_TWDT_STATUS
#include "freertos/task.h"
#include "esp_task_wdt.h"
#endif
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT    BIT1
#define WIFI_CONNECT_TIMEOUT_MS 30000U

static const char *TAG = "robot_wifi";

static EventGroupHandle_t s_wifi_events;
static int s_retry_count;

#ifdef ROBOT_WIFI_DIAGNOSTIC_TWDT_STATUS
static void log_idle_twdt_status(const char *stage)
{
    TaskHandle_t idle_tasks[portNUM_PROCESSORS];
    esp_err_t statuses[portNUM_PROCESSORS];

    for (BaseType_t core = 0; core < portNUM_PROCESSORS; ++core) {
        idle_tasks[core] = xTaskGetIdleTaskHandleForCore(core);
        statuses[core] = (idle_tasks[core] != NULL)
                             ? esp_task_wdt_status(idle_tasks[core])
                             : ESP_ERR_INVALID_STATE;
    }

#if portNUM_PROCESSORS == 2
    ESP_LOGI(TAG,
             "TWDT_PROBE phase=%s idle0=%p status0=%s(0x%x) idle1=%p status1=%s(0x%x)",
             stage,
             (void *)idle_tasks[0],
             esp_err_to_name(statuses[0]),
             (unsigned)statuses[0],
             (void *)idle_tasks[1],
             esp_err_to_name(statuses[1]),
             (unsigned)statuses[1]);
#else
    ESP_LOGI(TAG,
             "TWDT_PROBE phase=%s idle0=%p status0=%s(0x%x)",
             stage,
             (void *)idle_tasks[0],
             esp_err_to_name(statuses[0]),
             (unsigned)statuses[0]);
#endif
}
#endif

static void mark_connection_failed(const char *operation, esp_err_t err)
{
    ESP_LOGE(TAG, "%s failed: %s", operation, esp_err_to_name(err));
    if (s_wifi_events != NULL) {
        xEventGroupSetBits(s_wifi_events, WIFI_FAILED_BIT);
    }
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        const esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            mark_connection_failed("initial esp_wifi_connect", err);
        }
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
            const esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                mark_connection_failed("retry esp_wifi_connect", err);
            }
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

    s_retry_count = 0;
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

#ifdef ROBOT_WIFI_DIAGNOSTIC_TWDT_STATUS
    log_idle_twdt_status("before esp_wifi_start");
#endif

    err = esp_wifi_start();

#ifdef ROBOT_WIFI_DIAGNOSTIC_TWDT_STATUS
    log_idle_twdt_status("after esp_wifi_start");
#endif

    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "station started; waiting up to %u ms for IPv4",
             (unsigned)WIFI_CONNECT_TIMEOUT_MS);

    const EventBits_t result = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (result & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }
    if (result & WIFI_FAILED_BIT) {
        return ESP_FAIL;
    }

    ESP_LOGE(TAG, "connection timed out after %u ms",
             (unsigned)WIFI_CONNECT_TIMEOUT_MS);
    return ESP_ERR_TIMEOUT;
}
