#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

static QueueHandle_t s_counter_queue;

static void producer_task(void *arg)
{
    uint32_t counter = 0;

    for (;;) {
        counter++;

        if (xQueueSend(s_counter_queue, &counter,
                       pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGI("producer", "sent=%lu",
                     (unsigned long)counter);
        }

        if ((counter % 5U) == 0U) {
            ESP_LOGW("producer", "simulate 3000 ms link silence");
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void consumer_task(void *arg)
{
    uint32_t counter;

    for (;;) {
        if (xQueueReceive(s_counter_queue, &counter,
                          pdMS_TO_TICKS(1500)) == pdPASS) {
            ESP_LOGI("comm", "ONLINE received=%lu",
                     (unsigned long)counter);
        } else {
            ESP_LOGW("comm", "TIMEOUT: no data for 1500 ms");
        }
    }
}

void app_main(void)
{
    s_counter_queue = xQueueCreate(8, sizeof(uint32_t));
    configASSERT(s_counter_queue != NULL);

    configASSERT(xTaskCreate(producer_task, "producer",
                             3072, NULL, 5, NULL) == pdPASS);
    configASSERT(xTaskCreate(consumer_task, "consumer",
                             3072, NULL, 6, NULL) == pdPASS);

    ESP_LOGI("main", "two FreeRTOS tasks started");
}
