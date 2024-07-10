/* MQTT over SSL Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   Customized by Alena Grebneva for the Green Backbone Project

   DS18B20 code taken from https://github.com/DavidAntliff/esp32-ds18b20-example
   LCD1602 code taken from https://github.com/DavidAntliff/esp32-i2c-lcd1602-example
*/

#include "headers.h"

const char *MQTT_TAG = "MQTT_TEST";

QueueHandle_t measurement_queue;
SemaphoreHandle_t mqtt_semaphore;

void app_main(void)
{
    //ESP_LOGI(MQTT_TAG, "[APP] Startup..");
    //ESP_LOGI(MQTT_TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    //ESP_LOGI(MQTT_TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("SENSORS_READ", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);        
    esp_log_level_set("owb", ESP_LOG_VERBOSE);
    esp_log_level_set("ds18b20", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // Initialize GPIO pins for controlling outputs
    gpio_init();

    // Initialize UART
    UART_init(UART_MHZout, RXDO_PIN, TXDO_PIN);
    /* TO DO Initialize the second sensor UART*/
        
    // Queue for measurements
    measurement_queue = xQueueCreate(1, sizeof(measurement_q_t));

    // Create the semaphores
    // And initialize the semaphore to 0 (signifying not available)
    mqtt_semaphore = xSemaphoreCreateBinary();
    if (mqtt_semaphore == NULL) {
        ESP_LOGE("APP_MAIN", "Failed to create semaphore");
        return;
    } else {
        xSemaphoreTake(mqtt_semaphore, 0);
    }

    xTaskCreate(sensors_reading_task, "sensors_reading_task", 4096, NULL, configMAX_PRIORITIES, NULL);
    mqtt_app_start();
}
