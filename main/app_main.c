/* MQTT over SSL Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   Customized by Alena Grebneva for the Green Backbone Project

   DS18B20 code taken from https://github.com/DavidAntliff/esp32-ds18b20-example
   LCD1602 code taken from https://github.com/DavidAntliff/esp32-i2c-lcd1602-example
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "keys.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>

#include <esp_timer.h>

#include "driver/gpio.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
//#include "MHZ19/MHZ19.h"

#include <cJSON.h>

#define GPIO_DS18B20_0       (5)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)
QueueHandle_t temperature_queue;

// Define the pins for controlling relay outputs
#define RELAY_1 25
#define RELAY_2 32

// Define variables to store the output states
bool relay1_state = false;
bool relay2_state = false;

static const char *IO = "I/O TEST";
static const char *TAG = "MQTT_TEST";

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_crt_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_crt_start[]   asm("_binary_att_pem_start");
#endif
extern const uint8_t mqtt_crt_end[]   asm("_binary_att_pem_start");

void DS18B20_task(void){
    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);        
    
    // Create DS18B20 devices on the 1-Wire bus
    DS18B20_Info * DSB = 0;
    DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
    DSB = ds18b20_info;
    //printf("Single device optimisations enabled\n");
    ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
    float temp;
   
    while(1){    
        // Override global log level
        //esp_log_level_set("*", ESP_LOG_INFO);

        // To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
        //esp_log_level_set("owb", ESP_LOG_DEBUG);
        //esp_log_level_set("ds18b20", ESP_LOG_DEBUG);

        // Stable readings require a brief period before communication
        vTaskDelay(10000.0 / portTICK_PERIOD_MS);

        ds18b20_convert_and_read_temp(DSB, &temp);
        //printf("\nTemperature readings (degrees C):\n");
        printf("  T: %.3f degC\n", temp);
        
        xQueueSend(temperature_queue, &temp, portMAX_DELAY);
        // clean up dynamically allocated data
        //ds18b20_free(&devices[i]);
        //owb_uninitialize(owb);
    }
}

// Function to control the outputs based on MQTT data
void control_outputs(bool value) {
    // Set the GPIO pins based on the received value
    gpio_set_level(RELAY_1, value);
    gpio_set_level(RELAY_2, value);

    //NOTE: when output is LOW (0), current flows and the NO Switch closes/NC switch opens
    //For the projet we will use NC configuration

    // Update the output state variables
    relay1_state = value;
    relay2_state = value;

    ESP_LOGI(IO, "Outputs set to %s", value ? "ON" : "OFF");
}

static void gpio_init() {
    gpio_config_t io_conf;
    // Disable interrupt for the pins
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << RELAY_1) | (1ULL << RELAY_2);
    // Disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // Disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // Configure GPIO with the given settings
    gpio_config(&io_conf);

    // Initialize the output pins to a known state (e.g., OFF)
    gpio_set_level(RELAY_1, 0);
    gpio_set_level(RELAY_2, 0);
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// Note: this function is for testing purposes only publishing part of the active partition
//       (to be checked against the original binary)
//
static void send_payload_task(void *pvParameters){    
    
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameters;
    //const TickType_t delay_ms = 5000 / portTICK_PERIOD_MS; // Send measurements every 5 seconds

    const char *temperature_topic = MAKER_TOPIC_TEMP;
    const char *payload_format = "{\"value\":%.2f}";
    char payload[128];
    float temperature;
    while (xQueueReceive(temperature_queue, &temperature, portMAX_DELAY)) {
        int payload_length = snprintf(payload, sizeof(payload), payload_format, temperature);
        int msg_id = esp_mqtt_client_publish(client, temperature_topic, payload, payload_length, 0, 0);
        if (msg_id < 0) {
            ESP_LOGE(TAG, "Failed to send publish, error=%d", msg_id);

            // Delete the task if there was an error
            vTaskDelete(NULL);
        }
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, MAKER_TOPIC_CMD, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        xTaskCreate(&send_payload_task, "send_payload_task", 4096, client, configMAX_PRIORITIES - 1, NULL);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        esp_mqtt_client_reconnect(client);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        cJSON *root = cJSON_Parse(event->data);
        if (root != NULL) {
            cJSON *value = cJSON_GetObjectItem(root, "value");
            if (value != NULL) {
                if (cJSON_IsBool(value)) {
                    bool output_value = cJSON_IsTrue(value);
                    control_outputs(output_value);
                }
            }
            cJSON_Delete(root);
        }

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        } else {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = "mqtts://api.allthingstalk.io",
            .address.port = 8883,
            .verification.certificate = (const char *)mqtt_crt_start
        },
        .credentials = {
            .set_null_client_id = true,
            .username = MAKER_USERNAME,
            .authentication.password = MAKER_PASSWORD
        }
    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

     esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
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
    
    temperature_queue = xQueueCreate(1, sizeof(float));
    xTaskCreate(&DS18B20_task, "DS18B20_task", 4096, NULL, 5, NULL);
    mqtt_app_start();
}
