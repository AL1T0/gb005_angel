#include "headers.h"

const char *TAG = "MQTT_TEST";

esp_mqtt_client_handle_t mqtt_client;

// @brief Task to read and publish data from measurements queue
void mqtt_publish_task(void *pvParameters){    
    
    esp_mqtt_client_handle_t mqtt_client = (esp_mqtt_client_handle_t)pvParameters;

    const char *temperature_topic = MAKER_TOPIC_TEMP;
    const char *co2_out_topic = MAKER_TOPIC_CO2o;
    const char *output_topic = MAKER_TOPIC_COMP;

    const char *payload_format_f = "{\"value\":%.2f}";
    const char *payload_format_i = "{\"value\":%d}";
    const char *payload_format_b = "{\"value\":%s}";

    char payload[128];
    measurement_q_t measurement;

    while (xQueueReceive(measurement_queue, &measurement, portMAX_DELAY)) {
        xSemaphoreTake(mqtt_semaphore,0);
            int payload_length = snprintf(payload, sizeof(payload), payload_format_f, measurement.temperature);
            int msg_id = esp_mqtt_client_publish(mqtt_client, temperature_topic, payload, payload_length, 0, 0);
            if (msg_id < 0) {
                ESP_LOGE(TAG, "Failed to send publish, error=%d", msg_id);

                // Delete the task if there was an error
                vTaskDelete(NULL);
            }
            
            vTaskDelay(1 / portTICK_PERIOD_MS);

            // CO2
            payload_length = snprintf(payload, sizeof(payload), payload_format_i, measurement.co2o);
            msg_id = esp_mqtt_client_publish(mqtt_client, co2_out_topic, payload, payload_length, 0, 0);
            if (msg_id < 0) {
                ESP_LOGE(TAG, "Failed to send publish, error=%d", msg_id);

                // Delete the task if there was an error
                vTaskDelete(NULL);
            }

            // Relay state
            if (measurement.relay_state) {
                payload_length = snprintf(payload, sizeof(payload), payload_format_b, "true"); // Represent true as a string
            } else {
                payload_length = snprintf(payload, sizeof(payload), payload_format_b, "false"); // Represent false as a string
            }
            msg_id = esp_mqtt_client_publish(mqtt_client, output_topic, payload, payload_length, 0, 0);
            if (msg_id < 0) {
                ESP_LOGE(TAG, "Failed to send publish, error=%d", msg_id);

                // Delete the task if there was an error
                vTaskDelete(NULL);
            }

            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            xSemaphoreGive(mqtt_semaphore);        
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
    esp_mqtt_client_handle_t mqtt_client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(mqtt_client, MAKER_TOPIC_CMD, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        xSemaphoreGive(mqtt_semaphore);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        esp_mqtt_client_reconnect(mqtt_client);
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

void mqtt_app_start(void)
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
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    // Launch the task to publish data
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, mqtt_client, configMAX_PRIORITIES - 1, NULL);
}