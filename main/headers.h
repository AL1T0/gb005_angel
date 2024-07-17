#ifndef HEADERS_H
#define HEADERS_H

#include <stdio.h>
#include <stdlib.h>
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

#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "MHZ19.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <cJSON.h>

// Temperature sensor
#define GPIO_TEMP_SENS       (5)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_12_BIT)

// Define queues and semaphores
extern QueueHandle_t measurement_queue;
extern SemaphoreHandle_t mqtt_semaphore;

// Define a structure to hold measurement data
typedef struct {
    float temperature;
    uint16_t co2o;
    uint16_t co2i;
    float pH;
    bool relay_state;
    bool light_state;
} measurement_q_t;

// Define the pins for controlling relay outputs
#define GPIO_RELAY_1 (GPIO_NUM_25)
#define GPIO_RELAY_2 (GPIO_NUM_32)

//UART pins for CO2 out sensor
#define TXDi_PIN (GPIO_NUM_23)
#define RXDi_PIN (GPIO_NUM_19)
#define UART_MHZin (UART_NUM_1)

//UART pins for CO2 out sensor
#define TXDO_PIN (GPIO_NUM_16)
#define RXDO_PIN (GPIO_NUM_17)
#define UART_MHZout (UART_NUM_2)

// Define ADC input for pH sensor
#define ADC1_CHANNEL (ADC1_CHANNEL_6)

// Define light sensor input
#define GPIO_LIGHT (GPIO_NUM_33)

// Define button input
#define GPIO_BUTTON (GPIO_NUM_27)

// PEM certificate for AllThingsTalk MQTT broker
#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_crt_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_crt_start[]   asm("_binary_emqx_pem_start");//asm("_binary_att_pem_start");
#endif
extern const uint8_t mqtt_crt_end[]   asm("_binary_emqx_pem_start");//asm("_binary_att_pem_start");

extern esp_mqtt_client_handle_t mqtt_client;

// Functions
void UART_init(uart_port_t UART_num, int RXD_PIN, int TXD_PIN);
void control_outputs(bool value);
void mqtt_app_start(void);
void gpio_init(void);
float get_pH(void);
static void IRAM_ATTR sensor_isr_handler(void* arg);
static void IRAM_ATTR button_isr_handler(void* arg);

// Tasks
void sensors_reading_task(void *pvParameters);
void mqtt_publish_task(void *pvParameters);

#endif;