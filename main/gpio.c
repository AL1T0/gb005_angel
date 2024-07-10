#include "headers.h"

const char *IO = "I/O TEST";

//Function to initialize I/O
void gpio_init() {
    gpio_config_t io_conf;
    // Disable interrupt for the pins
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Set as output mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    // Bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << GPIO_RELAY_1) | (1ULL << GPIO_RELAY_2);
    // Disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // Disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // Configure GPIO with the given settings
    gpio_config(&io_conf);

    // Initialize the output pins to a known state (e.g., OFF)
    gpio_set_level(GPIO_RELAY_1, 0);
    gpio_set_level(GPIO_RELAY_2, 0);
}

// Function to control the outputs based on MQTT data
void control_outputs(bool value) {
    // Set the GPIO pins based on the received value
    gpio_set_level(GPIO_RELAY_1, value ? 0 : 1);
    gpio_set_level(GPIO_RELAY_2, value ? 0 : 1);

    //NOTE: when output is LOW (0), current flows and the NO Switch closes/NC switch opens
    //For the projet we will use NC configuration

    ESP_LOGI(IO, "Outputs set to %s", value ? "ON" : "OFF");
}