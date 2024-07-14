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

    esp_rom_gpio_pad_select_gpio(GPIO_LIGHT);
    gpio_set_direction(GPIO_LIGHT, GPIO_MODE_INPUT);

    // Initialize the output pins to a known state (e.g., OFF)
    gpio_set_level(GPIO_RELAY_1, 0);
    gpio_set_level(GPIO_RELAY_2, 0);

    // Configure ADC for pH reading
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_11);
    //esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

// Function to control the outputs based on MQTT data
void control_outputs(bool value) {
    // Set the GPIO pins based on the received value
    gpio_set_level(GPIO_RELAY_1, value ? 0 : 1);
    gpio_set_level(GPIO_RELAY_2, value ? 0 : 1);

    //NOTE: when output is LOW (0), current flows and the NO Switch closes/NC switch opens
    //For the projet we will use NC configuration

    //ESP_LOGI(IO, "Outputs set to %s", value ? "ON" : "OFF");
}

// Function to read ADC and convert the value to pH
float get_pH(void) {
    uint32_t adc_reading = adc1_get_raw(ADC1_CHANNEL);
    return 4.0 + ((adc_reading - 0.0) * (15.0 - 4.0) / (4095.0 - 0.0));
}