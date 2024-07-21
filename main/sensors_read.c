
#include "headers.h"

const char *TAGs = "SENSORS_READ";

void sensors_reading_task(void *pvParameters){
    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_TEMP_SENS, RMT_CHANNEL_1, RMT_CHANNEL_0);        
    
    // Create DS18B20 devices on the 1-Wire bus
    DS18B20_Info * DSB = 0;
    DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
    DSB = ds18b20_info;
    //printf("Single device optimisations enabled\n");
    ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
    float temp;
    measurement_t co2_out_reading;
    measurement_q_t measurement;
   
    while(1){    
        // Override global log level
        //esp_log_level_set("*", ESP_LOG_INFO);

        // To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
        //esp_log_level_set("owb", ESP_LOG_DEBUG);
        //esp_log_level_set("ds18b20", ESP_LOG_DEBUG);
        
        // Get sensors readings each 30s
        vTaskDelay(29000.0 / portTICK_PERIOD_MS);

        // Get DS18B20 temperature
        ds18b20_convert_and_read_temp(DSB, &temp);
        
        ESP_LOGI(TAGs, "Temperature: %.3f °C",temp);
        
        //printf("\nTemperature readings (degrees C):\n");
        //printf("    T: %.3f degC\n", temp);
        measurement.temperature = temp;

        // Get CO2 reading
        co2_out_reading = getMeasurement(UART_MHZout);
        measurement.co2o = co2_out_reading.co2_ppm;
        measurement.co2i = co2_out_reading.co2_ppm + temp*10.0; //Simulate CO2in

        ESP_LOGI(TAGs, "CO2 level: %d ppm",co2_out_reading.co2_ppm);
        //printf("    CO2 Level: %d ppm\n", co2_out_reading.co2_ppm);

        /* TO DO: add the reading of the second CO2 sensor*/
        
        // Get compressor relays status
        int state = gpio_get_level(GPIO_RELAY_2);
        measurement.relay_state = (state == 0);
        ESP_LOGI(TAGs, "Digital output is (%d)", state);

        // Simulate pH reading
        measurement.pH = (co2_out_reading.co2_ppm - temp*10.0)/100.0;

        //Send the data to the measurement queue
        xQueueSend(measurement_queue, &measurement, portMAX_DELAY);
    }
}

