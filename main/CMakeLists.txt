idf_component_register(SRCS "app_main.c" "gpio.c" "mqtt_handler.c" "sensors_read.c" "uart.c" INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
