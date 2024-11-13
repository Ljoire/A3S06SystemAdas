#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"


#define TRIG_PIN GPIO_NUM_5      
#define ECHO_PIN GPIO_NUM_18    
#define SOUND_SPEED 0.0343       

void init_ultrasonic_sensor() {
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}

float measure_distance() {

    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);  
    gpio_set_level(TRIG_PIN, 0);

    while (gpio_get_level(ECHO_PIN) == 0);
    int64_t start_time = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) == 1);
    int64_t end_time = esp_timer_get_time();

    int64_t pulse_duration = end_time - start_time;

    float distance = (pulse_duration / 2.0) * SOUND_SPEED;
    printf("Distance: %.2f cm\n", distance);  

    return distance;
}

void ultrasonic_task(void *pvParameters) {
    while (true) {
        float distance = measure_distance();
        vTaskDelay(10000 / portTICK_PERIOD_MS);  
    }
}

/* TO implement*/
//    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 5, NULL);
