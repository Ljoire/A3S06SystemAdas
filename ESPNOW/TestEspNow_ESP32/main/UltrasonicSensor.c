/*
*   SENSOR CODE 
*/
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "ultrasonic.h"


/* Initialization of the sensor*/
void init_ultrasonic_sensor() {
    gpio_set_direction(TRIG_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN_1, GPIO_MODE_INPUT);
    gpio_set_direction(TRIG_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN_2, GPIO_MODE_INPUT);
}

/*  measure of the distance for one sensor
*   INPUT : Pin for triger and echo (int)
*   OUTPUT: Floating number who represent the distance*/
float measure_distance(int trig_pin, int echo_pin) {
    gpio_set_level(trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_pin, 0);

    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0 && esp_timer_get_time() - start_wait < TIMEOUT);

    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1 && esp_timer_get_time() - start_time < TIMEOUT);
    int64_t end_time = esp_timer_get_time();

    int64_t pulse_duration = end_time - start_time;
    float distance = (pulse_duration / 2.0) * SOUND_SPEED;

    if (pulse_duration >= TIMEOUT) {
        return -1;
    }

    return distance;
}