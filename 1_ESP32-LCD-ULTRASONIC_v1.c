#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "HD44780.h"
#include <string.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "esp_timer.h"

// Définition des constantes
#define LCD_ADDR 0x27
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 16
#define LCD_ROWS 2

#define TRIG_PIN_1 GPIO_NUM_5
#define ECHO_PIN_1 GPIO_NUM_18
#define TRIG_PIN_2 GPIO_NUM_12
#define ECHO_PIN_2 GPIO_NUM_13
#define SOUND_SPEED 0.0343
#define CRITICAL_DISTANCE_CM 20.0
#define TIMEOUT 20000  

void init_ultrasonic_sensor() {
    gpio_set_direction(TRIG_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN_1, GPIO_MODE_INPUT);
    gpio_set_direction(TRIG_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN_2, GPIO_MODE_INPUT);
}

float measure_distance(gpio_num_t trig_pin, gpio_num_t echo_pin) {
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);  
    gpio_set_level(trig_pin, 0);

    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0 && esp_timer_get_time() - start_wait < TIMEOUT);

    int64_t start_time = esp_timer_get_time();
    printf("start_time: %" PRId64 " us\n", start_time);

    while (gpio_get_level(echo_pin) == 1 && esp_timer_get_time() - start_time < TIMEOUT);
    int64_t end_time = esp_timer_get_time();
    printf("end_time: %" PRId64 " us\n", end_time);

    int64_t pulse_duration = end_time - start_time;
    printf("pulse_duration: %" PRId64 " us\n", pulse_duration);

    float distance = (pulse_duration / 2.0) * SOUND_SPEED;
    printf("Distance: %.2f cm\n", distance);

    if (pulse_duration >= TIMEOUT) {
        return -1;  
    }

    return distance;
}

void ultrasonic_task(void *pvParameters) {
    while (true) {
        float distance1 = measure_distance(TRIG_PIN_1, ECHO_PIN_1);
        float distance2 = measure_distance(TRIG_PIN_2, ECHO_PIN_2);

        // Affichage sur l'écran LCD
        LCD_clearScreen();
        LCD_setCursor(0, 0);
        char dist1_str[10];
        sprintf(dist1_str, "%.2f cm", distance1);
        LCD_writeStr(dist1_str);

        LCD_setCursor(0, 1);
        char dist2_str[10];
        sprintf(dist2_str, "%.2f cm", distance2);
        LCD_writeStr(dist2_str);

        if (distance1 >= 0 && distance1 < CRITICAL_DISTANCE_CM && distance2 >= 0 && distance2 < CRITICAL_DISTANCE_CM) {
            printf("Objet critique détecté à gauche et à droite!\n");
        } else if (distance1 >= 0 && distance1 < CRITICAL_DISTANCE_CM) {
            printf("Objet critique détecté à gauche!\n");
        } else if (distance2 >= 0 && distance2 < CRITICAL_DISTANCE_CM) {
            printf("Objet critique détecté à droite!\n");
        } else {
            printf("Aucun objet critique détecté.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
}

void app_main() {
    // Initialiser le LCD
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);

    init_ultrasonic_sensor();
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 5, NULL);
}