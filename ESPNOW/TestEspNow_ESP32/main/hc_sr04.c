#include "hc_sr04.h"
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>

void hc_sr04_init(hc_sr04_t *sensor) {
    // Configure Trigger pin as output
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << sensor->trigger_pin),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    // Configure Echo pin as input
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
    gpio_config(&io_conf);
}

float measure_distance_cm(hc_sr04_t *sensor) {
    // Envoyer une impulsion sur la broche TRIGGER
    gpio_set_level(sensor->trigger_pin, 1);
    ets_delay_us(10);
    gpio_set_level(sensor->trigger_pin, 0);

    // Mesurer la durée de l'écho avec timeout
    uint64_t timeout = esp_timer_get_time() + 30000; // 30ms timeout
    
    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) return -1;
    }
    uint64_t echo_start = esp_timer_get_time();
    
    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) return -1;
    }
    uint64_t echo_end = esp_timer_get_time();

    float distance_cm = (echo_end - echo_start) / 58.0;
    
    // Filtrer les valeurs aberrantes
    if (distance_cm < 2 || distance_cm > 400) return -1;
    
    return distance_cm;
}