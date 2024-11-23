#include "hc_sr04.h"
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>

int hc_sr04_init(hc_sr04_t *sensor) {
    // Configure Trigger pin as output
    if (sensor == NULL) {
        printf("Erreur: Le pointeur du capteur est NULL\n");
        return -1;
    }

    if (sensor->trigger_pin < 0 || sensor->echo_pin < 0) {
        printf("Erreur: Les broches TRIGGER ou ECHO ne sont pas correctement définies\n");
        return -2;
    }

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << sensor->trigger_pin),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("Erreur: Impossible de configurer la broche TRIGGER\n");
        return -3;
    }

    // Configure Echo pin as input
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
    
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("Erreur: Impossible de configurer la broche ECHO\n");
        return -4;
    }

    return 0;
}

float measure_distance_cm(hc_sr04_t *sensor) {
    
    if (sensor == NULL) {
        printf("Erreur: Le pointeur du capteur est NULL\n");
        return -1;
    }

    if (sensor->trigger_pin < 0 || sensor->echo_pin < 0) {
        printf("Erreur: Les broches TRIGGER ou ECHO ne sont pas valides\n");
        return -2;
    }


    gpio_set_level(sensor->trigger_pin, 1);
    ets_delay_us(10);
    gpio_set_level(sensor->trigger_pin, 0);

    // Mesurer la durée de l'écho avec timeout
    uint64_t timeout = esp_timer_get_time() + 30000; // 30ms timeout
    
    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) {
            printf("Erreur: Pas de signal de début d'écho (timeout)\n");
            return -3;
        }
    }
    uint64_t echo_start = esp_timer_get_time();
    
    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) {
            printf("Erreur: Signal d'écho trop long (timeout)\n");
            return -4;
        }
    }
    uint64_t echo_end = esp_timer_get_time();

    float distance_cm = (echo_end - echo_start) / 58.0;
    
    // Filtrer les valeurs aberrantes
    if (distance_cm < 2 || distance_cm > 400) {
        printf("Erreur: Distance mesurée hors des limites valides (%.2f cm)\n", distance_cm);
        return -5;
    }
    
    return distance_cm;
}