#include "hc_sr04.h"
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>

#define ALERT_DISTANCE 20  // Distance seuil pour déclencher une alerte

void hc_sr04_init(hc_sr04_t *sensor) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << sensor->trigger_pin),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
    gpio_config(&io_conf);
}

float measure_distance_cm(hc_sr04_t *sensor) {
    gpio_set_level(sensor->trigger_pin, 1);
    ets_delay_us(10);
    gpio_set_level(sensor->trigger_pin, 0);

    uint64_t timeout = esp_timer_get_time() + 30000;
    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) return -1;
    }
    uint64_t echo_start = esp_timer_get_time();

    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) return -1;
    }
    uint64_t echo_end = esp_timer_get_time();

    float distance_cm = (echo_end - echo_start) / 58.0;
    if (distance_cm < 2 || distance_cm > 400) return -1;
    return distance_cm;
}

uint8_t detect_alert() {
    hc_sr04_t capteurs[] = {
        {TRIGGER_GPIO_ARG, ECHO_GPIO_ARG},
        {TRIGGER_GPIO_ARD, ECHO_GPIO_ARD},
        {TRIGGER_GPIO_AV, ECHO_GPIO_AV},
        {TRIGGER_GPIO_AVG, ECHO_GPIO_AVG},
        {TRIGGER_GPIO_AVD, ECHO_GPIO_AVD}
    };

    float distances[5];
    for (int i = 0; i < 5; i++) {
        hc_sr04_init(&capteurs[i]);
        distances[i] = measure_distance_cm(&capteurs[i]);
    }

    if (distances[0] < ALERT_DISTANCE) return 1; // Angle mort gauche
    if (distances[1] < ALERT_DISTANCE) return 2; // Angle mort droit

    if (distances[0] < ALERT_DISTANCE || distances[3] < ALERT_DISTANCE) return 3; // Obstacle arrière gauche ou avant gauche
    if (distances[1] < ALERT_DISTANCE || distances[4] < ALERT_DISTANCE) return 4; // Obstacle arrière droit ou avant droit

    if (distances[2] < 60) return 6; // Distance inférieure à 60 cm
    if (distances[2] < 40) return 7; // Distance inférieure à 40 cm
    if (distances[2] < 20) return 8; // Distance inférieure à 20 cm

    if (distances[2] != -1 && distances[4] == -1 && distances[1] == -1) return 9; // Dépassement à droite 
    if (distances[2] != -1 && distances[3] == -1 && distances[0] == -1) return 10;  // Dépassement à gauche
    return 0;
}
