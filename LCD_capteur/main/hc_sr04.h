#ifndef HC_SR04_H
#define HC_SR04_H

#include <driver/gpio.h>

// Définir les broches GPIO pour les capteurs HC-SR04
#define TRIGGER_GPIO_LEFT  GPIO_NUM_5
#define ECHO_GPIO_LEFT     GPIO_NUM_18
#define TRIGGER_GPIO_RIGHT GPIO_NUM_19
#define ECHO_GPIO_RIGHT    GPIO_NUM_23

// Structure pour représenter un capteur
typedef struct {
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} hc_sr04_t;

// Fonction d'initialisation des capteurs
void hc_sr04_init(hc_sr04_t *sensor);

// Fonction pour mesurer la distance en cm
float measure_distance_cm(hc_sr04_t *sensor);

#endif // HC_SR04_H