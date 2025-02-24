#ifndef HC_SR04_H
#define HC_SR04_H

#include <driver/gpio.h>

// Définition des broches GPIO pour les capteurs HC-SR04
#define TRIGGER_GPIO_ARG  GPIO_NUM_4   // Capteur Arrière Gauche
#define ECHO_GPIO_ARG     GPIO_NUM_16
#define TRIGGER_GPIO_ARD  GPIO_NUM_12  // Capteur Arrière Droit
#define ECHO_GPIO_ARD     GPIO_NUM_13
#define TRIGGER_GPIO_AV   GPIO_NUM_25  // Capteur Avant
#define ECHO_GPIO_AV      GPIO_NUM_26
#define TRIGGER_GPIO_AVG  GPIO_NUM_27  // Capteur Avant Gauche
#define ECHO_GPIO_AVG     GPIO_NUM_14
#define TRIGGER_GPIO_AVD  GPIO_NUM_19  // Capteur Avant Droit
#define ECHO_GPIO_AVD     GPIO_NUM_23
#define TRIGGER_GPIO_AR   GPIO_NUM_32  // Capteur Arrière
#define ECHO_GPIO_AR      GPIO_NUM_33

#define CAPTEUR_NUMBER 5

extern uint16_t global_distances[6];
// Structure pour représenter un capteur
typedef struct {
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} hc_sr04_t;

// Fonctions
void hc_sr04_init(hc_sr04_t *sensor);
uint16_t measure_distance_cm(hc_sr04_t *sensor);
void detect_alert();

#endif // HC_SR04_H

void sensor_task(void *pvParameters);
