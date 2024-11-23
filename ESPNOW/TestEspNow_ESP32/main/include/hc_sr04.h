#ifndef HC_SR04_H
#define HC_SR04_H

#include <driver/gpio.h>

// Définition des broches GPIO pour les capteurs HC-SR04
// Capteurs pour angles morts (affichés sur LCD 2x16)
#define TRIGGER_GPIO_G    GPIO_NUM_5   // Capteur Gauche
#define ECHO_GPIO_G       GPIO_NUM_18
#define TRIGGER_GPIO_D    GPIO_NUM_19  // Capteur Droite
#define ECHO_GPIO_D       GPIO_NUM_23

// Capteurs additionnels (affichés sur LCD 4x20)
#define TRIGGER_GPIO_AV   GPIO_NUM_25  // Capteur Avant
#define ECHO_GPIO_AV      GPIO_NUM_26
#define TRIGGER_GPIO_AVG  GPIO_NUM_27  // Capteur Avant Gauche
#define ECHO_GPIO_AVG     GPIO_NUM_14
#define TRIGGER_GPIO_AVD  GPIO_NUM_12  // Capteur Avant Droit
#define ECHO_GPIO_AVD     GPIO_NUM_13
#define TRIGGER_GPIO_AR   GPIO_NUM_4   // Capteur Arrière
#define ECHO_GPIO_AR      GPIO_NUM_16

// Structure pour représenter un capteur
typedef struct {
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} hc_sr04_t;

// Fonctions
int hc_sr04_init(hc_sr04_t *sensor);
float measure_distance_cm(hc_sr04_t *sensor);

#endif // HC_SR04_H
