#ifndef HC_SR04_H
#define HC_SR04_H

#include <driver/gpio.h>
#include "calculateur.h"
#define ALERT_DISTANCE_30 30 
#define ALERT_DISTANCE_20 20 
#define ALERT_DISTANCE_10 10

#define DIST_MIN_DETECT 2


#define CPT_ARG 0
#define CPT_ARD 1
#define CPT_AV 2
#define CPT_AVG 3
#define CPT_AVD 4
#define CPT_AR 5

#define ALERT_DISTANCE_30 30 
#define ALERT_DISTANCE_20 20 
#define ALERT_DISTANCE_10 10

#define DIST_MIN_DETECT 2



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
#define TRIGGER_GPIO_AR   GPIO_NUM_15  // Capteur Arrière
#define ECHO_GPIO_AR      GPIO_NUM_33

#define CAPTEUR_NUMBER 6

extern int16_t global_distances[6];
// Structure pour représenter un capteur

typedef struct {
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} hc_sr04_t;

// Fonctions
/**
 * @brief Initialisation des capteurs 
 * 
 * @param sensor 
 */
void hc_sr04_init(hc_sr04_t *sensor);

/**
 * @brief mesure de la distance. Renvoie une valeur en cm
 * 
 * @param sensor cpt 
 * @return int16_t
 * -1: erreur de timeout par la non-mise à 0 de la pin echo.
 * -2: erreur de timeout par la non-mise à 1 de la pin echo
 * -3: erreur car la mesure n'est pas comprise entre 2cm et 400cm
 */
int16_t measure_distance_cm(hc_sr04_t *sensor);

/**
 * @brief 
 * 
 * @param global_distances 
 * @param alert_code 
 * @return ALERT_DATA_FORMAT 
 */
ALERT_DATA_FORMAT update_alert_code(int16_t * global_distances, ALERT_DATA_FORMAT alert_code);


#endif // HC_SR04_H
/**
 * @brief Tâches de mesures périodique de l'ensemble des capteurs 
 * 
 * @param pvParameters 
 */
void sensor_task(void *pvParameters);
