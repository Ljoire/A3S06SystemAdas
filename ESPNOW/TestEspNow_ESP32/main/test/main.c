#include "anglemort.h"
#include "calculateur.h"
#include "i2c_lcd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"



#define ALERT_DISTANCE 20  // Distance seuil pour déclencher une alerte

static const char *TAG = "MAIN";


/**
 * @brief Initialisation de toutes les taches du système
 * mettre des log en cas d'erreur seulement
 * @return ESP_OK la configuration c'est bien déroulé
 * @return ESP_NOK une erreur a été rencontré
 */
esp_err_t ADASTaskQueueInitiator(void){
    
    esp_err_t ret = CalculatorTaskQueueInitiator();
    
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Erreur à la création des tâches calculateur");
    }

    if (xTaskCreate(sensor_task, "Sensor Task", 2048, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG,"Erreur à la création de la taches de capteur");
        return ESP_FAIL;
    }
    ESP_LOGE(TAG,"Création réussi");
    
    if (xTaskCreate(display_task, "Display task", 2048, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG,"Erreur à la création de la taches d'affichage");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void app_main() {

    
    esp_err_t ret = ADASTaskQueueInitiator();
    ESP_LOGI(TAG, "Système de détection d'angle mort initialisé.");
    // Vérification du retour d'erreur
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initialisation des tâches et des queues réussie.");
    } else {
        ESP_LOGE(TAG, "Échec de l'initialisation des tâches et des queues !");
    }
    
}