#include "anglemort.h"
#include "calculateur.h"

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
 * @brief initialisation des queues d'I/O et de la tache du calculateur
 * 
 * @return ESP_OK la configuration c'est bien déroulé
 * ESP_NOK une erreur a été rencontré
 */
esp_err_t CalculatorTaskQueueInitiator(void){
    // Queue réceptrice vu du calculateur
    queueCapteur_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,ALERT_DATA_FORMAT);
    queueMoteur_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,ALERT_DATA_FORMAT);
    queueESPNOW_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,ALERT_DATA_FORMAT);
    
    queueLCD_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,ALERT_DATA_FORMAT);
    queueMoteur_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,ALERT_DATA_FORMAT);
    queueESPNOW_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,ALERT_DATA_FORMAT);

    xTaskCreate(task_calculateur,"task_calculator",CALCULATOR_STACK_SIZE,NULL,1,NULL);
    xTaskCreate(sensor_task, "Sensor Task", 1024, NULL, 2, NULL);
    // Vérification des queues
    if (!queueCapteur_rx || !queueMoteur_rx || !queueESPNOW_rx ||
        !queueLCD_tx || !queueMoteur_tx || !queueESPNOW_tx) {
        ESP_LOGE(TAG,"Erreur à la création des Queue");
        return ESP_FAIL;
    }

    // Création des tâches
    if (xTaskCreate(task_calculateur, "task_calculator", CALCULATOR_STACK_SIZE, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG,"Erreur à la création de la taches calculator");
        return ESP_FAIL;
    }

    if (xTaskCreate(sensor_task, "Sensor Task", 1024, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG,"Erreur à la création de la taches de capteur");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,"Création réussi");
    return ESP_OK;
    
}

void app_main() {

    ESP_LOGI(TAG, "Système de détection d'angle mort initialisé.");

    esp_err_t ret = CalculatorTaskQueueInitiator();
    // Vérification du retour d'erreur
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initialisation des tâches et des queues réussie.");
    } else {
        ESP_LOGE(TAG, "Échec de l'initialisation des tâches et des queues ! Code d'erreur : %d", err);
        // Ici, tu peux décider de redémarrer l'ESP32 en cas d'échec
        // esp_restart();
    }
    
}









