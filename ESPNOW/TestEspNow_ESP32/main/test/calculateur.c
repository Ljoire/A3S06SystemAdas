/**
 * @file calculateur.c
 * @author Léon Joire (leon.joire@esme.fr)
 * @brief Calculateur du système ADAS en V2V.
 * Récupère les entrées capteurs et ESPNOW puis transmet aux sortie
 * Priorise les alertes 
 * Recode correctement ce qu'il faut envoyer aux tâches 
 * @version 0.1 
 * @date 2025-02-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "calculateur.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
static const char *TAG = "CALCULATEUR";
/**
 * @brief initialisation des queues d'I/O et de la tache du calculateur
 * 
 * @return ESP_OK la configuration c'est bien déroulé ESP_NOK une erreur a été rencontré
 * @brief Initialisation de tout les tâches et Queue 
 * @brief liste des queue d'entrées sorties 
 */
QueueHandle_t queueCapteur_rx = NULL;
QueueHandle_t queueMoteur_rx = NULL;
QueueHandle_t queueESPNOW_rx = NULL;

QueueHandle_t queueLCD_tx = NULL;
QueueHandle_t queueMoteur_tx = NULL;
QueueHandle_t queueESPNOW_tx = NULL;


//Queue par lequel le calculateur reçoit des codes erreur 
/*
extern QueueHandle_t queueMoteur_rx;
extern QueueHandle_t queueESPNOW_rx;
// Queue par lesquelles le calculateur transmet les code erreur
extern QueueHandle_t queueLCD_tx;
extern QueueHandle_t queueMoteur_tx;
extern QueueHandle_t queueESPNOW_tx;
*/

esp_err_t CalculatorTaskQueueInitiator(void){
    
        queueCapteur_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
        if (queueCapteur_rx == NULL) {
            ESP_LOGE("Queue", "Failed to create queueCapteur_rx");
            return ESP_FAIL;  // Si la création échoue, renvoyer une erreur
        }

    queueMoteur_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
    queueESPNOW_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));

    queueLCD_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
    queueMoteur_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
    queueESPNOW_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));



    // Création des tâches
    if (xTaskCreate(task_calculateur, "task_calculator", CALCULATOR_STACK_SIZE, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG,"Erreur à la création de la taches calculator");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG,"Création des taches du calculateur réussi");
    return ESP_OK;
    
}



bool ProcessEspNowData(void) {
    ALERT_DATA_FORMAT espnow_data;
    
    if (xQueueReceive(queueESPNOW_rx, &espnow_data, portMAX_DELAY) == pdTRUE) {
        ALERT_DATA_FORMAT DatatMoteur;

        switch (espnow_data) {
            case ESPNOW_EEBL_CRIT:
                // Mettre la PWM à 100% pour l'alerte 6
                vTaskDelay(pdMS_TO_TICKS(300));
                break;
            //même chose pour le cas a gauche ou a droite 
            case ESPNOW_DNPW_G:
            case ESPNOW_DNPW_D:
                DatatMoteur = SERVO_CENTRE;
                if (xQueueSend(queueMoteur_tx, &DatatMoteur, portMAX_DELAY) == pdTRUE) {
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                break;

            case ESPNOW_FCW_CRIT:
                DatatMoteur = MOTEUR_AVANT_LENT;
                if (xQueueSend(queueMoteur_tx, &DatatMoteur, portMAX_DELAY) == pdTRUE) {
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                break;

            default:
                break;
        }

        // Effectuer un ET logique avec 0xC0 pour extraire les 2 bits de poids fort
        int lcd_alert = espnow_data & 0xC0;
        if (xQueueSend(queueLCD_tx, &lcd_alert, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        return true; // Une donnée a été traitée
    }

    return false; // Aucune donnée à traiter
}

bool ProcessCapteurData(void) {
    ALERT_DATA_FORMAT capteur_data;
    if (xQueueReceive(queueCapteur_rx, &capteur_data, portMAX_DELAY) == pdTRUE) {
        
        ALERT_DATA_FORMAT moteur_data;
        ALERT_DATA_FORMAT espnow_data = capteur_data + 1;
        ALERT_DATA_FORMAT alert_data =capteur_data;
        switch (alert_data)
        {
        case CAPTEUR_EEBL_MID:
            //Mettre SONNERIE_LOW
            break;
        case CAPTEUR_EEBL_HIGH:
            moteur_data = MOTEUR_AVANT_LENT;
            //Mettre SONNERIE_INTERMEDIAIRE
            if (xQueueSend(queueMoteur_tx, &moteur_data, portMAX_DELAY) == pdTRUE && xQueueSend(queueESPNOW_tx,espnow_data,portMAX_DELAY) == pdTRUE) {
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            break;
        case CAPTEUR_EEBL_CRIT:
            //Mettre SONNERIE_FORT
            moteur_data = MOTEUR_STOP;
            if (xQueueSend(queueMoteur_tx, &moteur_data, portMAX_DELAY) == pdTRUE && xQueueSend(queueESPNOW_tx,espnow_data,portMAX_DELAY) == pdTRUE) {
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            break;
        case CAPTEUR_BSW_GAUCHE:
        case CAPTEUR_BSW_DROITE:
        case CAPTEUR_DNPW_G:
        case CAPTEUR_DNPW_D:
            moteur_data = SERVO_CENTRE;  
            //Mettre SONNERIE_INTERMEDIAIRE  
            if (xQueueSend(queueMoteur_tx, &moteur_data, portMAX_DELAY) == pdTRUE && xQueueSend(queueESPNOW_tx,espnow_data,portMAX_DELAY) == pdTRUE) {
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            break;      
        case CAPTEUR_FCW_HIGH:
            //mettre sonnerie low 
            moteur_data = MOTEUR_AVANT_LENT;
            if (xQueueSend(queueMoteur_tx, &moteur_data, portMAX_DELAY) == pdTRUE && xQueueSend(queueESPNOW_tx,espnow_data,portMAX_DELAY) == pdTRUE) {
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            break;
        case CAPTEUR_FCW_CRIT:
            //mettre SONNERIE_intermediaire
            moteur_data = MOTEUR_STOP;
            if (xQueueSend(queueMoteur_tx, &moteur_data, portMAX_DELAY) == pdTRUE && xQueueSend(queueESPNOW_tx,espnow_data,portMAX_DELAY) == pdTRUE) {
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            break;
            default:
            break;
        }
        //si il y a une alerte alors on fait un envoie en 2 fois sinon
        
        if ((alert_data & 0xC0) == 0xC0) {
            int lcd_alert = alert_data & 0xC0;
            if (xQueueSend(queueLCD_tx, &lcd_alert, portMAX_DELAY) == pdTRUE) {
                    vTaskDelay(pdTICKS_TO_MS(300));
                }
            }
        return true;
    }
return false;
}
        



void task_calculateur(void *pvParameters) {
    //Variable d'accueil local
    ALERT_DATA_FORMAT espnow_data, luminosite_data, capteur_data[SENSOR_FRAME_LENGH];
    ALERT_DATA_FORMAT moteur_received, capteur_received;
    while (1) {

        bool retCapt = ProcessCapteurData();
        bool retEspNow = ProcessEspNowData(); 
    }
}