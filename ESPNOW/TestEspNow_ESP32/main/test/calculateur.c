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



static const char *TAG = "CALCULATEUR";
    /***** @brief Liste des entrée ****/
    /** @brief ESPNOW **/
#define ESPNOW_EEBL_CRIT 6
#define ESPNOW_DNPW_G 12
#define ESPNOW_DNPW_D 14
#define ESPNOW_FCW_CRIT 19
//Trame du capteur [Code Alerte, Distance1, Distance2,Distance3,Distance4,Distance5,Distance6]
#define SENSOR_FRAME_LENGH 6 
//en comptant le 0

    /***** @brief Liste des sortie ****/
    /** @brief Sortie Moteur **/
#define MOTEUR_AVANT_LENT 2
#define MOTEUR_STOP 3
#define SERVO_CENTRE 4
    /** @brief buzzer*/
#define SONNERIE_STOP 0
#define SONNERIE_LOW 1
#define SONNERIE_INTERMEDIAIRE 2
#define SONNERIE_FORT 3
/** @brief LCD */


/**
 * @brief initialisation des queues d'I/O et de la tache du calculateur
 * 
 * @return ESP_OK la configuration c'est bien déroulé ESP_NOK une erreur a été rencontré
 * @brief Initialisation de tout les tâches et Queue 
 * @brief liste des queue d'entrées sorties 
 */
//Queue par lequel le calculateur reçoit des codes erreur 
extern QueueHandle_t queueCapteur_rx;
extern QueueHandle_t queueMoteur_rx;
extern QueueHandle_t queueESPNOW_rx;
// Queue par lesquelles le calculateur transmet les code erreur
extern QueueHandle_t queueLCD_tx;
extern QueueHandle_t queueMoteur_tx;
extern QueueHandle_t queueESPNOW_tx;
#define CALCULATOR_QUEUE_LENGHT 15
#define ALERT_DATA_FORMAT uint8_t
#define CALCULATOR_STACK_SIZE 2
esp_err_t CalculatorTaskQueueInitiator(void){
    
    queueCapteur_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
    queueMoteur_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
    queueESPNOW_rx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));

    queueLCD_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
    queueMoteur_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));
    queueESPNOW_tx = xQueueCreate(CALCULATOR_QUEUE_LENGHT,sizeof(ALERT_DATA_FORMAT));

    xTaskCreate(task_calculateur,"task_calculator",CALCULATOR_STACK_SIZE,NULL,1,NULL);

    // Création des tâches
    if (xTaskCreate(task_calculateur, "task_calculator", CALCULATOR_STACK_SIZE, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG,"Erreur à la création de la taches calculator");
        return ESP_FAIL;
    }
    //ESP_LOGI(TAG,"Création des taches du calculateur réussi");
    return ESP_OK;
    
}


/********** FONCTION D ALERTE  DE L ESP***********/
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

void task_calculateur(void *pvParameters) {
    //Variable d'accueil local
    ALERT_DATA_FORMAT espnow_data, moteur_data, capteur_data[SENSOR_FRAME_LENGH];
    ALERT_DATA_FORMAT moteur_received, capteur_received;
    while (1) {


        
        // Dépiler moteur et capteur séparément
        // Reprendre car un cas d'erreur est qu'il peut ne rien avoir car ça a été dépilé avant
        moteur_data = xQueueReceive(queueMoteur_rx, &moteur_received, portMAX_DELAY);
        capteur_data = xQueueReceive(queueCapteur_rx, &capteur_received, portMAX_DELAY);
        /**
         * @brief Le capteur détecte un DNPW alors que le moteur est en train de tourner 
         * TODO implémenter le cas distant avec un envoie en ESPNOW
         * // A implementer quand plus clair
         * // xQueueSend(queueESPNOW_tx, &capteur_data, portMAX_DELAY);
         * @param capteur_data qui détecte un DNPW 
         * @param moteur_data qui transmet qu'il est en train de tourner
         * 
         * @brief Sortie : Le moteur se remet droit et on affiche sur le LCD un DNPW
         */
        if ((capteur_data == DNPW_G) && (moteur_data == DEPACEMENT_G)) {
            xQueueSend(queueMoteur_tx, &capteur_data, portMAX_DELAY);
            xQueueSend(queueLCD_tx, &capteur_data, portMAX_DELAY);
            moteur_data = 0;
            // A implementer quand plus clair
            // xQueueSend(queueESPNOW_tx, &capteur_data, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
            }
        if ((capteur_data == DNPW_D) && (moteur_data == DEPACEMENT_D)) {
            xQueueSend(queueLCD_tx, &moteur_data, portMAX_DELAY);
            moteur_data = 0;
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        /**
         * @brief Cas des alerte traité en local
         * Blind spot warning gauche et droite (BSW_D & BSW_G)
         * EEBL
         * Front colisions warning (low et avancée)
         * @brief Sortie vers le LCD
         */
        if (capteur_data == BSW_G || capteur_data == BSW_D || capteur_data == EEBL || 
            capteur_data == FCW_LOW || capteur_data == FCW_ADV) {
            xQueueSend(queueLCD_tx, &capteur_data, portMAX_DELAY);
        }
            
        /**
         * @brief Cas d'une colision critique
         * 
         */
        if (capteur_data == FCW_CRT) {
            int value = EEBL;
            xQueueSend(queueESPNOW_tx, &value, portMAX_DELAY);
            xQueueSend(queueLCD_tx, &capteur_data, portMAX_DELAY);
            xQueueSend(queueMoteur_tx, &capteur_data, portMAX_DELAY);
            xQueueSend(queueESPNOW_tx, &capteur_data, portMAX_DELAY);

            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        //Losrque les deux capteur latéraux sont OK et que on voit un objet à l'avant. 
        /**
         * @brief Envoie vers la queue moteur d'une demande de dépassement a gauche ou a droite 
         * @param DEPACEMENT_G Capteur AVG et ARG
         * @param DEPACEMENT_D Capteur ADG et ARD
         * @brief Le moteur enverra une informations de rotation au calculateur. Le cas ou un obstacle se présente est testé plus haut
         */
        if (capteur_data == DEPACEMENT_G || capteur_data == DEPACEMENT_D) {
            xQueueSend(queueMoteur_tx, &capteur_data, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

    }
