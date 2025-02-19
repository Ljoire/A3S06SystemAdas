#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/**
 * @brief Liste des alertes
 *  
 */
#define BSW_G 1
#define BSW_D 2
#define DNPW_G 3
#define DNPW_D 4
#define EEBL 5
#define FCW_LOW 6
#define FCW_ADV 7
#define FCW_CRT 8
#define DEPACEMENT_G 9
#define DEPACEMENT_D 10
// A implementer
#define ROAD_LIGHT_LOC 11
#define ROAD_LIGHT_DIST 12

extern QueueHandle_t queueLCD_rx;
extern QueueHandle_t queueLCD_tx;
extern QueueHandle_t queueMoteur_rx;
extern QueueHandle_t queueMoteur_tx;
extern QueueHandle_t queueESPNOW_rx;
extern QueueHandle_t queueESPNOW_tx;

void task_calculateur(void *pvParameters) {
    //Variable d'accueil local
    uint8_t espnow_data, moteur_data, capteur_data;
    uint8_t moteur_received, capteur_received;

    while (1) {
        /**
         * @brief Réception d'un EEBL du véhicule distant on envoie l'info au LCD
         * @param EEBL Freinage brusque
         * 
         */
        if (xQueueReceive(queueESPNOW_rx, &espnow_data, portMAX_DELAY) == pdTRUE) {
            if (espnow_data == EEBL) {
                xQueueSend(queueLCD_tx, &espnow_data, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(300));
                continue;
                
            }
        }

        // Dépiler moteur et capteur séparément
        // Reprendre car un cas d'erreur est qu'il peut ne rien avoir car ça a été dépilé avant
        
        moteur_data = xQueueReceive(queueMoteur_rx, &moteur_received, portMAX_DELAY);
        capteur_data = xQueueReceive(queueLCD_rx, &capteur_received, portMAX_DELAY);
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

    }
