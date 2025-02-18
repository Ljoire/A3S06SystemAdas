#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern QueueHandle_t queueLCD_rx;
extern QueueHandle_t queueLCD_tx;
extern QueueHandle_t queueMoteur_rx;
extern QueueHandle_t queueMoteur_tx;
extern QueueHandle_t queueESPNOW_rx;
extern QueueHandle_t queueESPNOW_tx;

void task_process_data(void *pvParameters) {
    int espnow_data, moteur_data, capteur_data;
    BaseType_t moteur_received, capteur_received;

    while (1) {
        // Dépiler ESPNOW
        if (xQueueReceive(queueESPNOW_rx, &espnow_data, portMAX_DELAY) == pdTRUE) {
            if (espnow_data == 5) {
                xQueueSend(queueLCD_tx, &espnow_data, portMAX_DELAY);
            }
        }

        // Dépiler moteur et capteur séparément
        moteur_received = xQueueReceive(queueMoteur_rx, &moteur_data, portMAX_DELAY);
        capteur_received = xQueueReceive(queueLCD_rx, &capteur_data, portMAX_DELAY);

        if (moteur_received == pdTRUE && capteur_received == pdTRUE) {
            if (capteur_data == 9) {
                if (moteur_data == 3) {
                    xQueueSend(queueLCD_tx, &moteur_data, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    continue;
                }
                xQueueSend(queueMoteur_tx, &capteur_data, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(300));
                continue;
            }
            
            if (capteur_data == 10) {
                if (moteur_data == 4) {
                    xQueueSend(queueLCD_tx, &moteur_data, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    continue;
                }
                xQueueSend(queueMoteur_tx, &capteur_data, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(300));
                continue;
            }
            
            if (capteur_data == 1 || capteur_data == 2 || capteur_data == 5 || 
                capteur_data == 6 || capteur_data == 7) {
                xQueueSend(queueLCD_tx, &capteur_data, portMAX_DELAY);
            }
            
            if (capteur_data == 3 || capteur_data == 4 || capteur_data == 8) {
                if (capteur_data == 8) {
                    int value = 5;
                    xQueueSend(queueESPNOW_tx, &value, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    continue;
                }
                xQueueSend(queueLCD_tx, &capteur_data, portMAX_DELAY);
                xQueueSend(queueMoteur_tx, &capteur_data, portMAX_DELAY);
                xQueueSend(queueESPNOW_tx, &capteur_data, portMAX_DELAY);
            }
            
            if (capteur_data == 9 || capteur_data == 10) {
                xQueueSend(queueMoteur_tx, &capteur_data, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(300));
                continue;
            }
        }

        // Rendre la main après chaque boucle
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
