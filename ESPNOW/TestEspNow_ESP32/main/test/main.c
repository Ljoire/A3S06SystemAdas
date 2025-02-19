
/**
 * @file main.c
 * @author JOIRE Léon leon.joire@esme.fr
 * @author ANDRIANJAFINDRADILO Sitraka Marc andrianjafindradilo-sitraka.marc@esme.fr
 * @author RAFFOUX Pol-Eloi pol-eloi.raffoux@esme.fr
 * @brief Main file of our end of study project about a V2V communication system
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright
 * 
 */

 #include <stdio.h>
 #include <stdbool.h>
 #include <esp_system.h>
 #include <esp_log.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "freertos/semphr.h"
 #include "driver/gpio.h"
 #include "esp_timer.h"
 #include "nvs_flash.h"
 
 #include "anglemort.h"
 #include "i2c_lcd.h"
 #include "i2c_lcd2.h"
 #include "espnow_handler.h"


void app_main(void) {
    ESP_LOGI(TAG, "Démarrage du système...");

    ESP_LOGI(TAG, "Starting vehicle sensor system...");
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
    }
    ESP_ERROR_CHECK(ret);

    example_wifi_init();
    printf("wifi initialized");
    example_espnow_send_param_t *send_param = SendingParamCreator();
    
    if (example_espnow_init(send_param) != ESP_OK) {
        ESP_LOGE(TAG, "error during the initialization of espnow");        
    }
    printf("ESP now init");

    // Création de la file d'attente pour les données des capteurs
    sensor_queue = xQueueCreate(2, sizeof(sensor_data_t));
    if (sensor_queue == NULL) {
        ESP_LOGE(TAG, "Échec de création de la file d'attente des capteurs");
        return;
    }

    // Création du mutex I2C
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Échec de création du mutex I2C");
        return;
    }

    // Création des tâches
    BaseType_t xReturned;
    
    // Tâche des capteurs
    xReturned = xTaskCreate(
        sensor_task,
        "SENSOR",
        SENSOR_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Échec de création de la tâche capteur");
        return;
    }

    // Tâche d'affichage
    xReturned = xTaskCreate(
        display_task,
        "DISPLAY",
        DISPLAY_STACK_SIZE,
        NULL,                                                                               
        DISPLAY_TASK_PRIORITY,
        NULL
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Échec de création de la tâche affichage");
        return;
    }

    ESP_LOGI(TAG, "Toutes les tâches ont été créées avec succès");

    while (1) {
        uint8_t alert = detect_alert();
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            lcd_clear();
            if (alert == 1) {
                lcd_set_cursor(0, 0);
                lcd_print("Angle mort gauche");
                ESP_LOGI(TAG, "Alert: Angle mort gauche");
            } else if (alert == 2) {
                lcd_set_cursor(0, 0);
                lcd_print("Angle mort droit");
                ESP_LOGI(TAG, "Alert: Angle mort droit");
            }
            xSemaphoreGive(i2c_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Rafraîchir l'affichage toutes les 500ms
    }
}
