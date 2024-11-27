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

#include "hc_sr04.h"
#include "i2c_lcd.h"
#include "i2c_lcd2.h"
#include "espnow_example.h"
#include "nvs.h"





static const char *TAG = "main";

//declaration en externe pour accès depuis plusieurs fichier sources

sensor_data_t sensor_data;

void app_main() {

    ESP_LOGI(TAG, "Starting vehicle sensor system...");
    esp_err_t ret = nvs_flash_init();

    //accession a sensor_data

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
    }
    ESP_ERROR_CHECK( ret );
    

    example_wifi_init();
    printf("wifi initialized");
    example_espnow_send_param_t *send_param = SendingParamCreator();
    
    if(example_espnow_init(send_param) != ESP_OK){
        ESP_LOGE(TAG,"error during the initialization of espnow");        
    }
    printf("ESP now init");

    // Création de la file d'attente pour les données des capteurs
    sensor_queue = xQueueCreate(500, sizeof(sensor_data_t));
    if (sensor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor queue");
        return;
    }



    // Création des tâches
    BaseType_t xReturned;
    // Tâche d'affichage
    xReturned = xTaskCreate(
        display_task,
        "DISPLAY",
        DISPLAY_STACK_SIZE,
        &sensor_data,
        DISPLAY_TASK_PRIORITY,
        NULL
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        return;
    }
    
    // Tâche des capteurs
    xReturned = xTaskCreate(
        sensor_task,
        "SENSOR",
        SENSOR_STACK_SIZE * 3,
        &sensor_data,
        SENSOR_TASK_PRIORITY,
        NULL
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }


    ESP_LOGI(TAG, "All tasks created successfully");
}