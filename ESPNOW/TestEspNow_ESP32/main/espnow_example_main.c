/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include "espnow_example.h"
#include "ultrasonic.h"


#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "freertos/timers.h"

#include "esp_log.h"
#include "nvs_flash.h"



static const char *TAG = "espnow_example";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    init_ultrasonic_sensor();
    example_wifi_init();
    printf("wifi initialized");
    example_espnow_init();
    printf("ESP now init");
    while (true) {
        float distance1 = measure_distance(TRIG_PIN_1, ECHO_PIN_1);
        float distance2 = measure_distance(TRIG_PIN_2, ECHO_PIN_2);

        if(distance1 >= 0 && distance1 < CRITICAL_DISTANCE_CM){
            ESP_LOGI(TAG, "Critical distance detected: %.2f cm", distance1);
            //esp_now_send(NULL, (uint8_t *)&distance1, sizeof(distance1));
            /*xTaskCreate(example_espnow_task, "example_espnow_task", 4096, send_param, 4,&ESPNOW_data)*/
        } else if (distance2 >= 0 && distance2 < CRITICAL_DISTANCE_CM) {
            ESP_LOGI(TAG, "Critical distance detected: %.2f cm", distance2);
            //esp_now_send(NULL, (uint8_t *)&distance2, sizeof(distance2));
           /* xTaskCreate(example_espnow_task, "example_espnow_task", 4096, send_param, 4,&ESPNOW_data)*/
        } 
        else{
            ESP_LOGI(TAG, "No critical object detected. Distance1: %.2f cm, Distance2: %.2f cm", distance1, distance2);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}