/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This code is our final year project
   School : ESME
   Location : France, Ivry-sur-Seine
   Author : 
        ANDRIANJAFINDRADILO Sitraka Marc
        RAFFOUX Pol-Eloi
        JOIRE Leon
    The project is an ADAS system with communication in v2v (Vehicule 2 Vehicule)

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

    
    example_wifi_init();
    printf("wifi initialized");
    example_espnow_send_param_t *send_param = SendingParamCreator();
    
    if(example_espnow_init(send_param) != ESP_OK){
        ESP_LOGE(TAG,"error during the initialization of espnow");        
    }
    printf("ESP now init");
    

    init_ultrasonic_sensor();

    //array for stock the result and his pointer
    /*
    u_int8_t result[MAX_PAYLOAD_SIZE];
    u_int8_t *pResult = &result[0];
    */

    while (true) {
        float distance1 = measure_distance();

        if(distance1 <= 20.00){
            //if we find a critacal distance we copy the value inside the table
            //memcpy(result,&distance1,sizeof(MAX_PAYLOAD_SIZE));
            ESP_LOGI(TAG, "Critical distance detected: %.2f cm", distance1);
            
            if(send_param->state == 1){
                espnow_datasending(send_param, (uint8_t*) "D2crit",send_param->dest_mac);
                ESP_LOGI(TAG, "will be sent");
            }    
            //we post an order to send data via xQueueSend
        }        
        else{
            ESP_LOGI(TAG, "No critical object detected. Distance1: %.2f cm", distance1);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}