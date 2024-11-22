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

// Définition des priorités des tâches
#define SENSOR_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)
#define DISPLAY_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)

// Taille des piles pour les tâches
#define SENSOR_STACK_SIZE       (configMINIMAL_STACK_SIZE * 2)
#define DISPLAY_STACK_SIZE      (configMINIMAL_STACK_SIZE * 2)

// Structure pour les données des capteurs
typedef struct {
    float dist_av;
    float dist_g;
    float dist_d;
    float dist_avg;
    float dist_avd;
    float dist_ar;
    bool data_ready;
} sensor_data_t;

// Variables globales
static QueueHandle_t sensor_queue;
static SemaphoreHandle_t i2c_mutex;
static const char *TAG = "VEHICLE";

// Fonction pour vérifier si une mesure est valide
static bool is_valid_measurement(float distance) {
    return (distance >= 0 && distance <= 400);
}

// Tâche de lecture des capteurs
static void sensor_task(void *pvParameters) {
    // Initialisation des capteurs
    hc_sr04_t sensor_av = {.trigger_pin = TRIGGER_GPIO_AV, .echo_pin = ECHO_GPIO_AV};
    hc_sr04_t sensor_g = {.trigger_pin = TRIGGER_GPIO_G, .echo_pin = ECHO_GPIO_G};
    hc_sr04_t sensor_d = {.trigger_pin = TRIGGER_GPIO_D, .echo_pin = ECHO_GPIO_D};
    hc_sr04_t sensor_avg = {.trigger_pin = TRIGGER_GPIO_AVG, .echo_pin = ECHO_GPIO_AVG};
    hc_sr04_t sensor_avd = {.trigger_pin = TRIGGER_GPIO_AVD, .echo_pin = ECHO_GPIO_AVD};
    hc_sr04_t sensor_ar = {.trigger_pin = TRIGGER_GPIO_AR, .echo_pin = ECHO_GPIO_AR};

    // Initialisation de tous les capteurs
    hc_sr04_init(&sensor_av);
    hc_sr04_init(&sensor_g);
    hc_sr04_init(&sensor_d);
    hc_sr04_init(&sensor_avg);
    hc_sr04_init(&sensor_avd);
    hc_sr04_init(&sensor_ar);

    sensor_data_t sensor_data;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // Lecture des capteurs avec délai entre chaque mesure
        sensor_data.dist_av = measure_distance_cm(&sensor_av);
        vTaskDelay(pdMS_TO_TICKS(10));
        sensor_data.dist_g = measure_distance_cm(&sensor_g);
        vTaskDelay(pdMS_TO_TICKS(10));
        sensor_data.dist_d = measure_distance_cm(&sensor_d);
        vTaskDelay(pdMS_TO_TICKS(10));
        sensor_data.dist_avg = measure_distance_cm(&sensor_avg);
        vTaskDelay(pdMS_TO_TICKS(10));
        sensor_data.dist_avd = measure_distance_cm(&sensor_avd);
        vTaskDelay(pdMS_TO_TICKS(10));
        sensor_data.dist_ar = measure_distance_cm(&sensor_ar);

        sensor_data.data_ready = true;

        // Envoi des données dans la file d'attente
        if (xQueueSend(sensor_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW(TAG, "Failed to send sensor data to queue");
        }

        // Attendre la prochaine période
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

// Tâche d'affichage
static void display_task(void *pvParameters) {
    sensor_data_t sensor_data;
    char buffer[21]; // Buffer assez grand pour LCD 4x20

    // Initialisation des LCD avec protection mutex
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        lcd_init();
        lcd2_init();
        lcd_backlight(true);
        lcd2_backlight(true);
        xSemaphoreGive(i2c_mutex);
    }
    uint8_t DataToEspNow = 0x00;
    bool is_DATA2send = false;
    uint8_t DataFromEspNow = 0x00;
    bool is_dataFromESP = false;
    while (1) {
        //si reception ESPNOW
        if(xQueueReceive(receive_calback_queue,&DataFromEspNow,ESPNOW_MAXDELAY) == pdTRUE){
            ESP_LOGI(TAG,"The data received is : %u",DataFromEspNow);
            is_dataFromESP = true;
        }

        if (xQueueReceive(sensor_queue, &sensor_data, pdMS_TO_TICKS(500)) == pdPASS) {
            // Prendre le mutex I2C
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                // Effacement des écrans
                lcd_clear();
                lcd2_clear();
                vTaskDelay(pdMS_TO_TICKS(10));

                // LCD2 - Ligne 1 (AV)
                lcd2_set_cursor(0, 0);
                if (is_valid_measurement(sensor_data.dist_av) || is_dataFromESP) {// && compteur = 0 || reception ESPNOW
                    if (is_dataFromESP && (DataFromEspNow & 0x02)){ //si le bit n°1 est a 1
                        snprintf(buffer, sizeof(buffer), "Att. Freinez !");
                    }
                    else if (sensor_data.dist_av > 5) {
                        snprintf(buffer, sizeof(buffer), "AV: %.1fcm", sensor_data.dist_av);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Att. F. Urgence !");
                        is_DATA2send = true;
                        DataToEspNow = DataToEspNow | 0x02;
                    }
                    lcd2_print(buffer);
                }
                // compteur --;
                // LCD1 - Ligne 1 (G)
                lcd_set_cursor(0, 0);
                if (is_valid_measurement(sensor_data.dist_g)) {
                    if (sensor_data.dist_g > 10) {
                        snprintf(buffer, sizeof(buffer), "G: %.1fcm", sensor_data.dist_g);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Angle mort G !");
                    }
                    lcd_print(buffer);
                }

                // LCD1 - Ligne 2 (D)
                lcd_set_cursor(1, 0);
                if (is_valid_measurement(sensor_data.dist_d)) {
                    if (sensor_data.dist_d > 10) {
                        snprintf(buffer, sizeof(buffer), "D: %.1fcm", sensor_data.dist_d);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Angle mort D !");
                    }
                    lcd_print(buffer);
                }

                // LCD2 - Ligne 2 (AVG)
                lcd2_set_cursor(1, 0);
                if (is_valid_measurement(sensor_data.dist_av) && 
                    is_valid_measurement(sensor_data.dist_avg) && 
                    is_valid_measurement(sensor_data.dist_g)) {
                    if (sensor_data.dist_av > 5 && 
                        sensor_data.dist_avg > 20 && 
                        sensor_data.dist_g > 10) {
                        snprintf(buffer, sizeof(buffer), "AVG: %.1fcm", sensor_data.dist_avg);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Dep. Non Aut. !");
                        //mise du flag d'envoie a 1. Est nettoyé lorsque la donnée est envoyé
                        is_DATA2send = true;
                    }
                    lcd2_print(buffer);
                }

                // LCD2 - Ligne 3 (AVD)
                lcd2_set_cursor(2, 0);
                if (is_valid_measurement(sensor_data.dist_av) && 
                    is_valid_measurement(sensor_data.dist_avd) && 
                    is_valid_measurement(sensor_data.dist_d)) {
                    if (sensor_data.dist_av > 5 && 
                        sensor_data.dist_avd > 20 && 
                        sensor_data.dist_d > 10) {
                        snprintf(buffer, sizeof(buffer), "AVD: %.1fcm", sensor_data.dist_avd);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Dep. Non Aut. !");
                    }
                    lcd2_print(buffer);
                }

                // LCD2 - Ligne 4 (AR)
                lcd2_set_cursor(3, 0);
                if (is_valid_measurement(sensor_data.dist_ar)) {
                    if (sensor_data.dist_ar > 10) {
                        snprintf(buffer, sizeof(buffer), "AR: %.1fcm", sensor_data.dist_ar);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Att. AR !");
                    }
                    lcd2_print(buffer);
                }

                // Libérer le mutex I2C
                xSemaphoreGive(i2c_mutex);
            }
            // si des données importante sont à envoyer, elle le sont ici 
            //on envoie en ESP NOW sur l'autre LCD. Attention ! Sera envoyé seulement quand en unicast
            if(is_DATA2send == true){
                //remise à false du flag 
                is_DATA2send = false;
                ESP_LOGI(TAG,"des données vont être envoyés");
                if (xQueueSend(data_queue_2other_ESPNOW,DataToEspNow,ESPNOW_MAXDELAY) != pdTRUE){
                    ESP_LOGW(TAG, "Send send queue fail");
                }
                // remise à 0 de l'octet d'alerte
                DataToEspNow = 0x00;
            }
        }
    }
}

void app_main() {
    ESP_LOGI(TAG, "Starting vehicle sensor system...");
    esp_err_t ret = nvs_flash_init();

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
    sensor_queue = xQueueCreate(2, sizeof(sensor_data_t));
    if (sensor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor queue");
        return;
    }

    // Création du mutex I2C
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
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
        ESP_LOGE(TAG, "Failed to create sensor task");
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
        ESP_LOGE(TAG, "Failed to create display task");
        return;
    }

    ESP_LOGI(TAG, "All tasks created successfully");
}