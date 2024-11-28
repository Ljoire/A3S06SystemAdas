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

#include "hc_sr04.h"
#include "i2c_lcd.h"
#include "i2c_lcd2.h"
#include "espnow_handler.h"

// MAC Address du récepteur (à modifier selon l'adresse de la carte de ton collègue)

#define SENSOR_TASK_PRIORITY    (tskIDLE_PRIORITY + 3)
#define DISPLAY_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define ESPNOW_TASK_PRIORITY   (tskIDLE_PRIORITY + 4)

#define SENSOR_STACK_SIZE       (configMINIMAL_STACK_SIZE * 2)
#define DISPLAY_STACK_SIZE      (configMINIMAL_STACK_SIZE * 2)
#define ESPNOW_STACK_SIZE      (configMINIMAL_STACK_SIZE * 3)

static const char *TAG = "MAIN";
static SemaphoreHandle_t i2c_mutex;

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

// Files d'attente
static QueueHandle_t sensor_queue;

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

    // Initialisation
    hc_sr04_init(&sensor_av);
    hc_sr04_init(&sensor_g);
    hc_sr04_init(&sensor_d);
    hc_sr04_init(&sensor_avg);
    hc_sr04_init(&sensor_avd);
    hc_sr04_init(&sensor_ar);

    sensor_data_t sensor_data;
    // float last_distance_av = -1;
    // bool alert_sent = false;
	static int last_state = -1;  // -1: non initialisé, 0: distance critique, 1: système actif
	static uint32_t last_send_time = 0;
	#define MIN_SEND_INTERVAL_MS 500  // Intervalle minimum entre chaque envoi de message


    while (1) {
        // Lecture du capteur avant
        sensor_data.dist_av = measure_distance_cm(&sensor_av);
        
        // Vérification pour l'envoi d'alerte
		if (is_valid_measurement(sensor_data.dist_av)) {
			if (sensor_data.dist_av <= 5) {
				// Distance critique - envoyer le message d'alerte
				if (last_state != 0) {
					espnow_send_message(receiver_mac, "Att. F. Urg !");
					last_state = 0;
				}
			} else {
				// Distance > 5cm - envoyer un message vide pour effacer son LCD
				if (last_state != 1) {
					espnow_send_message(receiver_mac, "");  // Message vide
					last_state = 1;
				}
			}
		}

        // Lecture des autres capteurs
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

        if (xQueueSend(sensor_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW(TAG, "Échec de l'envoi des données capteur");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Tâche d'affichage local
static void display_task(void *pvParameters) {
    sensor_data_t sensor_data;
    char buffer[21];

    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        lcd_init();
        lcd2_init();
        lcd_backlight(true);
        lcd2_backlight(true);
        xSemaphoreGive(i2c_mutex);
    }

    while (1) {
        if (xQueueReceive(sensor_queue, &sensor_data, pdMS_TO_TICKS(500)) == pdPASS) {
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                // Effacement des écrans
                lcd_clear();
                lcd2_clear();
                vTaskDelay(pdMS_TO_TICKS(10));

                // LCD2 - Ligne 1 (AV)
                lcd2_set_cursor(0, 0);
                if (is_valid_measurement(sensor_data.dist_av)) {
                    if (sensor_data.dist_av > 5) {
                        snprintf(buffer, sizeof(buffer), "AV: %.1fcm", sensor_data.dist_av);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Att. F. Urgence !");
                    }
                    lcd2_print(buffer);
                }

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

                xSemaphoreGive(i2c_mutex);
            }
        }
    }
}

// Tâche de réception ESP-NOW modifiée
static void espnow_display_task(void *pvParameter) {
    espnow_message_t msg;
    char buffer[33];  // Augmenté à 33 pour accueillir MAX_MESSAGE_LENGTH + null terminator

    while (1) {
        if (xQueueReceive(espnow_receive_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                // Afficher le message reçu sur l'écran LCD
                lcd_clear();
                lcd_set_cursor(0, 0);
                strncpy(buffer, "Message recu:", sizeof(buffer) - 1);
                lcd_print(buffer);
                
                lcd_set_cursor(1, 0);
                // Limiter la longueur du message à 16 caractères pour l'écran LCD
                strncpy(buffer, msg.message, 16);
                buffer[16] = '\0';  // Assurer la terminaison
                lcd_print(buffer);
                
                xSemaphoreGive(i2c_mutex);
            }
        }
    }
}

void app_main(void) {
    
    ESP_LOGI(TAG, "Démarrage du système...");

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

    // Tâche de réception ESP-NOW
    xReturned = xTaskCreate(
        espnow_display_task,
        "ESPNOW_DISPLAY",
        ESPNOW_STACK_SIZE,
        NULL,
        ESPNOW_TASK_PRIORITY,
        NULL
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Échec de création de la tâche ESP-NOW");
        return;
    }

    ESP_LOGI(TAG, "Toutes les tâches ont été créées avec succès");
}