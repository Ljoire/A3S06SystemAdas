#include "anglemort.h"
#include "calculateur.h"
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#define ALERT_DISTANCE_30 30 
#define ALERT_DISTANCE_20 20 
#define ALERT_DISTANCE_10 10

static const char *TAG = "ANGLEMORT";

uint16_t global_distances[6] = {0, 0, 0, 0, 0, 0};

extern QueueHandle_t queueCapteur_rx; 

void hc_sr04_init(hc_sr04_t *sensor) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << sensor->trigger_pin),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
    gpio_config(&io_conf);
}




uint16_t measure_distance_cm(hc_sr04_t *sensor) {
    gpio_set_level(sensor->trigger_pin, 1);
    ets_delay_us(10);
    gpio_set_level(sensor->trigger_pin, 0);

    uint64_t timeout = esp_timer_get_time() + 30000;
    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) return 0;
    }
    uint64_t echo_start = esp_timer_get_time();

    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) return 0;
    }
    uint64_t echo_end = esp_timer_get_time();

    uint16_t distance_cm = (uint16_t)((echo_end - echo_start) / 58.0);
    if (distance_cm < 2 || distance_cm > 400) return 0;
    return distance_cm;
}

ALERT_DATA_FORMAT alert_code = CAPTEUR_NO_ERROR;

void detect_alert(hc_sr04_t *capteurs) {

    for (int i = 0; i < 6; i++) {
        global_distances[i] = measure_distance_cm(&capteurs[i]);
    }

    if (global_distances[2] <= ALERT_DISTANCE_30 && global_distances[2] > ALERT_DISTANCE_20 &&
        global_distances[5] <= ALERT_DISTANCE_30 && global_distances[5] > ALERT_DISTANCE_20) {
        alert_code = 1;
    } else if (global_distances[2] <= ALERT_DISTANCE_20 && global_distances[2] > ALERT_DISTANCE_10 &&
               global_distances[5] <= ALERT_DISTANCE_20 && global_distances[5] > ALERT_DISTANCE_10) {
        alert_code = 3;
    } else if (global_distances[2] <= ALERT_DISTANCE_10 && global_distances[2] > 2 &&
        global_distances[5] <= ALERT_DISTANCE_10 && global_distances[5] > 2) {
        alert_code = 5;
    }

    if (global_distances[0] <= ALERT_DISTANCE_30) {
        alert_code = 7;
    }
    if (global_distances[1] <= ALERT_DISTANCE_30) {
        alert_code = 9;
    }

    if (global_distances[1] <= ALERT_DISTANCE_30 && global_distances[3] <= ALERT_DISTANCE_30) {
        alert_code = 11;
    }
    if (global_distances[0] <= ALERT_DISTANCE_30 && global_distances[3] <= ALERT_DISTANCE_30) {
        alert_code = 13;
    }
    ESP_LOGI(TAG,"Le code renvoyé est :%d",alert_code);
    if (alert_code != CAPTEUR_NO_ERROR) {
        if (queueCapteur_rx != NULL) {
            if (xQueueSend(queueCapteur_rx, &alert_code, pdMS_TO_TICKS(200)) != pdPASS) {
                ESP_LOGE("Queue", "Failed to send alert_code to queueCapteur_rx");
            }
            ESP_LOGI(TAG,"envoie d'une info");
        } else {
            ESP_LOGE("Queue", "queueCapteur_rx is NULL");
        }   
    }
}



void sensor_task(void *pvParameters) {
    hc_sr04_t toto[] = {
        {TRIGGER_GPIO_ARG, ECHO_GPIO_ARG},
        {TRIGGER_GPIO_ARD, ECHO_GPIO_ARD},
        {TRIGGER_GPIO_AV, ECHO_GPIO_AV},
        {TRIGGER_GPIO_AVG, ECHO_GPIO_AVG},
        {TRIGGER_GPIO_AVD, ECHO_GPIO_AVD},
        {TRIGGER_GPIO_AR, ECHO_GPIO_AR}  
    };

    ESP_LOGI(TAG,"Initialisation des capteur");
    for (int i = 0; i < CAPTEUR_NUMBER; i++) {
        hc_sr04_init(&toto[i]);
    }
    while (1) {
        ESP_LOGE(TAG,"Prend la main");
        detect_alert(&toto);
        ESP_LOGE(TAG,"Rend la main");
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}
