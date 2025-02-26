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

#define DIST_MIN_DETECT 2



static const char *TAG = "ANGLEMORT";

uint16_t global_distances[6] = {0, 0, 0, 0, 0, 0};

static uint8_t av, ar, arg, ard, avg, avd;


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


// Déclaration des variables globales pour éviter leur redéclaration à chaque boucle

ALERT_DATA_FORMAT update_alert_code(uint16_t * global_distances, ALERT_DATA_FORMAT alert_code) {
    // Mise à jour des variables (hors du while(true), mais exécutée à chaque appel)
    arg = global_distances[CPT_ARG];
    ard = global_distances[CPT_ARD];
    av = global_distances[CPT_AV];
    avg = global_distances[CPT_AVG];
    avd = global_distances[CPT_AVD];
    ar = global_distances[CPT_AR];

    alert_code = CAPTEUR_NO_ERROR; // Valeur par défaut

    // ⚠️ Freinage brusque EEBL (avant/arrière)
    if (av <= ALERT_DISTANCE_30 && ar <= ALERT_DISTANCE_30) {
        if (av > ALERT_DISTANCE_20 && ar > ALERT_DISTANCE_20) {
            alert_code = CAPTEUR_EEBL_MID;
        } else if (av > ALERT_DISTANCE_10 && ar > ALERT_DISTANCE_10) {
            alert_code = CAPTEUR_EEBL_HIGH;
        } else if (av > DIST_MIN_DETECT && ar > DIST_MIN_DETECT) {
            alert_code = CAPTEUR_EEBL_CRIT;
        }
    }

    // ⚠️ Alerte collision frontale (FCW)
    if (av <= ALERT_DISTANCE_30 && av > ALERT_DISTANCE_20) {
        alert_code = CAPTEUR_FCW_HIGH;
    } else if (av <= ALERT_DISTANCE_20) {
        alert_code = CAPTEUR_FCW_CRIT;
    }

    // ⚠️ Détection des angles morts
    if (arg <= ALERT_DISTANCE_30) {
        alert_code = CAPTEUR_BSW_GAUCHE;
    }
    if (ard <= ALERT_DISTANCE_30) {
        alert_code = CAPTEUR_BSW_DROITE;
    }

    // ⚠️ Détection de non-priorité (Danger croisement)
    if (ard <= ALERT_DISTANCE_30 && avg <= ALERT_DISTANCE_30) {
        alert_code = CAPTEUR_DNPW_G;
    } else if (arg <= ALERT_DISTANCE_30 && avg <= ALERT_DISTANCE_30) {
        alert_code = CAPTEUR_DNPW_D;
    }
    return alert_code;
}




void sensor_task(void *pvParameters) {
    static hc_sr04_t capteurs[] = {
        {TRIGGER_GPIO_ARG, ECHO_GPIO_ARG},
        {TRIGGER_GPIO_ARD, ECHO_GPIO_ARD},
        {TRIGGER_GPIO_AV, ECHO_GPIO_AV},
        {TRIGGER_GPIO_AVG, ECHO_GPIO_AVG},
        {TRIGGER_GPIO_AVD, ECHO_GPIO_AVD},
        {TRIGGER_GPIO_AR, ECHO_GPIO_AR}  
    };
    ALERT_DATA_FORMAT alert_code = CAPTEUR_NO_ERROR;
    ESP_LOGI(TAG, "Initialisation des capteurs");
    for (int i = 0; i < CAPTEUR_NUMBER; i++) {
        hc_sr04_init(&capteurs[i]);
    }

    while (1) {
        for (int i = 0; i < 6; i++) {
            global_distances[i] = measure_distance_cm(&capteurs[i]);
        }
        alert_code = update_alert_code(global_distances,alert_code);
        if (alert_code != CAPTEUR_NO_ERROR) {
            if (queueCapteur_rx != NULL) {
                if (xQueueSend(queueCapteur_rx, &alert_code, pdMS_TO_TICKS(200)) != pdPASS) {
                    ESP_LOGE("Queue", "Failed to send alert_code to queueCapteur_rx");
                }
                ESP_LOGI(TAG,"Le code renvoyé est :%d",alert_code);
                //ESP_LOGI(TAG,"envoie d'une info");
            } else {
                ESP_LOGE("Queue", "queueCapteur_rx is NULL");
            }   
        }
        // Passage correct du tableau

        vTaskDelay(pdMS_TO_TICKS(200));  // Rafraîchissement plus fréquent (modifiable)
    }
}
