#include "hc_sr04.h"
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>
#include <portmacro.h>

static const char *TAG = "Sensor";

// Fonction pour vérifier si une mesure est valide
static bool is_valid_measurement(float distance) {
    return (distance >= 0 && distance <= 400);
}

int hc_sr04_init(hc_sr04_t *sensor) {
    // Configure Trigger pin as output
    if (sensor == NULL) {
        printf("Erreur: Le pointeur du capteur est NULL\n");
        return -1;
    }

    if (sensor->trigger_pin < 0 || sensor->echo_pin < 0) {
        printf("Erreur: Les broches TRIGGER ou ECHO ne sont pas correctement définies\n");
        return -2;
    }

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << sensor->trigger_pin),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("Erreur: Impossible de configurer la broche TRIGGER\n");
        return -3;
    }

    // Configure Echo pin as input
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
    
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("Erreur: Impossible de configurer la broche ECHO\n");
        return -4;
    }

    return 0;
}

float measure_distance_cm(hc_sr04_t *sensor) {
    
    if (sensor == NULL) {
        printf("Erreur: Le pointeur du capteur est NULL\n");
        return -1;
    }

    if (sensor->trigger_pin < 0 || sensor->echo_pin < 0) {
        printf("Erreur: Les broches TRIGGER ou ECHO ne sont pas valides\n");
        return -2;
    }


    gpio_set_level(sensor->trigger_pin, 1);
    ets_delay_us(10);
    gpio_set_level(sensor->trigger_pin, 0);

    // Mesurer la durée de l'écho avec timeout
    uint64_t timeout = esp_timer_get_time() + 30000; // 30ms timeout
    
    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) {
            printf("Erreur: Pas de signal de début d'écho (timeout)\n");
            return -3;
        }
    }
    uint64_t echo_start = esp_timer_get_time();
    
    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) {
            printf("Erreur: Signal d'écho trop long (timeout)\n");
            return -4;
        }
    }
    uint64_t echo_end = esp_timer_get_time();

    float distance_cm = (echo_end - echo_start) / 58.0;
    
    // Filtrer les valeurs aberrantes
    if (distance_cm < 2 || distance_cm > 400) {
        printf("Erreur: Distance mesurée hors des limites valides (%.2f cm)\n", distance_cm);
        return -5;
    }
    
    return distance_cm;
}

// Tâche de lecture des capteurs
static void sensor_task(void *pvParameters) {
    // Initialisation des capteurs
    //cast du pvParameters pour acceder au donnees 
    sensor_data_t sensor_data = (*sensor_data_t) pvParameters;
    
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

        // Envoi des données dans la file d'attente --> Les données sont passés par références
        //if (xQueueSend(sensor_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS) {
        //    ESP_LOGW(TAG, "Failed to send sensor data to queue");
        //}

        // Attendre la prochaine période
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}