#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "lcd_i2c.h"

// Définition des broches pour les deux capteurs
#define TRIGGER_PIN_LEFT GPIO_NUM_5    // Trigger du capteur gauche
#define ECHO_PIN_LEFT    GPIO_NUM_18   // Echo du capteur gauche
#define TRIGGER_PIN_RIGHT GPIO_NUM_17  // Trigger du capteur droit
#define ECHO_PIN_RIGHT   GPIO_NUM_16   // Echo du capteur droit

#define TRIGGER_PULSE_DURATION_US 10
#define MEASUREMENT_TIMEOUT_US 23529
#define SOUND_SPEED_CM_US 0.0343

// Structure pour stocker les temps de mesure de chaque capteur
typedef struct {
    volatile uint64_t start_time;
    volatile uint64_t end_time;
    volatile bool measurement_done;
    int gpio_pin;
} ultrasonic_sensor_t;

// Variables pour les deux capteurs
static ultrasonic_sensor_t sensor_left = {0, 0, false, ECHO_PIN_LEFT};
static ultrasonic_sensor_t sensor_right = {0, 0, false, ECHO_PIN_RIGHT};

// Gestionnaire d'interruption pour les deux capteurs
static void IRAM_ATTR echo_isr_handler(void* arg) {
    ultrasonic_sensor_t* sensor = (ultrasonic_sensor_t*)arg;
    int level = gpio_get_level(sensor->gpio_pin);
    
    if (level == 1) {
        sensor->start_time = esp_timer_get_time();
    } else {
        sensor->end_time = esp_timer_get_time();
        sensor->measurement_done = true;
    }
}

void ultrasonic_init(void) {
    // Configuration des pins Trigger
    gpio_config_t io_conf_trigger = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN_LEFT) | (1ULL << TRIGGER_PIN_RIGHT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_trigger);

    // Configuration des pins Echo
    gpio_config_t io_conf_echo = {
        .pin_bit_mask = (1ULL << ECHO_PIN_LEFT) | (1ULL << ECHO_PIN_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf_echo);

    // Installation du service d'interruption
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_PIN_LEFT, echo_isr_handler, &sensor_left);
    gpio_isr_handler_add(ECHO_PIN_RIGHT, echo_isr_handler, &sensor_right);
}

float measure_distance(int trigger_pin, ultrasonic_sensor_t* sensor) {
    sensor->measurement_done = false;
    
    // Envoie de l'impulsion trigger
    gpio_set_level(trigger_pin, 1);
    esp_rom_delay_us(TRIGGER_PULSE_DURATION_US);
    gpio_set_level(trigger_pin, 0);

    // Attente de la fin de la mesure
    int timeout_count = 0;
    while (!sensor->measurement_done && timeout_count < 100) {
        vTaskDelay(1);
        timeout_count++;
    }

    if (!sensor->measurement_done) {
        return -1.0f; // Erreur de mesure
    }

    // Calcul de la distance
    uint64_t time_diff = sensor->end_time - sensor->start_time;
    float distance = (time_diff * SOUND_SPEED_CM_US) / 2;
    
    return distance;
}

void app_main(void) {
    // Initialisation du LCD
    lcd_init();
    lcd_clear();
    
    // Initialisation des capteurs ultrasoniques
    ultrasonic_init();
    
    char left_str[20];
    char right_str[20];
    float distance_left, distance_right;
    
    while (1) {
        // Mesure de la distance pour le capteur gauche
        distance_left = measure_distance(TRIGGER_PIN_LEFT, &sensor_left);
        vTaskDelay(pdMS_TO_TICKS(50)); // Petit délai entre les mesures
        
        // Mesure de la distance pour le capteur droit
        distance_right = measure_distance(TRIGGER_PIN_RIGHT, &sensor_right);
        
        lcd_clear();
        
        // Affichage pour le capteur gauche
        if (distance_left >= 0) {
            snprintf(left_str, sizeof(left_str), "Gauche: %.1f cm", distance_left);
        } else {
            snprintf(left_str, sizeof(left_str), "Gauche: Collision !");
        }
        lcd_write_string(0, 0, left_str);
        
        // Affichage pour le capteur droit
        if (distance_right >= 0) {
            snprintf(right_str, sizeof(right_str), "Droit: %.1f cm", distance_right);
        } else {
            snprintf(right_str, sizeof(right_str), "Droit: Collision !");
        }
        lcd_write_string(1, 0, right_str);
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Délai de 500ms entre chaque cycle de mesure
    }
}