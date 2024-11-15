#include <stdio.h>
#include <stdbool.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "hc_sr04.h"
#include "i2c_lcd.h"

void app_main() {
    // Définir les capteurs
    hc_sr04_t sensor_left = {
        .trigger_pin = TRIGGER_GPIO_LEFT,
        .echo_pin = ECHO_GPIO_LEFT
    };
    
    hc_sr04_t sensor_right = {
        .trigger_pin = TRIGGER_GPIO_RIGHT,
        .echo_pin = ECHO_GPIO_RIGHT
    };

    // Initialiser les capteurs et le LCD
    hc_sr04_init(&sensor_left);
    hc_sr04_init(&sensor_right);
    
    // Initialiser le LCD et attendre qu'il soit prêt
    lcd_init();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Activer le rétroéclairage
    lcd_backlight(true);
    
    char line1[17];  // 16 caractères + terminateur nul
    char line2[17];

    while (1) {
        // Mesurer les distances
        float dist_left = measure_distance_cm(&sensor_left);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        float dist_right = measure_distance_cm(&sensor_right);

        // Effacer l'écran
        lcd_clear();
        
        // Première ligne - capteur gauche
        lcd_set_cursor(0, 1);
        snprintf(line1, sizeof(line1), "Gauche:%5.1fcm", 
                dist_left < 0 ? 0.0 : dist_left);
        lcd_print(line1);
        
        // Deuxième ligne - capteur droit
        lcd_set_cursor(1, 0);
        snprintf(line2, sizeof(line2), "Droit:%6.1fcm", 
                dist_right < 0 ? 0.0 : dist_right);
        lcd_print(line2);

        // Attendre avant la prochaine mesure
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}