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
#include "i2c_lcd2.h"

// Fonction pour vérifier si une mesure est valide
bool is_valid_measurement(float distance) {
    return (distance >= 0 && distance <= 400);  // Distance valide entre 0 et 400 cm
}

void app_main() {
    // Définition de tous les capteurs
    hc_sr04_t sensor_g = {       // Capteur Gauche
        .trigger_pin = TRIGGER_GPIO_G,
        .echo_pin = ECHO_GPIO_G
    };
    
    hc_sr04_t sensor_d = {       // Capteur Droite
        .trigger_pin = TRIGGER_GPIO_D,
        .echo_pin = ECHO_GPIO_D
    };

    hc_sr04_t sensor_av = {      // Capteur Avant
        .trigger_pin = TRIGGER_GPIO_AV,
        .echo_pin = ECHO_GPIO_AV
    };

    hc_sr04_t sensor_avg = {     // Capteur Avant Gauche
        .trigger_pin = TRIGGER_GPIO_AVG,
        .echo_pin = ECHO_GPIO_AVG
    };

    hc_sr04_t sensor_avd = {     // Capteur Avant Droit
        .trigger_pin = TRIGGER_GPIO_AVD,
        .echo_pin = ECHO_GPIO_AVD
    };

    hc_sr04_t sensor_ar = {      // Capteur Arrière
        .trigger_pin = TRIGGER_GPIO_AR,
        .echo_pin = ECHO_GPIO_AR
    };

    // Initialisation de tous les capteurs
    hc_sr04_init(&sensor_g);
    hc_sr04_init(&sensor_d);
    hc_sr04_init(&sensor_av);
    hc_sr04_init(&sensor_avg);
    hc_sr04_init(&sensor_avd);
    hc_sr04_init(&sensor_ar);
    
    // Initialisation des deux LCD
    lcd_init();     // LCD 2x16
    lcd2_init();    // LCD 4x20
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Activation des rétroéclairages
    lcd_backlight(true);
    lcd2_backlight(true);
    
    // Buffers pour les messages LCD
    char line_g[17] = "";     // Pour LCD1 - ligne 1
    char line_d[17] = "";     // Pour LCD1 - ligne 2
    char line_av[21] = "";    // Pour LCD2 - ligne 1
    char line_avg[21] = "";   // Pour LCD2 - ligne 2
    char line_avd[21] = "";   // Pour LCD2 - ligne 3
    char line_ar[21] = "";    // Pour LCD2 - ligne 4

    // Boucle principale
    while (1) {
        // Mesure des distances pour tous les capteurs avec délais
        float dist_g = measure_distance_cm(&sensor_g);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        float dist_d = measure_distance_cm(&sensor_d);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        float dist_av = measure_distance_cm(&sensor_av);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        float dist_avg = measure_distance_cm(&sensor_avg);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        float dist_avd = measure_distance_cm(&sensor_avd);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        float dist_ar = measure_distance_cm(&sensor_ar);

        // Effacement complet des écrans
        lcd_clear();
        lcd2_clear();
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Petit délai après effacement

        // LCD1 - Première ligne (Capteur G)
        memset(line_g, 0, sizeof(line_g));
        lcd_set_cursor(0, 0);
        if (is_valid_measurement(dist_g)) {
            if (dist_g > 10) {
                snprintf(line_g, sizeof(line_g), "G: %.1fcm", dist_g);
            } else {
                snprintf(line_g, sizeof(line_g), "Angle mort G !");
            }
            lcd_print(line_g);
        }
        
        // LCD1 - Deuxième ligne (Capteur D)
        memset(line_d, 0, sizeof(line_d));
        lcd_set_cursor(1, 0);
        if (is_valid_measurement(dist_d)) {
            if (dist_d > 10) {
                snprintf(line_d, sizeof(line_d), "D: %.1fcm", dist_d);
            } else {
                snprintf(line_d, sizeof(line_d), "Angle mort D !");
            }
            lcd_print(line_d);
        }

        // LCD2 - Première ligne (Capteur AV)
        memset(line_av, 0, sizeof(line_av));
        lcd2_set_cursor(0, 0);
        if (is_valid_measurement(dist_av)) {
            if (dist_av > 5) {
                snprintf(line_av, sizeof(line_av), "AV: %.1fcm", dist_av);
            } else {
                snprintf(line_av, sizeof(line_av), "Att. F. Urgence !");
            }
            lcd2_print(line_av);
        }
        
        // LCD2 - Deuxième ligne (Capteur AVG)
        memset(line_avg, 0, sizeof(line_avg));
        lcd2_set_cursor(1, 0);
        if (is_valid_measurement(dist_av) && is_valid_measurement(dist_avg) && 
            is_valid_measurement(dist_g)) {
            if (dist_av > 5 && dist_avg > 20 && dist_g > 10) {
                snprintf(line_avg, sizeof(line_avg), "AVG: %.1fcm", dist_avg);
            } else {
                snprintf(line_avg, sizeof(line_avg), "Dep. Non Aut. !");
            }
            lcd2_print(line_avg);
        }
        
        // LCD2 - Troisième ligne (Capteur AVD)
        memset(line_avd, 0, sizeof(line_avd));
        lcd2_set_cursor(2, 0);
        if (is_valid_measurement(dist_av) && is_valid_measurement(dist_avd) && 
            is_valid_measurement(dist_d)) {
            if (dist_av > 5 && dist_avd > 20 && dist_d > 10) {
                snprintf(line_avd, sizeof(line_avd), "AVD: %.1fcm", dist_avd);
            } else {
                snprintf(line_avd, sizeof(line_avd), "Dep. Non Aut. !");
            }
            lcd2_print(line_avd);
        }
        
        // LCD2 - Quatrième ligne (Capteur AR)
        memset(line_ar, 0, sizeof(line_ar));
        lcd2_set_cursor(3, 0);
        if (is_valid_measurement(dist_ar)) {
            if (dist_ar > 10) {
                snprintf(line_ar, sizeof(line_ar), "AR: %.1fcm", dist_ar);
            } else {
                snprintf(line_ar, sizeof(line_ar), "Att. AR !");
            }
            lcd2_print(line_ar);
        }

        // Délai avant la prochaine mesure
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}