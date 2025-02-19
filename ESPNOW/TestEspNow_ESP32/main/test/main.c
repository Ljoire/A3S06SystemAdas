#include "anglemort.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"



#define ALERT_DISTANCE 20  // Distance seuil pour déclencher une alerte

static const char *TAG = "SENSOR";

void sensor_task(void *pvParameters) {
    while (1) {
        uint8_t alerts = detect_alert();

        if (alerts & (1 << 0)) ESP_LOGI(TAG, "⚠️ Angle mort gauche !");
        if (alerts & (1 << 1)) ESP_LOGI(TAG, "⚠️ Angle mort droit !");
        if (alerts & (1 << 2)) ESP_LOGI(TAG, "⚠️ Obstacle arrière gauche ou avant gauche !");
        if (alerts & (1 << 3)) ESP_LOGI(TAG, "⚠️ Obstacle arrière droit ou avant droit !");
        if (alerts & (1 << 4)) ESP_LOGI(TAG, "⚠️ Distance < 60 cm !");
        if (alerts & (1 << 5)) ESP_LOGI(TAG, "⚠️ Distance < 40 cm !");
        if (alerts & (1 << 6)) ESP_LOGI(TAG, "⚠️ Distance < 20 cm !");
        if (alerts & (1 << 7)) ESP_LOGI(TAG, "🚗 Dépassement à droite !");
        if (alerts & (1 << 8)) ESP_LOGI(TAG, "🚗 Dépassement à gauche !");

        if (alerts == 0) ESP_LOGI(TAG, "Aucun danger détecté.");
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main() {
    ESP_LOGI(TAG, "Système de détection d'angle mort initialisé.");
    xTaskCreate(sensor_task, "Sensor Task", 4096, NULL, 5, NULL);
}









