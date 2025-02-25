#include "BH1750.h"
#include "esp_log.h"
#include "calculateur.h"


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

static const char *TAG = "BH1750";

extern QueueHandle_t queueluminosité_rx; 

void bh1750_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "I2C initialisé");
}

esp_err_t bh1750_write(uint8_t cmd) {
    uint8_t data = cmd;
    return i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}

esp_err_t bh1750_read(uint16_t *lux) {
    uint8_t data[2];
    esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_NUM, BH1750_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        *lux = ((data[0] << 8) | data[1]);
    }
    return ret;
}

uint16_t get_luminosity() {
    uint16_t lux = 0;
    bh1750_write(BH1750_CMD);
    vTaskDelay(pdMS_TO_TICKS(200));  
    bh1750_read(&lux);
    return lux;
}

void luminosity_task(void *pvParameters) {
    //bh1750_init();
    while (1) {
        uint16_t lux = get_luminosity();
        ESP_LOGI(TAG, "Luminosité mesurée: %u lux", lux);
        
        uint8_t alert = 0;
        if (lux >= HIGH_BEAM_THRESHOLD) {
            alert = LUMINOSITE_PLEIN_PHARE;
            if (queueLuminosite_rx != NULL) {
                if (xQueueSend(queueLuminosite_rx, &alert, pdMS_TO_TICKS(200)) != pdPASS) {
                    ESP_LOGE("Queue", "Failed to send alert_code to queueCapteur_rx");
                }
                //ESP_LOGI(TAG,"envoie d'une info");
            } else {
                ESP_LOGE("Queue", "queueCapteur_rx is NULL");
            }   
            ESP_LOGW(TAG, "Alerte! Pleins phares détectés, code: %u", alert);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}
