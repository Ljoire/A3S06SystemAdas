#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

static const char *TAG = "ESP_NOW_RECEIVER";
static QueueHandle_t msg_queue;
static SemaphoreHandle_t i2c_mutex;

// Structure pour les messages reçus
typedef struct {
    char message[32];
} espnow_message_t;

// Callback de réception ESP-NOW
static void esp_now_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len) {
    if (len >= sizeof(espnow_message_t)) {
        return; // Message trop long
    }
    
    espnow_message_t msg;
    memset(&msg, 0, sizeof(msg));  // Effacer la structure
    memcpy(msg.message, data, len);
    msg.message[len] = '\0';  // Assurer la terminaison de la chaîne
    
    if (xQueueSend(msg_queue, &msg, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Erreur d'envoi dans la queue");
    }
}

// Fonction d'écriture sur le LCD
static void lcd_write_byte(uint8_t data, bool rs) {
    uint8_t high = (data & 0xF0) | 0x08 | (rs ? 1 : 0);
    uint8_t low = ((data << 4) & 0xF0) | 0x08 | (rs ? 1 : 0);
    
    uint8_t buf[4] = {
        high | 0x04, high,
        low | 0x04, low
    };
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x27 << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 4, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(1);
}

static void lcd_cmd(uint8_t cmd) {
    lcd_write_byte(cmd, false);
}

static void lcd_data(uint8_t data) {
    lcd_write_byte(data, true);
}

static void lcd_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    lcd_cmd(0x33);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_cmd(0x32);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_cmd(0x28);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_cmd(0x0C);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_cmd(0x01);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_cmd(0x06);
    vTaskDelay(5 / portTICK_PERIOD_MS);
}

static void lcd_clear(void) {
    lcd_cmd(0x01);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_cmd(0x80 | (col + row_offsets[row]));
}

static void lcd_print(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

// Tâche d'affichage des messages modifiée
static void display_task(void *pvParameter) {
    espnow_message_t msg;
    while (1) {
        if (xQueueReceive(msg_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                // N'afficher que le message "Att. F. Urg !"
                if (strcmp(msg.message, "Att. F. Urg !") == 0) {
                    lcd_clear();
                    lcd_set_cursor(0, 0);
                    lcd_print(msg.message);
                } else {
                    // Pour tout autre message, effacer l'écran
                    lcd_clear();
                }
                xSemaphoreGive(i2c_mutex);
            }
        }
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    msg_queue = xQueueCreate(10, sizeof(espnow_message_t));
    i2c_mutex = xSemaphoreCreateMutex();

    wifi_init();
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    lcd_init();
    lcd_clear();

    xTaskCreate(display_task, "display_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Récepteur ESP-NOW initialisé et en attente de messages");
}