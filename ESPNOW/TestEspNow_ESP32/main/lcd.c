/**
 * @file lcd.c
 * @author ANDRIANJAFINDRADILO Sitraka Marc (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <driver/i2c.h>
#include "esp_log.h"
include "freertos/FreeRTOS.h"
#include "lcd.h"
#include "espnow_example.h"

static const char *TAG = "LCD";
static i2c_port_t i2c_port = I2C_NUM_0;

// Bits de contrôle PCF8574
#define LCD_RS_BIT      0x01
#define LCD_RW_BIT      0x02
#define LCD_EN_BIT      0x04
#define LCD_BL_BIT      0x08
#define LCD_DATA_BITS   0xF0

#define LCD_I2C_ADDR    0x27  // Adresse I2C du module PCF8574
#define LCD_COLS        16    // Nombre de colonnes
#define LCD_ROWS        2     // Nombre de lignes

// Commandes LCD
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

// Drapeaux pour l'affichage
#define LCD_DISPLAYON       0x04
#define LCD_DISPLAYOFF      0x00
#define LCD_CURSORON       0x02
#define LCD_CURSOROFF      0x00
#define LCD_BLINKON        0x01
#define LCD_BLINKOFF       0x00

// Drapeaux pour le mode d'entrée
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT          0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

static uint8_t backlight_state = LCD_BL_BIT;

static esp_err_t lcd_write_byte(uint8_t cmd, bool is_data) {
    uint8_t high_nibble = (cmd & 0xF0) | backlight_state;
    uint8_t low_nibble = ((cmd << 4) & 0xF0) | backlight_state;
    
    if (is_data) {
        high_nibble |= LCD_RS_BIT;
        low_nibble |= LCD_RS_BIT;
    }

    uint8_t data[4];
    data[0] = high_nibble | LCD_EN_BIT;
    data[1] = high_nibble;
    data[2] = low_nibble | LCD_EN_BIT;
    data[3] = low_nibble;

    return i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, data, 4, 1000 / portTICK_PERIOD_MS);
}

static void lcd_send_cmd(uint8_t cmd) {
    switch (lcd_write_byte(cmd, false))
    {
    case ESP_OK:
        if (cmd == LCD_CLEARDISPLAY || cmd == LCD_RETURNHOME) {
            vTaskDelay(2 / portTICK_PERIOD_MS);
        } else {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        break;
    case ESP_ERR_INVALID_ARG:
        ESP_LOGW(TAG,"invalid arguement when sending command");
        break;
    case  ESP_FAIL:
        //ESP_LOGW(TAG,"Sending command error, slave hasn't ACK the transfer.");
        break;
    case  ESP_ERR_INVALID_STATE :
        ESP_LOGW(TAG,"I2C driver not installed or not in master mode ");
        break;
    case ESP_ERR_TIMEOUT:
        ESP_LOGW(TAG,"Operation timeout because the bus is busy.");
        break;        
    default:
        ESP_LOGW(TAG,"Passed in the defaultd case, weird case");
        break;
    }
}

void lcd_init(void) {
    // Configuration I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));

    // Attendre que le LCD soit prêt
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Séquence d'initialisation 4 bits
    uint8_t init_seq[] = {0x03, 0x03, 0x03, 0x02};
    for(int i = 0; i < 4; i++) {
        uint8_t data = (init_seq[i] << 4) | backlight_state;
        uint8_t with_en = data | LCD_EN_BIT;
        uint8_t without_en = data;
        
        uint8_t buf[2] = {with_en, without_en};
        i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    // Configuration du LCD
    lcd_send_cmd(LCD_FUNCTIONSET | 0x08);        // 4-bit, 2 lignes, 5x8 pixels
    vTaskDelay(50 / portTICK_PERIOD_MS);
    lcd_send_cmd(LCD_DISPLAYCONTROL | 0x04);     // Display ON, pas de curseur
    vTaskDelay(50 / portTICK_PERIOD_MS);
    lcd_send_cmd(LCD_CLEARDISPLAY);              // Effacer l'écran
    vTaskDelay(50 / portTICK_PERIOD_MS);
    lcd_send_cmd(LCD_ENTRYMODESET | 0x02);       // Entrée de gauche à droite

    ESP_LOGI(TAG, "LCD initialized successfully");
}

void lcd_clear(void) {
    lcd_send_cmd(LCD_CLEARDISPLAY);
}

void lcd_home(void) {
    lcd_send_cmd(LCD_RETURNHOME);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_print(const char* str) {
    while (*str) {
        lcd_write_byte(*str++, true);
    }
}

void lcd_backlight(bool on) {
    backlight_state = on ? LCD_BL_BIT : 0x00;
    uint8_t data = backlight_state;
    i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}

void lcd_task(void *pvParameter){
    //cast the send_param put as global variable
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    uint8_t *pReceivedMessage = NULL;
    while(xQueueReceive(receive_calback_queu,&pReceivedMessage, portMAX_DELAY) == pdTRUE){ 
        lcd_clear();
        lcd_set_cursor(0,0);
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "Capteur 1 : ");
        lcd_print(buffer);
        for(size_t i = 0; i <= MAX_PAYLOAD_SIZE;i++){
            lcd_write_byte(pReceivedMessage[i],true);
            printf("data displayed on i : %u %c",i,pReceivedMessage[i]);
        }
        ESP_LOGI(TAG,"Message displayed on the LCD");
    }
}
