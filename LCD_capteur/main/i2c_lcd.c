#include "i2c_lcd.h"
#include <esp_log.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LCD";
static i2c_port_t i2c_port = I2C_NUM_0;

// Bits de contrôle PCF8574
#define LCD_RS_BIT      0x01
#define LCD_RW_BIT      0x02
#define LCD_EN_BIT      0x04
#define LCD_BL_BIT      0x08
#define LCD_DATA_BITS   0xF0

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
    lcd_write_byte(cmd, false);
    if (cmd == LCD_CLEARDISPLAY || cmd == LCD_RETURNHOME) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
    } else {
        vTaskDelay(1 / portTICK_PERIOD_MS);
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
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Séquence d'initialisation 4 bits
    uint8_t init_seq[] = {0x03, 0x03, 0x03, 0x02};
    for(int i = 0; i < 4; i++) {
        uint8_t data = (init_seq[i] << 4) | backlight_state;
        uint8_t with_en = data | LCD_EN_BIT;
        uint8_t without_en = data;
        
        uint8_t buf[2] = {with_en, without_en};
        i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // Configuration du LCD
    lcd_send_cmd(LCD_FUNCTIONSET | 0x08);        // 4-bit, 2 lignes, 5x8 pixels
    lcd_send_cmd(LCD_DISPLAYCONTROL | 0x04);     // Display ON, pas de curseur
    lcd_send_cmd(LCD_CLEARDISPLAY);              // Effacer l'écran
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