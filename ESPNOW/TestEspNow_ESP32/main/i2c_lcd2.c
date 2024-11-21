#include "i2c_lcd2.h"
#include <esp_log.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LCD2";
static i2c_port_t i2c_port2 = I2C_NUM_1; // Utilisation du second port I2C
static uint8_t backlight_state2 = 0x08;   // État initial du rétroéclairage

// Bits de contrôle PCF8574
#define LCD2_RS_BIT      0x01
#define LCD2_RW_BIT      0x02
#define LCD2_EN_BIT      0x04
#define LCD2_BL_BIT      0x08
#define LCD2_DATA_BITS   0xF0

// Fonction pour écrire un octet sur le LCD2
static esp_err_t lcd2_write_byte(uint8_t cmd, bool is_data) {
    uint8_t high_nibble = (cmd & 0xF0) | backlight_state2;
    uint8_t low_nibble = ((cmd << 4) & 0xF0) | backlight_state2;
    
    if (is_data) {
        high_nibble |= LCD2_RS_BIT;
        low_nibble |= LCD2_RS_BIT;
    }

    uint8_t data[4];
    data[0] = high_nibble | LCD2_EN_BIT;
    data[1] = high_nibble;
    data[2] = low_nibble | LCD2_EN_BIT;
    data[3] = low_nibble;

    return i2c_master_write_to_device(i2c_port2, LCD2_I2C_ADDR, data, 4, 1000 / portTICK_PERIOD_MS);
}

// Fonction pour envoyer une commande au LCD2
static void lcd2_send_cmd(uint8_t cmd) {
    lcd2_write_byte(cmd, false);
    if (cmd == LCD2_CLEARDISPLAY || cmd == LCD2_RETURNHOME) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
    } else {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Initialisation du LCD2
void lcd2_init(void) {
    // Configuration I2C pour le second LCD
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_32,        // Nouvelle broche SDA
        .scl_io_num = GPIO_NUM_33,        // Nouvelle broche SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    
    ESP_ERROR_CHECK(i2c_param_config(i2c_port2, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port2, conf.mode, 0, 0, 0));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Séquence d'initialisation 4 bits
    uint8_t init_seq[] = {0x03, 0x03, 0x03, 0x02};
    for(int i = 0; i < 4; i++) {
        uint8_t data = (init_seq[i] << 4) | backlight_state2;
        uint8_t with_en = data | LCD2_EN_BIT;
        uint8_t without_en = data;
        
        uint8_t buf[2] = {with_en, without_en};
        i2c_master_write_to_device(i2c_port2, LCD2_I2C_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // Configuration du LCD
    lcd2_send_cmd(LCD2_FUNCTIONSET | 0x08);    // 4-bit, 2 lignes, 5x8 pixels
    lcd2_send_cmd(LCD2_DISPLAYCONTROL | 0x04); // Display ON
    lcd2_send_cmd(LCD2_CLEARDISPLAY);          // Clear display
    lcd2_send_cmd(LCD2_ENTRYMODESET | 0x02);   // Left to right

    ESP_LOGI(TAG, "LCD2 initialized successfully");
}

// Autres fonctions pour LCD2
void lcd2_clear(void) {
    lcd2_send_cmd(LCD2_CLEARDISPLAY);
}

void lcd2_home(void) {
    lcd2_send_cmd(LCD2_RETURNHOME);
}

void lcd2_set_cursor(uint8_t row, uint8_t col) {
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54}; // Offsets pour LCD 4x20
    if (row >= LCD2_ROWS) row = LCD2_ROWS - 1;
    if (col >= LCD2_COLS) col = LCD2_COLS - 1;
    lcd2_send_cmd(LCD2_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd2_print(const char* str) {
    while (*str) {
        lcd2_write_byte(*str++, true);
    }
}

void lcd2_backlight(bool on) {
    backlight_state2 = on ? LCD2_BL_BIT : 0x00;
    uint8_t data = backlight_state2;
    i2c_master_write_to_device(i2c_port2, LCD2_I2C_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}
