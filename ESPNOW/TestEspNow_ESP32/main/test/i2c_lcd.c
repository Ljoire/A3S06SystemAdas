#include "i2c_lcd.h"
#include "calculateur.h"
#include <esp_err.h>

// ################### PORT I2C LCD1 ET LCD2 ###################
//LCD2
static i2c_port_t i2c_port2 = I2C_NUM_1; // Utilisation du second port I2C
static uint8_t backlight_state2 = 0x08;   // État initial du rétroéclairage
//LCD1 
static i2c_port_t i2c_port = I2C_NUM_0;

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


esp_err_t lcd16x2StdPrint(void){
    return ESP_OK;
}