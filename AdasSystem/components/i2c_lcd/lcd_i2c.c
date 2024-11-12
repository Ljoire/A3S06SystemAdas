#include "lcd_i2c.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0

#define LCD_CMD 0x00
#define LCD_DATA 0x01
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_DDRAM 0x80

static void lcd_send(uint8_t data, uint8_t mode) {
    uint8_t high = mode | (data & 0xF0) | 0x08;
    uint8_t low = mode | ((data << 4) & 0xF0) | 0x08;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, high, true);
    i2c_master_write_byte(cmd, (high | 0x04), true);
    i2c_master_write_byte(cmd, (high & ~0x04), true);
    i2c_master_write_byte(cmd, low, true);
    i2c_master_write_byte(cmd, (low | 0x04), true);
    i2c_master_write_byte(cmd, (low & ~0x04), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void lcd_init(void) {
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

    vTaskDelay(100 / portTICK_PERIOD_MS);
    lcd_send(0x03, LCD_CMD);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send(0x03, LCD_CMD);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send(0x03, LCD_CMD);
    lcd_send(0x02, LCD_CMD);
    lcd_send(LCD_FUNCTION_SET | 0x08, LCD_CMD);
    lcd_send(LCD_DISPLAY_CONTROL | 0x04, LCD_CMD);
    lcd_send(LCD_CLEAR, LCD_CMD);
    lcd_send(LCD_ENTRY_MODE | 0x02, LCD_CMD);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

void lcd_clear(void) {
    lcd_send(LCD_CLEAR, LCD_CMD);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

void lcd_write_string(uint8_t row, uint8_t col, const char *str) {
    uint8_t offset = row == 0 ? 0x00 : 0x40;
    lcd_send(LCD_SET_DDRAM | (offset + col), LCD_CMD);
    while (*str) {
        lcd_send(*str++, LCD_DATA);
    }
}