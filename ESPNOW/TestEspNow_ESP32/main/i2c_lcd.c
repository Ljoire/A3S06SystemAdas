/**
 * @file lcd_i2c.c
 * @brief Implementation of I2C-based LCD control functions.
 *
 * This file contains the implementation of functions to initialize and control
 * an LCD module connected via I2C using the PCF8574 I/O expander. It supports
 * operations such as clearing the display, printing text, setting the cursor position,
 * and toggling the backlight.
 */

#include "i2c_lcd.h"
#include <esp_log.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LCD"; /**< Tag used for ESP log messages. */
static i2c_port_t i2c_port = I2C_NUM_0; /**< I2C port used for communication with the LCD. */

/**
 * @brief Control bits for the PCF8574.
 */
#define LCD_RS_BIT      0x01 /**< Register Select bit: Command(0)/Data(1). */
#define LCD_RW_BIT      0x02 /**< Read/Write bit: Write(0)/Read(1). */
#define LCD_EN_BIT      0x04 /**< Enable bit: Activates data read/write. */
#define LCD_BL_BIT      0x08 /**< Backlight control bit. */
#define LCD_DATA_BITS   0xF0 /**< Data bits mask for the high nibble. */

static uint8_t backlight_state = LCD_BL_BIT; /**< Stores the current state of the backlight. */



// Fonction pour envoyer une commande au LCD2
static void lcd_send_cmd(uint8_t cmd) {
    lcd_write_byte(cmd, false);
    if (cmd == LCD_CLEARDISPLAY || cmd == LCD_RETURNHOME) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
    } else {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


void lcd_init(void) {
    /**
     * @brief Initializes the LCD and the I2C interface.
     *
     * Configures the I2C master interface and initializes the LCD in 4-bit mode.
     * Sends a series of initialization commands to configure the LCD.
     */
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

    vTaskDelay(100 / portTICK_PERIOD_MS);

    uint8_t init_seq[] = {0x03, 0x03, 0x03, 0x02};
    for (int i = 0; i < 4; i++) {
        uint8_t data = (init_seq[i] << 4) | backlight_state;
        uint8_t buf[2] = {data | LCD_EN_BIT, data};
        i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    lcd_send_cmd(LCD_FUNCTIONSET | 0x08);
    lcd_send_cmd(LCD_DISPLAYCONTROL | 0x04);
    lcd_send_cmd(LCD_CLEARDISPLAY);
    lcd_send_cmd(LCD_ENTRYMODESET | 0x02);

    ESP_LOGI(TAG, "LCD initialized successfully");
}

void lcd_clear(void) {
    /**
     * @brief Clears the LCD display.
     *
     * Sends the `LCD_CLEARDISPLAY` command to clear the LCD and reset the cursor position.
     */
    lcd_send_cmd(LCD_CLEARDISPLAY);
}

void lcd_home(void) {
    /**
     * @brief Resets the cursor to the home position.
     *
     * Sends the `LCD_RETURNHOME` command to move the cursor to the top-left corner.
     */
    lcd_send_cmd(LCD_RETURNHOME);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    /**
     * @brief Sets the cursor to a specific position on the LCD.
     *
     * Calculates the DDRAM address based on the row and column and sends the corresponding command.
     *
     * @param row Row number (0-indexed). Must be less than `LCD_ROWS`.
     * @param col Column number (0-indexed). Must be less than `LCD_COLS`.
     */
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_print(const char* str) {
    /**
     * @brief Prints a string to the LCD at the current cursor position.
     *
     * Iterates through the string and sends each character as data to the LCD.
     *
     * @param str Pointer to the null-terminated string to display.
     */
    while (*str) {
        lcd_write_byte(*str++, true);
    }
}

void lcd_backlight(bool on) {
    /**
     * @brief Controls the LCD backlight.
     *
     * Enables or disables the backlight by updating the `backlight_state`.
     *
     * @param on Set to `true` to enable the backlight, or `false` to disable it.
     */
    backlight_state = on ? LCD_BL_BIT : 0x00;
    uint8_t data = backlight_state;
    i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}
