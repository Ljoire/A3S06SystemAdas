/**
 * @file i2c_lcd.h
 * @brief Header file for I2C LCD module control.
 *
 * This file provides macros and function declarations for controlling an LCD
 * module connected via I2C using the PCF8574 I/O expander. It supports basic 
 * LCD operations such as clearing the display, setting the cursor position, 
 * and printing text.
 */

#ifndef I2C_LCD_H
#define I2C_LCD_H

#include <driver/i2c.h>
#include <stdbool.h>

/**
 * @brief I2C address of the LCD module with PCF8574.
 */
#define LCD_I2C_ADDR    0x27  

/**
 * @brief Dimensions of the LCD display.
 *
 * These macros define the number of columns and rows supported by the LCD.
 */
#define LCD_COLS        16    /**< Number of columns on the LCD. */
#define LCD_ROWS        2     /**< Number of rows on the LCD. */

/**
 * @brief LCD command set.
 *
 * These commands are used to control the LCD and modify its behavior.
 */
#define LCD_CLEARDISPLAY    0x01 /**< Command to clear the LCD display. */
#define LCD_RETURNHOME      0x02 /**< Command to reset the cursor to the home position. */
#define LCD_ENTRYMODESET    0x04 /**< Command to set the entry mode. */
#define LCD_DISPLAYCONTROL  0x08 /**< Command to control display settings. */
#define LCD_FUNCTIONSET     0x20 /**< Command to set the LCD function mode. */
#define LCD_SETCGRAMADDR    0x40 /**< Command to set the CGRAM address for custom characters. */
#define LCD_SETDDRAMADDR    0x80 /**< Command to set the DDRAM address for display data. */

// Définition des priorités des tâches
#define DISPLAY_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
// Taille des piles pour les tâches
#define DISPLAY_STACK_SIZE      (configMINIMAL_STACK_SIZE * 2)

/**
 * @brief Flags for display control.
 *
 * These flags are combined with `LCD_DISPLAYCONTROL` to modify the display settings.
 */
#define LCD_DISPLAYON       0x04 /**< Turn the display on. */
#define LCD_DISPLAYOFF      0x00 /**< Turn the display off. */
#define LCD_CURSORON        0x02 /**< Turn the cursor on. */
#define LCD_CURSOROFF       0x00 /**< Turn the cursor off. */
#define LCD_BLINKON         0x01 /**< Enable blinking of the cursor. */
#define LCD_BLINKOFF        0x00 /**< Disable blinking of the cursor. */

/**
 * @brief Flags for entry mode control.
 *
 * These flags are combined with `LCD_ENTRYMODESET` to set the entry mode of the LCD.
 */
#define LCD_ENTRYRIGHT          0x00 /**< Text flows from left to right. */
#define LCD_ENTRYLEFT           0x02 /**< Text flows from right to left. */
#define LCD_ENTRYSHIFTINCREMENT 0x01 /**< Automatically shift the display when a character is written. */
#define LCD_ENTRYSHIFTDECREMENT 0x00 /**< Do not shift the display when a character is written. */

/**
 * @brief Initializes the LCD module.
 *
 * Configures the LCD and I2C interface. This function must be called
 * before any other LCD operation.
 */
void lcd_init(void);

/**
 * @brief Clears the LCD display.
 *
 * Resets the display and moves the cursor to the top-left corner.
 */
void lcd_clear(void);

/**
 * @brief Resets the cursor to the home position.
 *
 * Moves the cursor to the top-left corner without clearing the display.
 */
void lcd_home(void);

/**
 * @brief Sets the cursor to a specific position on the LCD.
 *
 * @param row Row number (0-indexed). Must be less than `LCD_ROWS`.
 * @param col Column number (0-indexed). Must be less than `LCD_COLS`.
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Prints a string to the LCD at the current cursor position.
 *
 * @param str Pointer to the null-terminated string to display.
 */
void lcd_print(const char* str);

/**
 * @brief Writes a byte to the LCD over I2C.
 *
 * Sends a byte to the LCD in 4-bit mode by splitting it into a high and low nibble.
 * The data is sent along with control signals such as RS, RW, EN, and BL.
 *
 * @param cmd The byte to write (command or data).
 * @param is_data Set to `true` for data, or `false` for command.
 * @return esp_err_t Returns `ESP_OK` on success, or an error code on failure.
 */
static esp_err_t lcd_write_byte(uint8_t cmd, bool is_data);

/**
 * @brief Sends a command to the LCD.
 *
 * Sends a command byte to the LCD using the `lcd_write_byte` function.
 * Adds appropriate delays for specific commands like `LCD_CLEARDISPLAY` and `LCD_RETURNHOME`.
 *
 * @param cmd The command byte to send.
 */
void lcd_send_cmd(uint8_t cmd);

/**
 * @brief Controls the LCD backlight.
 *
 * @param on Set to `true` to enable the backlight, or `false` to disable it.
 */
void lcd_backlight(bool on);

extern void display_task(void *pvParameters);

#endif // I2C_LCD_H
