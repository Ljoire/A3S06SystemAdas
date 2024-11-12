#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <driver/i2c.h>

#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

void lcd_init(void);
void lcd_clear(void);
void lcd_write_string(uint8_t row, uint8_t col, const char *str);

#endif