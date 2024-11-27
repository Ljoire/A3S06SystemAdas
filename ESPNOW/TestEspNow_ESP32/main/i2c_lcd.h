#ifndef I2C_LCD_H
#define I2C_LCD_H

#include <driver/i2c.h>
#include <stdbool.h>

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

void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char* str);
void lcd_backlight(bool on);

#endif // I2C_LCD_H