#ifndef I2C_LCD2_H
#define I2C_LCD2_H

#include <driver/i2c.h>
#include <stdbool.h>

// Configuration du deuxième LCD (4x20)
#define LCD2_I2C_ADDR    0x27  // Adresse I2C du deuxième module LCD
#define LCD2_COLS        20    // 20 colonnes
#define LCD2_ROWS        4     // 4 lignes

// Commandes LCD (identiques au premier LCD)
#define LCD2_CLEARDISPLAY    0x01
#define LCD2_RETURNHOME      0x02
#define LCD2_ENTRYMODESET    0x04
#define LCD2_DISPLAYCONTROL  0x08
#define LCD2_FUNCTIONSET     0x20
#define LCD2_SETCGRAMADDR    0x40
#define LCD2_SETDDRAMADDR    0x80

// Drapeaux pour l'affichage
#define LCD2_DISPLAYON       0x04
#define LCD2_DISPLAYOFF      0x00
#define LCD2_CURSORON       0x02
#define LCD2_CURSOROFF      0x00
#define LCD2_BLINKON        0x01
#define LCD2_BLINKOFF       0x00

// Drapeaux pour le mode d'entrée
#define LCD2_ENTRYRIGHT          0x00
#define LCD2_ENTRYLEFT          0x02
#define LCD2_ENTRYSHIFTINCREMENT 0x01
#define LCD2_ENTRYSHIFTDECREMENT 0x00

static SemaphoreHandle_t i2c_mutex;
extern QueueHandle_t sensor_queue;

// Fonctions pour le second LCD
void lcd2_init(void);
void lcd2_clear(void);
void lcd2_home(void);
void lcd2_set_cursor(uint8_t row, uint8_t col);
void lcd2_print(const char* str);
void lcd2_backlight(bool on);
void init_i2c_mutex(void);
void display_task(void *pvParameters);

#endif // I2C_LCD2_H
