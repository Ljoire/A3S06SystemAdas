#ifndef I2C_LCD2_H
#define I2C_LCD2_H

#include <driver/i2c.h>
#include <stdbool.h>
const char *TAG = "LCD";





//################################# PARAMETRES UTILE
// affichage des distanee 
#define DISTANCE_RANGE_PRINT 5
//################################# PARAMETRES DE CONFIGURATION DU LCD 20X04   #################################

#ifndef I2C_LCD_H
#define I2C_LCD_H

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

// Bits de contrôle PCF8574
#define LCD_RS_BIT      0x01
#define LCD_RW_BIT      0x02
#define LCD_EN_BIT      0x04
#define LCD_BL_BIT      0x08
#define LCD_DATA_BITS   0xF0

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

//################################# PARAMETRES DE CONFIIGURATION DU LCD 16X02   #################################

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

// Fonctions pour le second LCD
void lcd2_init(void);
void lcd2_clear(void);
void lcd2_home(void);
void lcd2_set_cursor(uint8_t row, uint8_t col);
void lcd2_print(const char* str);
void lcd2_backlight(bool on);

#endif // I2C_LCD2_H

/**
 * @brief Reprend l'affichage standard du LCD quand il n'y a pas d'alerte.
 * Cette fonction  est appelé lors d'une demande de RAZ fourni par le calculateur
 * 
 * @return esp_err_t ESP_OK l'affichage c'est bien réalisé ESP_NOK Erreur dans le réaffichage
 */
esp_err_t lcd16x2StdPrint(void);

/**
 * @brief Fonction pour l'initalisation des 2 LCD. 
 * 
 * @param i2c_port port utilisé par l'écran
 * @param i2caddr Adresse utilisé par l'écran
 * @param FourOrTwoLine Si true l'écran initialisé est un 4 ligne si false un deux ligne
 */
void lcd_init(i2c_port_t i2c_port,uint8_t i2caddr,bool FourOrTwoLine);


