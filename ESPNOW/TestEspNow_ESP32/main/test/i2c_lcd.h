#include <driver/i2c.h>
#include <stdbool.h>






//################################# PARAMETRES UTILE
// affichage des distane
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


//#endif // I2C_LCD_H

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

#endif // I2C_LCD2_H



/**
 * @brief Fonction pour l'initalisation des 2 LCD. 
 * 
 * @param i2c_port port utilisé par l'écran
 * @param i2caddr Adresse utilisé par l'écran
 * @param FourOrTwoLine Si true l'écran initialisé est un 4 ligne si false un deux ligne
 */
void lcd_init(i2c_port_t i2c_port,uint8_t i2caddr,bool FourOrTwoLine);

/**
 * @brief Configuration du rétroéclairage du LCD
 * 
 * @param i2c_port Port i2c utilisé par l'ESP32 (I2C_NUM_0 ou I2C_NUM_1)
 * @param i2caddr Adresse de l'appareil LCD_I2C_ADDR ou LCD2_I2C_ADDR
 * @param on True : utilisation de l'éclairage Fales : Extinction
 */
void lcd_backlight(i2c_port_t i2c_port,uint8_t i2caddr,bool on);

/**
 * @brief Mise du curseur à un endroit spécifique si on utilise le LDD2(20x4) alors un offset est appliqué
 * 
 * @param i2c_port Port i2c utilisé par l'ESP32 (I2C_NUM_0 ou I2C_NUM_1)
 * @param i2caddr Adresse de l'appareil LCD_I2C_ADDR ou LCD2_I2C_ADDR
 * @param row Ligne désiré
 * @param col Collonne désiré 
 */
void lcd_set_cursor(i2c_port_t i2c_port,uint8_t i2caddr,uint8_t row, uint8_t col);

/**
 * @brief écriture d'un octet dans le LCD. Cette fonction est réutiliser pour lcd_print et send_cmd
 * 
 * @param i2c_port Port i2c utilisé par l'ESP32 (I2C_NUM_0 ou I2C_NUM_1)
 * @param i2caddr Adresse de l'appareil LCD_I2C_ADDR ou LCD2_I2C_ADDR
 * @param cmd données désirée, il faut envoyer la valeur en ASCII
 * @param is_data présence de données 
 * @return esp_err_t ESP_OK ou ESP_NOK selon l'éxécution de la fonction
 */
static esp_err_t lcd_write_byte(i2c_port_t i2c_port,uint8_t i2caddr,uint8_t cmd, bool is_data);

/**
 * @brief Enovie d'une commande spécifique au LCD
 * 
 * @param i2c_port Port i2c utilisé par l'ESP32 (I2C_NUM_0 ou I2C_NUM_1)
 * @param i2caddr Adresse de l'appareil LCD_I2C_ADDR ou LCD2_I2C_ADDR
 * @param cmd Commande désirée
 */
static void lcd_send_cmd(i2c_port_t i2c_port,uint8_t i2caddr,uint8_t cmd);

/**
 * @brief 
 * 
 * @param i2c_port Port i2c utilisé par l'ESP32 (I2C_NUM_0 ou I2C_NUM_1)
 * @param i2caddr Adresse de l'appareil LCD_I2C_ADDR ou LCD2_I2C_ADDR
 * @param str chaine de caractère à afficher
 */
void lcd_print(i2c_port_t i2c_port,uint8_t i2caddr,const char* str);

/**
 * @brief RAZ de l'affichage
 * 
 * @param i2c_port Port i2c utilisé par l'ESP32 (I2C_NUM_0 ou I2C_NUM_1)
 * @param i2caddr Adresse de l'appareil LCD_I2C_ADDR ou LCD2_I2C_ADDR
 * @return esp_err_t ESP_OK ou ESP_NOK selon l'éxécution de la fonction
 */
esp_err_t lcdStdPrint(i2c_port_t i2c_port,uint8_t i2caddr);

/**
 * @brief Affichage des distances sur l'ensemble des lignes à l'exception de celles avec une erreur affiché
 * 
 * @param distance Les distances à afficher sur les lignes 
 * @param MaskLine Masque binaire pour ne pas afficher de distance sur les lignes qui possède un erreur
 * @return esp_err_t 
 */
esp_err_t lcdDistancePrint(uint16_t *distance,uint8_t MaskLine);