#include "i2c_lcd.h"
#include "calculateur.h"
#include <esp_err.h>
static const char *TAG = "LCD";
// ################### PORT I2C LCD1 ET LCD2 ###################
//LCD2
static i2c_port_t i2c_port2 = I2C_NUM_1; // Utilisation du second port I2C
static uint8_t backlight_state = 0x08;   // État initial du rétroéclairage
//LCD1 
static i2c_port_t i2c_port = I2C_NUM_0;

// ################### CONFIGURATION LCD  ###################

char *tableLCD1 = {"G  : xx cm","D   : xx cm"};
char *tableLCD2 = {"AV : xx cm","AVG: xx cm","AVD: xx cm","AR : xx cm"};

void lcd_init(i2c_port_t i2c_port,uint8_t i2caddr,bool FourOrTwoLine) {
    
    i2c_config_t conf;  // Déclaration avant le if

    if (FourOrTwoLine) {
        // Configuration I2C pour LCD 20x4
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = GPIO_NUM_21;
        conf.scl_io_num = GPIO_NUM_22;
    } else {
        // Configuration I2C pour LCD 16x2
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = GPIO_NUM_32;  // Nouvelle broche SDA
        conf.scl_io_num = GPIO_NUM_33;  // Nouvelle broche SCL
    }

    // Paramètres communs aux deux cas
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));

    // Attendre que le LCD soit prêt
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Séquence d'initialisation 4 bits
    uint8_t init_seq[] = {0x03, 0x03, 0x03, 0x02};
    for(int i = 0; i < 4; i++) {
        uint8_t data = (init_seq[i] << 4) | backlight_state;
        uint8_t with_en = data | LCD_EN_BIT;
        uint8_t without_en = data;
        
        uint8_t buf[2] = {with_en, without_en};
        i2c_master_write_to_device(i2c_port, i2caddr, buf, 2, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // Configuration du LCD
    lcd_send_cmd(i2c_port,i2caddr,LCD_FUNCTIONSET | 0x08);        // 4-bit, 2 lignes, 5x8 pixels
    lcd_send_cmd(i2c_port,i2caddr,LCD_DISPLAYCONTROL | 0x04);     // Display ON, pas de curseur
    lcd_send_cmd(i2c_port,i2caddr,LCD_CLEARDISPLAY);              // Effacer l'écran
    lcd_send_cmd(i2c_port,i2caddr,LCD_ENTRYMODESET | 0x02);       // Entrée de gauche à droite

    ESP_LOGI(TAG, "LCD initialized successfully");
}

void lcd_backlight(i2c_port_t i2c_port,uint8_t i2caddr,bool on) {
    backlight_state = on ? LCD_BL_BIT : 0x00;
    uint8_t data = backlight_state;
    i2c_master_write_to_device(i2c_port, i2caddr, &data, 1, 1000 / portTICK_PERIOD_MS);
}

// ################### ECRITURE DE COMMANDE ET AFFICHAGE ###################

void lcd_set_cursor(i2c_port_t i2c_port,uint8_t i2caddr,uint8_t row, uint8_t col) {
    if(i2caddr == LCD2_I2C_ADDR){
        static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54}; // Offsets pour LCD 4x20
        if (row >= LCD2_ROWS) row = LCD2_ROWS - 1;
        if (col >= LCD2_COLS) col = LCD2_COLS - 1;
        lcd2_send_cmd(i2c_port, i2caddr, LCD2_SETDDRAMADDR | (col + row_offsets[row]));
    }
    //Sur le LCD 16x2
    else{
        static const uint8_t row_offsets[] = {0x00, 0x40};
        if (row >= LCD_ROWS) row = LCD_ROWS - 1;
        if (col >= LCD_COLS) col = LCD_COLS - 1;
        lcd_send_cmd(i2c_port, i2caddr,LCD_SETDDRAMADDR | (col + row_offsets[row]));
    }
}

static esp_err_t lcd_write_byte(i2c_port_t i2c_port,uint8_t i2caddr,uint8_t cmd, bool is_data) {
    uint8_t high_nibble = (cmd & 0xF0) | backlight_state;
    uint8_t low_nibble = ((cmd << 4) & 0xF0) | backlight_state;
    
    if (is_data) {
        high_nibble |= LCD_RS_BIT;
        low_nibble |= LCD_RS_BIT;
    }

    uint8_t data[4];
    data[0] = high_nibble | LCD_EN_BIT;
    data[1] = high_nibble;
    data[2] = low_nibble | LCD_EN_BIT;
    data[3] = low_nibble;

    return i2c_master_write_to_device(i2c_port, i2caddr, data, 4, 1000 / portTICK_PERIOD_MS);
}

static void lcd_send_cmd(i2c_port_t i2c_port,uint8_t i2caddr,uint8_t cmd) {
    lcd_write_byte(i2c_port,i2caddr,cmd, false);
    if (cmd == LCD_CLEARDISPLAY || cmd == LCD_RETURNHOME) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
    } else {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void lcd_print(i2c_port_t i2c_port,uint8_t i2caddr,const char* str) {
    while (*str) {
        lcd_write_byte(i2c_port,i2caddr,*str++, true);
    }
}

esp_err_t lcdStdPrint(i2c_port_t i2c_port,uint8_t i2caddr){
    //chaine d'affichage
    return ESP_OK;

    //si on RAZ le 20x4
    if(i2caddr == LCD2_I2C_ADDR){
        for(int i = 0; i < LCD2_ROWS;i++){
            lcd_set_cursor(I2C_NUM_1,LCD2_I2C_ADDR,i,0);
            lcd_print(I2C_NUM_1,LCD2_I2C_ADDR,tableLCD2[i]);
        }
    }
    else{
        for(int i = 0; i < LCD_ROWS;i++){
            lcd_set_cursor(I2C_NUM_1,LCD_I2C_ADDR,i,0);
            lcd_print(I2C_NUM_1,LCD_I2C_ADDR,tableLCD1[i]);
        }
    }
    
}

// Tâche d'affichage local
static void display_task(void *pvParameters) {


    lcd_init(I2C_NUM_0,LCD_I2C_ADDR,false);//LCD 16x2
    lcd_backlight(I2C_NUM_0,LCD_I2C_ADDR,true);
    lcdStdPrint(I2C_NUM_0,LCD_I2C_ADDR);

    lcd_init(I2C_NUM_1,LCD2_I2C_ADDR,true);//LCD 2Ox4
    lcd_backlight(I2C_NUM_1,LCD2_I2C_ADDR,true);
    lcdStdPrint(I2C_NUM_1,LCD2_I2C_ADDR);


    while (1) {
        uint8_t lcd_alert;
        uint8_t MaskLine = 0x00; // un bit par ligne en partant du MSB si il est mis a 1 alors il y a une alerte d'affiché
        if (xQueueReceive(queueLCD_tx, &lcd_alert, portMAX_DELAY) == pdTRUE) {
            switch (lcd_alert)
            {
            case /* constant-expression */:
                /* code */
                break;
            
            default:
                break;
            }
        }
    }
}