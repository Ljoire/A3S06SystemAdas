#include "i2c_lcd.h"
#include "calculateur.h"
#include <esp_err.h>
static const char *TAG = "LCD";
// ################### PORT I2C LCD1 ET LCD2 ###################
//LCD2
static i2c_port_t PortI2c_20x4 = I2C_NUM_0; // Utilisation du second port I2C
static uint8_t backlight_state = 0x08;   // État initial du rétroéclairage
//LCD1 
static i2c_port_t PortI2c_16x2 = I2C_NUM_0;

// ################### CONFIGURATION LCD  ###################

char *tableLCD1[] = {"G  : xxx cm","D  : xxx cm"};
char *tableLCD2[] = {"AV : xxx cm","AVG: xxx cm","AVD: xxx cm","AR : xxx cm"};

//attention !  n'est actuellement pas utilisé

char *tableAlerte[] = {
    "Att. Ralentir !",  // ALERTE_ATT_RALENTIR
    "Ralentir",         // ALERTE_RALENTIR
    "Frein urgence !",  // ALERTE_FREIN_URGENCE
    "FREINAGE!",        // ALERTE_FREINAGE
    "STOP",             // ALERTE_STOP
    "Danger Dtct",      // ALERTE_DANGER
    "Dep. D No",        // ALERTE_DEP_D_NO
    "Trajec. OK",       // ALERTE_TRAJEC_OK
    "Attention!",       // ALERTE_ATTENTION
    "Angle mort G",     // ALERTE_ANGLE_MORT_G
    "Angle mort D",      // ALERTE_ANGLE_MORT_D
    "Présence AG",      // ALERTE_PRESENCE_AG
    "Présence AD"       // ALERTE_PRESENCE_AD
    "Depassement"       // ALERTE_DEPASSEMENT
    "Impossible"        // ALERTE_IMPOSSIBLE
    "DNPW"              // ALERTE_DNPW
    "Gauche bloque"     // ALERTE_GAUCHE_BLOQUE
    "Droite bloque"     // ALERTE_DROITE_BLOQUE
};

void lcd_init(i2c_port_t i2c_port,uint8_t i2caddr,bool is_configured) {
    
    i2c_config_t conf;  // Déclaration avant le if

    // Configuration I2C pour LCD 12x2
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.scl_io_num = GPIO_NUM_22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    if(!is_configured){        
        ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
    }

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
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_LOGE(TAG,"Allumage du LCD");

    // Configuration du LCD
    lcd_send_cmd(i2c_port,i2caddr,LCD_FUNCTIONSET | 0x08);        // 4-bit, 2 lignes, 5x8 pixels
    vTaskDelay(pdMS_TO_TICKS(20));
    lcd_send_cmd(i2c_port, i2caddr, 0x0F);  // Affichage ON, curseur ON, clignotement ON
    //lcd_send_cmd(i2c_port,i2caddr,LCD_DISPLAYCONTROL | 0x04);     // Display ON, pas de curseur
    vTaskDelay(pdMS_TO_TICKS(20));
    lcd_send_cmd(i2c_port,i2caddr,LCD_CLEARDISPLAY);              // Effacer l'écran
    vTaskDelay(pdMS_TO_TICKS(20));
    lcd_send_cmd(i2c_port,i2caddr,LCD_ENTRYMODESET | 0x02);       // Entrée de gauche à droite

    vTaskDelay(pdMS_TO_TICKS(20));

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
        lcd_send_cmd(i2c_port, i2caddr, LCD2_SETDDRAMADDR | (col + row_offsets[row]));
    }
    //Sur le LCD 16x2
    else{
        ESP_LOGE(TAG,"mise du curseur sur le LCD 16x2");
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
    ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_write_byte(i2c_port,i2caddr,cmd, false));
    if (cmd == LCD_CLEARDISPLAY || cmd == LCD_RETURNHOME) {
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lcd_print(i2c_port_t i2c_port,uint8_t i2caddr,const char* str) {
    while (*str) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_write_byte(i2c_port,i2caddr,*str++, true));
    }
}

esp_err_t lcdStdPrint(i2c_port_t i2c_port,uint8_t i2caddr,char * tableLCD[]){
    //chaine d'affichage
    
    //si on RAZ le 20x4
    if(i2caddr == LCD2_I2C_ADDR){
        ESP_LOGE(TAG,"affichage du 20x4");
        for(int i = 0; i < LCD2_ROWS;i++){
            lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,i,0);
            ESP_LOGE(TAG,"on print sur le LCD");
            lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableLCD[i]);
        }
    }
    else{
        for(int i = 0; i < LCD_ROWS;i++){
            ESP_LOGE(TAG,"affichage du 16x2 le curseur va aller sur la ligne %d",i);
            lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,i,0);
            ESP_LOGE(TAG,"on print sur le LCD");
            lcd_print(PortI2c_16x2,LCD_I2C_ADDR,tableLCD[i]);
        }
    }
    return ESP_OK;
    
}

esp_err_t lcdDistancePrint(uint16_t *distance,uint8_t MaskLine){
//affichage sur le 20x4
    for(int i = 0;i <LCD2_ROWS;i++){
        //si le bit de MaskLine est à 1 on n'éxcécute pas cette itération car une erreur est affiché
        if (MaskLine & (1 << i)) {
            distance++;
            continue;
        }
        lcd_set_cursor(PortI2c_16x2,LCD2_I2C_ADDR,i,DISTANCE_RANGE_PRINT);
        lcd_print(PortI2c_16x2,LCD2_I2C_ADDR,*distance);
        distance++;
    }
    for(int i=0;i<LCD_ROWS;i++){
        //on ajoute LCD_ROWS pour vérifier le 0+4 ème bit
        if (MaskLine & (1 << (i + LCD2_ROWS))){
            distance++;
            continue;
        }
        lcd_set_cursor(PortI2c_20x4,LCD_I2C_ADDR,i,DISTANCE_RANGE_PRINT);
        lcd_print(PortI2c_20x4,LCD_I2C_ADDR,*distance);
        distance++;
    }
    return ESP_OK;
}
// Tâche d'affichage local

esp_err_t lcd_initialization(void){
    lcd_init(PortI2c_16x2,LCD_I2C_ADDR,false);//LCD 16x2
    lcd_backlight(PortI2c_16x2,LCD_I2C_ADDR,true);
    ESP_LOGE(TAG,"Backlight allumé");
    lcdStdPrint(PortI2c_16x2,LCD_I2C_ADDR,tableLCD1);
    ESP_LOGE(TAG,"init 1 OK");

    lcd_init(PortI2c_20x4,LCD2_I2C_ADDR,true);//LCD 2Ox4
    lcd_backlight(PortI2c_20x4,LCD2_I2C_ADDR,true);
    lcdStdPrint(PortI2c_20x4,LCD2_I2C_ADDR,tableLCD2);
    ESP_LOGE(TAG,"init 2 OK");

    return ESP_OK;
}

void display_task(void *pvParameters) {
    lcd_initialization();
    uint16_t lcd_alert;
    // un bit par ligne en partant du MSB si il est mis a 1 alors il y a une alerte d'affiché
    uint8_t MaskLine = 0x00; 
    while (1) {

        if (xQueueReceive(queueLCD_tx, &lcd_alert, portMAX_DELAY) == pdTRUE) {
            ESP_LOGE(TAG,"La queue dépile le code alerte est : %d",lcd_alert);
            if(lcd_alert == DISTANCE_A_RECEVOIR){
                lcdDistancePrint(&lcd_alert,MaskLine);
                vTaskDelay(pdMS_TO_TICKS(300));
                break;
            }
            lcd_alert = (uint8_t) lcd_alert;
            switch(lcd_alert){
                case CAPTEUR_EEBL_MID:
                case CAPTEUR_EEBL_HIGH:
                case CAPTEUR_FCW_HIGH:
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,4);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_ATT_RALENTIR]);
                    MaskLine = MaskLine + LCD_20X4_L4;
                    break;
                case ESPNOW_EEBL_MID:
                case ESPNOW_EEBL_HIGH:

                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,4);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_ATT_RALENTIR]);
                    MaskLine = MaskLine + LCD_20X4_L4;

                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,1);
                    lcd_print(PortI2c_16x2,LCD_I2C_ADDR,tableAlerte[ALERTE_RALENTIR]);
                    MaskLine = MaskLine + LCD_16X2_L2;
                    break;
                case CAPTEUR_EEBL_CRIT:
                case ESPNOW_EEBL_CRIT:
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,4);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_FREIN_URGENCE]);
                    MaskLine = MaskLine + LCD_20X4_L4;
                case CAPTEUR_BSW_GAUCHE:
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,2);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_ANGLE_MORT_G]);
                    MaskLine = MaskLine + LCD_20X4_L2;
                case ESPNOW_BSW_G:
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,2);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_PRESENCE_AG]);
                    MaskLine = MaskLine + LCD_20X4_L2;          
                case CAPTEUR_BSW_DROITE:
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,2);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_ANGLE_MORT_D]);
                    MaskLine = MaskLine + LCD_20X4_L2;
                    break;
                case ESPNOW_BSW_D:
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,2);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_PRESENCE_AD]);
                    MaskLine = MaskLine + LCD_20X4_L2;
                    break;
                case CAPTEUR_DNPW_G:

                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,1);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_DEPASSEMENT]);
                    MaskLine = MaskLine + LCD_20X4_L1;
                    
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,2);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_IMPOSSIBLE]);
                    MaskLine = MaskLine + LCD_20X4_L2;

                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,0);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_DNPW]);
                    MaskLine = MaskLine + LCD_16X2_L1;

                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,1);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_GAUCHE_BLOQUE]);
                    MaskLine = MaskLine + LCD_16X2_L2;
                    break;
                case ESPNOW_DNPW_G:
                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,0);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_DNPW]);
                    MaskLine = MaskLine + LCD_16X2_L1;

                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,1);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_GAUCHE_BLOQUE]);
                    MaskLine = MaskLine + LCD_16X2_L2;
                    break;
                case CAPTEUR_DNPW_D:                                          
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,1);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_DEPASSEMENT]);
                    MaskLine = MaskLine + LCD_20X4_L1;
                    
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,2);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_IMPOSSIBLE]);
                    MaskLine = MaskLine + LCD_20X4_L2;

                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,0);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_DNPW]);
                    MaskLine = MaskLine + LCD_16X2_L1;

                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,1);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_DROITE_BLOQUE]);
                    MaskLine = MaskLine + LCD_16X2_L2;
                    break;
                case ESPNOW_DNPW_D:
                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,0);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_DNPW]);
                    MaskLine = MaskLine + LCD_16X2_L1;

                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,1);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_DROITE_BLOQUE]);
                    MaskLine = MaskLine + LCD_16X2_L2;
                    break;
                //Le code 15 (FCW mid) n'a pas été implémenté car il faut le clarifier
                case ESPNOW_FCW_HIGH:
                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,1);
                    lcd_print(PortI2c_16x2,LCD_I2C_ADDR,tableAlerte[ALERTE_RALENTIR]);
                    MaskLine = MaskLine + LCD_16X2_L2;
                    break;
                case CAPTEUR_FCW_CRIT:
                    lcd_set_cursor(PortI2c_20x4,LCD2_I2C_ADDR,0,4);
                    lcd_print(PortI2c_20x4,LCD2_I2C_ADDR,tableAlerte[ALERTE_FREINAGE]);
                    MaskLine = MaskLine + LCD_20X4_L4;
                    
                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,1);
                    lcd_print(PortI2c_16x2,LCD_I2C_ADDR,tableAlerte[ALERTE_STOP]);
                    MaskLine = MaskLine + LCD_16X2_L2;
                    break;
                case ESPNOW_FCW_CRIT:
                    lcd_set_cursor(PortI2c_16x2,LCD_I2C_ADDR,0,0);
                    lcd_print(PortI2c_20x4,LCD_I2C_ADDR,tableAlerte[ALERTE_DANGER]);
                    MaskLine = MaskLine + LCD_16X2_L1; 
                    break;                   
                default:
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}