#include <esp_log.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hc_sr04.h"
#include "i2c_lcd2.h"
#include "espnow_example.h"
#include "i2c_lcd.h"


static const char *TAG = "LCD2";
static i2c_port_t i2c_port2 = I2C_NUM_1; // Utilisation du second port I2C
static uint8_t backlight_state2 = 0x08;   // État initial du rétroéclairage
QueueHandle_t sensor_queue;

// Bits de contrôle PCF8574
#define LCD2_RS_BIT      0x01
#define LCD2_RW_BIT      0x02
#define LCD2_EN_BIT      0x04
#define LCD2_BL_BIT      0x08
#define LCD2_DATA_BITS   0xF0

// Fonction pour écrire un octet sur le LCD2
static esp_err_t lcd2_write_byte(uint8_t cmd, bool is_data) {
    uint8_t high_nibble = (cmd & 0xF0) | backlight_state2;
    uint8_t low_nibble = ((cmd << 4) & 0xF0) | backlight_state2;
    
    if (is_data) {
        high_nibble |= LCD2_RS_BIT;
        low_nibble |= LCD2_RS_BIT;
    }

    uint8_t data[4];
    data[0] = high_nibble | LCD2_EN_BIT;
    data[1] = high_nibble;
    data[2] = low_nibble | LCD2_EN_BIT;
    data[3] = low_nibble;

    return i2c_master_write_to_device(i2c_port2, LCD2_I2C_ADDR, data, 4, 1000 / portTICK_PERIOD_MS);
}

// Fonction pour envoyer une commande au LCD2
static void lcd2_send_cmd(uint8_t cmd) {
    lcd2_write_byte(cmd, false);
    if (cmd == LCD2_CLEARDISPLAY || cmd == LCD2_RETURNHOME) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
    } else {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


// Initialisation du LCD2
void lcd2_init(void) {
    // Configuration I2C pour le second LCD
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_32,        // Nouvelle broche SDA
        .scl_io_num = GPIO_NUM_33,        // Nouvelle broche SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    
    ESP_ERROR_CHECK(i2c_param_config(i2c_port2, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port2, conf.mode, 0, 0, 0));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Séquence d'initialisation 4 bits
    uint8_t init_seq[] = {0x03, 0x03, 0x03, 0x02};
    for(int i = 0; i < 4; i++) {
        uint8_t data = (init_seq[i] << 4) | backlight_state2;
        uint8_t with_en = data | LCD2_EN_BIT;
        uint8_t without_en = data;
        
        uint8_t buf[2] = {with_en, without_en};
        i2c_master_write_to_device(i2c_port2, LCD2_I2C_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // Configuration du LCD
    lcd2_send_cmd(LCD2_FUNCTIONSET | 0x08);    // 4-bit, 2 lignes, 5x8 pixels
    lcd2_send_cmd(LCD2_DISPLAYCONTROL | 0x04); // Display ON
    lcd2_send_cmd(LCD2_CLEARDISPLAY);          // Clear display
    lcd2_send_cmd(LCD2_ENTRYMODESET | 0x02);   // Left to right

    ESP_LOGI(TAG, "LCD2 initialized successfully");
}

// Autres fonctions pour LCD2
void lcd2_clear(void) {
    lcd2_send_cmd(LCD2_CLEARDISPLAY);
}

void lcd2_home(void) {
    lcd2_send_cmd(LCD2_RETURNHOME);
}

void lcd2_set_cursor(uint8_t row, uint8_t col) {
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54}; // Offsets pour LCD 4x20
    if (row >= LCD2_ROWS) row = LCD2_ROWS - 1;
    if (col >= LCD2_COLS) col = LCD2_COLS - 1;
    lcd2_send_cmd(LCD2_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd2_print(const char* str) {
    while (*str) {
        lcd2_write_byte(*str++, true);
    }
}

void lcd2_backlight(bool on) {
    backlight_state2 = on ? LCD2_BL_BIT : 0x00;
    uint8_t data = backlight_state2;
    i2c_master_write_to_device(i2c_port2, LCD2_I2C_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}


void display_task(void *pvParameters) {
    
    sensor_data_t sensor_data = *(sensor_data_t *)pvParameters;
    SemaphoreHandle_t i2c_mutex;
    // Création du mutex I2C
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return;
    }
    char buffer[21]; // Buffer assez grand pour LCD 4x20

    // Initialisation des LCD avec protection mutex
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        lcd_init();
        lcd2_init();
        lcd_backlight(true);
        lcd2_backlight(true);
        if(i2c_mutex == NULL){
            ESP_LOGW(TAG,"mutex null");
        }
        ESP_LOGI(TAG,"here");
        xSemaphoreGive(i2c_mutex);
        ESP_LOGI(TAG,"after the give");
    }
    ESP_LOGI(TAG,"out of the semaphore take");
    uint8_t DataToEspNow = 0x00;
    bool is_DATA2send = false;
    uint8_t DataFromEspNow = 0x00;
    bool is_dataFromESP = false;
    while (1) {
        /*si reception ESPNOW
        if(receive_calback_queue == NULL){
            ESP_LOGW(TAG,"la queue est  null");
        }
        if(uxQueueMessagesWaiting(receive_calback_queue) == 0){
            ESP_LOGI(TAG,"La queue est vide");
        }*/
        if(xQueueReceive(receive_calback_queue,&DataFromEspNow,pdMS_TO_TICKS(2)) == pdTRUE){
            ESP_LOGI(TAG,"The data received is : %u",DataFromEspNow);
            is_dataFromESP = true;
        }
        //SP_LOGI(TAG,"after receive callback queue");

        //if (xQueueReceive(sensor_queue, &sensor_data, pdMS_TO_TICKS(500)) == pdPASS) {
            // Prendre le mutex I2C
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                // Effacement des écrans
                //ESP_LOGI(TAG,"inside the treatment function");
                lcd_clear();
                lcd2_clear();
                vTaskDelay(pdMS_TO_TICKS(10));

                // LCD2 - Ligne 1 (AV)
                lcd2_set_cursor(0, 0);
                if (is_valid_measurement(sensor_data.dist_av) || is_dataFromESP) {// && compteur = 0 || reception ESPNOW
                    if (is_dataFromESP && (DataFromEspNow & 0x02)){ //si le bit n°1 est a 1
                        snprintf(buffer, sizeof(buffer), "Att. Freinez !");
                    }
                    else if (sensor_data.dist_av > 5) {
                        snprintf(buffer, sizeof(buffer), "AV: %.1fcm", sensor_data.dist_av);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Att. F. Urgence !");
                        is_DATA2send = true;
                        DataToEspNow = DataToEspNow | 0x02;
                    }
                    lcd2_print(buffer);
                }
                // compteur --;
                // LCD1 - Ligne 1 (G)
                lcd_set_cursor(0, 0);
                if (is_valid_measurement(sensor_data.dist_g)) {
                    if (sensor_data.dist_g > 10) {
                        snprintf(buffer, sizeof(buffer), "G: %.1fcm", sensor_data.dist_g);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Angle mort G !");
                    }
                    lcd_print(buffer);
                }

                // LCD1 - Ligne 2 (D)
                lcd_set_cursor(1, 0);
                if (is_valid_measurement(sensor_data.dist_d)) {
                    if (sensor_data.dist_d > 10) {
                        snprintf(buffer, sizeof(buffer), "D: %.1fcm", sensor_data.dist_d);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Angle mort D !");
                    }
                    lcd_print(buffer);
                }

                // LCD2 - Ligne 2 (AVG)
                lcd2_set_cursor(1, 0);
                if (is_valid_measurement(sensor_data.dist_av) && 
                    is_valid_measurement(sensor_data.dist_avg) && 
                    is_valid_measurement(sensor_data.dist_g)) {
                    if (sensor_data.dist_av > 5 && 
                        sensor_data.dist_avg > 20 && 
                        sensor_data.dist_g > 10) {
                        snprintf(buffer, sizeof(buffer), "AVG: %.1fcm", sensor_data.dist_avg);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Dep. Non Aut. !");
                        //mise du flag d'envoie a 1. Est nettoyé lorsque la donnée est envoyé
                        //is_DATA2send = true;
                    }
                    lcd2_print(buffer);
                }

                // LCD2 - Ligne 3 (AVD)
                lcd2_set_cursor(2, 0);
                if (is_valid_measurement(sensor_data.dist_av) && 
                    is_valid_measurement(sensor_data.dist_avd) && 
                    is_valid_measurement(sensor_data.dist_d)) {
                    if (sensor_data.dist_av > 5 && 
                        sensor_data.dist_avd > 20 && 
                        sensor_data.dist_d > 10) {
                        snprintf(buffer, sizeof(buffer), "AVD: %.1fcm", sensor_data.dist_avd);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Dep. Non Aut. !");
                    }
                    lcd2_print(buffer);
                }

                // LCD2 - Ligne 4 (AR)
                lcd2_set_cursor(3, 0);
                if (is_valid_measurement(sensor_data.dist_ar)) {
                    if (sensor_data.dist_ar > 10) {
                        snprintf(buffer, sizeof(buffer), "AR: %.1fcm", sensor_data.dist_ar);
                    } else {
                        snprintf(buffer, sizeof(buffer), "Att. AR !");
                    }
                    lcd2_print(buffer);
                }

                // Libérer le mutex I2C
                xSemaphoreGive(i2c_mutex);
            }
            // si des données importante sont à envoyer, elle le sont ici 
            //on envoie en ESP NOW sur l'autre LCD. Attention ! Sera envoyé seulement quand en unicast
            if(is_DATA2send == true){
                //remise à false du flag 
                is_DATA2send = false;
                ESP_LOGI(TAG,"des données vont être envoyés");
                if (xQueueSend(data_queue_2other_ESPNOW,&DataToEspNow,ESPNOW_MAXDELAY) != pdTRUE){
                    ESP_LOGW(TAG, "Send send queue fail");
                }
                // remise à 0 de l'octet d'alerte
                DataToEspNow = 0x00;
            }
        //}
    }
}
