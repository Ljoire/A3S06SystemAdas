#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <driver/i2c.h>

#include "ultrasonic.h"
#include "espnow_example.h"

static const char *TAG = "UltrasonicSensor";

//include trouble
#include <esp32/rom/ets_sys.h>


static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

#define timeout_expired(start, len) ((esp_timer_get_time() - (start)) >= (len))

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define RETURN_CRITICAL(RES) do { PORT_EXIT_CRITICAL; return RES; } while(0)

#define MAX_DISTANCE_CM 500 // 5m max

#define TRIGGER_GPIO 5
#define ECHO_GPIO 18
#define TRIGGER_GPIO_2 19
#define ECHO_GPIO_2 33
#define TRIGGER_GPIO_3 25
#define ECHO_GPIO_3 23
#define SENSOR_SEND_QUEUE_SIZE 10

static i2c_port_t i2c_port = I2C_NUM_0;

// Bits de contrôle PCF8574
#define LCD_RS_BIT      0x01
#define LCD_RW_BIT      0x02
#define LCD_EN_BIT      0x04
#define LCD_BL_BIT      0x08
#define LCD_DATA_BITS   0xF0

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

static uint8_t backlight_state = LCD_BL_BIT;

static esp_err_t lcd_write_byte(uint8_t cmd, bool is_data) {
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

    return i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, data, 4, 1000 / portTICK_PERIOD_MS);
}

static void lcd_send_cmd(uint8_t cmd) {
    lcd_write_byte(cmd, false);
    if (cmd == LCD_CLEARDISPLAY || cmd == LCD_RETURNHOME) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
    } else {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void lcd_init(void) {
    // Configuration I2C
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

    // Attendre que le LCD soit prêt
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Séquence d'initialisation 4 bits
    uint8_t init_seq[] = {0x03, 0x03, 0x03, 0x02};
    for(int i = 0; i < 4; i++) {
        uint8_t data = (init_seq[i] << 4) | backlight_state;
        uint8_t with_en = data | LCD_EN_BIT;
        uint8_t without_en = data;
        
        uint8_t buf[2] = {with_en, without_en};
        i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // Configuration du LCD
    lcd_send_cmd(LCD_FUNCTIONSET | 0x08);        // 4-bit, 2 lignes, 5x8 pixels
    lcd_send_cmd(LCD_DISPLAYCONTROL | 0x04);     // Display ON, pas de curseur
    lcd_send_cmd(LCD_CLEARDISPLAY);              // Effacer l'écran
    lcd_send_cmd(LCD_ENTRYMODESET | 0x02);       // Entrée de gauche à droite

    ESP_LOGI(TAG, "LCD initialized successfully");
}

void lcd_clear(void) {
    lcd_send_cmd(LCD_CLEARDISPLAY);
}

void lcd_home(void) {
    lcd_send_cmd(LCD_RETURNHOME);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    static const uint8_t row_offsets[] = {0x00, 0x40};
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcd_print(const char* str) {
    while (*str) {
        lcd_write_byte(*str++, true);
    }
}

void lcd_backlight(bool on) {
    backlight_state = on ? LCD_BL_BIT : 0x00;
    uint8_t data = backlight_state;
    i2c_master_write_to_device(i2c_port, LCD_I2C_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}

esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, uint32_t max_time_us, uint32_t *time_us)
{
    CHECK_ARG(dev && time_us);

    PORT_ENTER_CRITICAL;

    // Ping: Low for 2..4 us, then high 10 us
    CHECK(gpio_set_level(dev->trigger_pin, 0));
    ets_delay_us(TRIGGER_LOW_DELAY);
    CHECK(gpio_set_level(dev->trigger_pin, 1));
    ets_delay_us(TRIGGER_HIGH_DELAY);
    CHECK(gpio_set_level(dev->trigger_pin, 0));

    // Previous ping isn't ended
    if (gpio_get_level(dev->echo_pin))
        RETURN_CRITICAL(ESP_ERR_ULTRASONIC_PING);

    // Wait for echo
    int64_t start = esp_timer_get_time();
    while (!gpio_get_level(dev->echo_pin))
    {
        if (timeout_expired(start, PING_TIMEOUT))
            RETURN_CRITICAL(ESP_ERR_ULTRASONIC_PING_TIMEOUT);
    }

    // got echo, measuring
    int64_t echo_start = esp_timer_get_time();
    int64_t time = echo_start;
    while (gpio_get_level(dev->echo_pin))
    {
        time = esp_timer_get_time();
        if (timeout_expired(echo_start, max_time_us))
            RETURN_CRITICAL(ESP_ERR_ULTRASONIC_ECHO_TIMEOUT);
    }
    PORT_EXIT_CRITICAL;

    *time_us = time - echo_start;

    return ESP_OK;
}

esp_err_t ultrasonic_measure(const ultrasonic_sensor_t *dev, float max_distance, float *distance)
{
    //verify the argument passed
    CHECK_ARG(dev && distance);

    uint32_t time_us;
    
    /*
    *   execute the measure function ultrasonic_measure_raw
    *   verify the distance is not higher than 5 meter and if encapsulte the error
    *   the error will be displayed on the task
    */
    CHECK(ultrasonic_measure_raw(dev, max_distance * ROUNDTRIP_M, &time_us));
    
    //return the distance for being used on the task
    *distance = time_us / ROUNDTRIP_M;

    return ESP_OK;
}


esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev)
{
    CHECK_ARG(dev);

    CHECK(gpio_set_direction(dev->trigger_pin, GPIO_MODE_OUTPUT));
    CHECK(gpio_set_direction(dev->echo_pin, GPIO_MODE_INPUT));

    return gpio_set_level(dev->trigger_pin, 0);
}


void ultrasonic_task(void *pvParameters)
{
    ultrasonic_sensor_t sensor1 = {
        .trigger_pin = TRIGGER_GPIO_2,
        .echo_pin = ECHO_GPIO_2
    };

    ultrasonic_sensor_t sensor2 = {
        .trigger_pin = TRIGGER_GPIO_2,
        .echo_pin = ECHO_GPIO_2
    };

    ultrasonic_sensor_t sensor3 = {
        .trigger_pin = TRIGGER_GPIO_3,
        .echo_pin = ECHO_GPIO_3
    };

    ultrasonic_init(&sensor1);
    ultrasonic_init(&sensor2);
    ultrasonic_init(&sensor3);

    lcd_init();

    while (true)
    {
        float distance1, distance2, distance3;
        uint8_t sensor_data[MAX_PAYLOAD_SIZE];
        char buffer[16];
        lcd_clear();


        esp_err_t res1 = ultrasonic_measure(&sensor1, MAX_DISTANCE_CM, &distance1);
        if (res1 != ESP_OK)
        {
            lcd_set_cursor(0, 0);
            snprintf(buffer, sizeof(buffer), "Erreur C1");
            lcd_print(buffer);
            switch (res1)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Impossible de ping (capteur dans un état invalide)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (aucun capteur trouvé)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (distance trop grande)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res1));
            }
        }
        else
        {
            lcd_set_cursor(0, 0); 
            snprintf(buffer, sizeof(buffer), "C1: %.1f cm", distance1 * 100);
            lcd_print(buffer);
            if (distance1 * 100 <= 20)
            {
                ESP_LOGI(TAG, "Capteur 1: Envoi des données dans la queue");
                if (xQueueSend(sensor_data_queue, sensor_data, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Échec de l'envoi des données du capteur 1 dans la queue");
                }
                ESP_LOGI(TAG, "Données du Capteur 1 envoyées dans la queue");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));

        esp_err_t res2 = ultrasonic_measure(&sensor2, MAX_DISTANCE_CM, &distance2);
        if (res2 != ESP_OK)
        {
            lcd_set_cursor(1, 0);
            snprintf(buffer, sizeof(buffer), "Erreur C2");
            lcd_print(buffer);
            switch (res2)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Impossible de ping (capteur dans un état invalide)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (aucun capteur trouvé)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (distance trop grande)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res2));
            }
        }
        else
        {
            printf("Capteur 2 Distance: %.04f cm\n", distance2 * 100);
            if (distance2 * 100 <= 20)
            {
                ESP_LOGI(TAG, "Capteur 2: Envoi des données dans la queue");
                if (xQueueSend(sensor_data_queue, sensor_data, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Échec de l'envoi des données du capteur 2 dans la queue");
                }
                ESP_LOGI(TAG, "Données du Capteur 2 envoyées dans la queue");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));

        esp_err_t res3 = ultrasonic_measure(&sensor3, MAX_DISTANCE_CM, &distance3);
        if (res3 != ESP_OK)
        {
            printf("Erreur Capteur 3 %d: ", res3);
            switch (res3)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Impossible de ping (capteur dans un état invalide)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (aucun capteur trouvé)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (distance trop grande)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res3));
            }
        }
        else
        {
            printf("Capteur 3 Distance: %.04f cm\n", distance3 * 100);
            if (distance3 * 100 <= 20)
            {
                ESP_LOGI(TAG, "Capteur 3: Envoi des données dans la queue");
                if (xQueueSend(sensor_data_queue, sensor_data, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Échec de l'envoi des données du capteur 3 dans la queue");
                }
                ESP_LOGI(TAG, "Données du Capteur 3 envoyées dans la queue");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
        
    }
}