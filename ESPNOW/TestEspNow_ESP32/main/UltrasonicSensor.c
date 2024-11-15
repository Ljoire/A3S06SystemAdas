#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

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

    while (true)
    {
        float distance1, distance2, distance3;
        uint8_t sensor_data[MAX_PAYLOAD_SIZE];

        esp_err_t res1 = ultrasonic_measure(&sensor1, MAX_DISTANCE_CM, &distance1);
        if (res1 != ESP_OK)
        {
            printf("Erreur Capteur 1 %d: ", res1);
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
            printf("Capteur 1 Distance: %.04f cm\n", distance1 * 100);
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

        vTaskDelay(pdMS_TO_TICKS(10));

        esp_err_t res2 = ultrasonic_measure(&sensor2, MAX_DISTANCE_CM, &distance2);
        if (res2 != ESP_OK)
        {
            printf("Erreur Capteur 2 %d: ", res2);
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