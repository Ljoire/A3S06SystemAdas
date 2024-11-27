/**
 * @file hc_sr04.c
 * @author ***** (you@domain.com)
 * @brief Implementation of HC-SR04 ultrasonic sensor interface.
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "hc_sr04.h"
#include <esp_system.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>
#include <math.h>

/* FreeRTOS Libraries */
#include "freertos/FreeRTOS.h"   /**< FreeRTOS API for real-time operating system tasks and queues. */
#include "freertos/semphr.h"     /**< FreeRTOS API for semaphore and mutex operations. */
#include "freertos/timers.h"     /**< FreeRTOS API for software timers. */
#include "freertos/task.h"       /**< FreeRTOS API for task management. */


#include "esp_timer.h"           /**< Timer library for high-resolution timing and delays. */
#include <esp_log.h>
static const char *TAG = "LCD2";

/**
 * @brief Validates a measured distance.
 *
 * Ensures the measured distance is within the sensor's valid range (2-400 cm).
 *
 * @param distance Measured distance in centimeters.
 * @return bool Returns true if the distance is valid, false otherwise.
 */
bool is_valid_measurement(float distance) {
    return (distance >= 2.0 && distance <= 400.0);
}

/**
 * @brief Initializes an HC-SR04 sensor.
 *
 * Configures the Trigger pin as output and the Echo pin as input.
 *
 * @param sensor Pointer to the `hc_sr04_t` structure representing the sensor.
 * @return int Returns 0 on success, or an error code on failure.
 */
int hc_sr04_init(hc_sr04_t *sensor) {
    if (sensor == NULL) {
        printf("Error: Sensor pointer is NULL\n");
        return -1;
    }

    // Configure Trigger pin
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << sensor->trigger_pin),
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("Error: Unable to configure Trigger pin\n");
        return -3;
    }

    // Configure Echo pin
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << sensor->echo_pin);
    if (gpio_config(&io_conf) != ESP_OK) {
        printf("Error: Unable to configure Echo pin\n");
        return -4;
    }

    return 0;
}

/**
 * @brief Measures the distance in centimeters using an HC-SR04 sensor.
 *
 * Sends a 10µs pulse on the Trigger pin and measures the echo duration to calculate the distance.
 *
 * @param sensor Pointer to the `hc_sr04_t` structure representing the sensor.
 * @return float Distance in centimeters, or a negative value on error.
 */
float measure_distance_cm(hc_sr04_t *sensor) {
    if (sensor == NULL) {
        printf("Error: Sensor pointer is NULL\n");
        return -1.0;
    }

    gpio_set_level(sensor->trigger_pin, 1);
    ets_delay_us(10); // Trigger pulse width
    gpio_set_level(sensor->trigger_pin, 0);

    uint64_t timeout = esp_timer_get_time() + 30000; // 30ms timeout

    // Wait for echo start
    while (gpio_get_level(sensor->echo_pin) == 0) {
        if (esp_timer_get_time() > timeout) {
            printf("Error: Echo signal timeout\n");
            return -3.0;
        }
    }

    uint64_t echo_start = esp_timer_get_time();

    // Wait for echo end
    while (gpio_get_level(sensor->echo_pin) == 1) {
        if (esp_timer_get_time() > timeout) {
            printf("Error: Echo signal too long\n");
            return -4.0;
        }
    }

    uint64_t echo_end = esp_timer_get_time();

    float distance_cm = (echo_end - echo_start) / 58.0;

    // Validate distance
    if (!is_valid_measurement(distance_cm)) {
        printf("Error: Invalid distance (%.2f cm)\n", distance_cm);
        return -5.0;
    }

    return distance_cm;
}

/**
 * @brief Task for reading distances from multiple sensors.
 *
 * Continuously reads data from the sensors, validates the measurements, and updates the `sensor_data_t` structure.
 *
 * @param pvParameters Pointer to the `sensor_data_t` structure for storing sensor data.
 */
void sensor_task(void *pvParameters) {
    sensor_data_t *sensor_data = (sensor_data_t *)pvParameters;
    if (sensor_data == NULL) {
        ESP_LOGE(TAG, "Invalid sensor data pointer");
    }
    hc_sr04_t sensors[] = {
        {TRIGGER_GPIO_AV, ECHO_GPIO_AV},
        {TRIGGER_GPIO_G, ECHO_GPIO_G},
        {TRIGGER_GPIO_D, ECHO_GPIO_D},
        {TRIGGER_GPIO_AVG, ECHO_GPIO_AVG},
        {TRIGGER_GPIO_AVD, ECHO_GPIO_AVD},
        {TRIGGER_GPIO_AR, ECHO_GPIO_AR}
    };

    for (int i = 0; i < 6; i++) {
        if (hc_sr04_init(&sensors[i]) != 0) {
            printf("Error: Failed to initialize sensor %d\n", i);
        }
    }

    while (1) {
        for (int i = 0; i < 6; i++) {
            (&sensor_data->dist_av)[i] = measure_distance_cm(&sensors[i]);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        sensor_data->data_ready = true;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
