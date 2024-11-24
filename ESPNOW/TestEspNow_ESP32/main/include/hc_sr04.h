/**
 * @file hc_sr04.h
 * @brief Header file for HC-SR04 ultrasonic sensor module interface.
 *
 * This file contains definitions, structures, and function declarations for
 * initializing and using HC-SR04 ultrasonic sensors to measure distances.
 */

#ifndef HC_SR04_H
#define HC_SR04_H

#include <driver/gpio.h>

/**
 * @brief GPIO pin assignments for the HC-SR04 sensors.
 *
 * The pins are categorized by the direction of the sensors:
 * - Left (`G`) and Right (`D`) for blind spot detection.
 * - Front (`AV`, `AVG`, `AVD`) and Rear (`AR`) for general detection.
 */
#define TRIGGER_GPIO_G    GPIO_NUM_5   /**< Trigger pin for Left sensor. */
#define ECHO_GPIO_G       GPIO_NUM_18 /**< Echo pin for Left sensor. */
#define TRIGGER_GPIO_D    GPIO_NUM_19 /**< Trigger pin for Right sensor. */
#define ECHO_GPIO_D       GPIO_NUM_23 /**< Echo pin for Right sensor. */

#define TRIGGER_GPIO_AV   GPIO_NUM_25 /**< Trigger pin for Front sensor. */
#define ECHO_GPIO_AV      GPIO_NUM_26 /**< Echo pin for Front sensor. */
#define TRIGGER_GPIO_AVG  GPIO_NUM_27 /**< Trigger pin for Front Left sensor. */
#define ECHO_GPIO_AVG     GPIO_NUM_14 /**< Echo pin for Front Left sensor. */
#define TRIGGER_GPIO_AVD  GPIO_NUM_12 /**< Trigger pin for Front Right sensor. */
#define ECHO_GPIO_AVD     GPIO_NUM_13 /**< Echo pin for Front Right sensor. */
#define TRIGGER_GPIO_AR   GPIO_NUM_4  /**< Trigger pin for Rear sensor. */
#define ECHO_GPIO_AR      GPIO_NUM_16 /**< Echo pin for Rear sensor. */

/**
 * @brief Structure representing an HC-SR04 sensor.
 */
typedef struct {
    gpio_num_t trigger_pin; /**< GPIO pin used as Trigger. */
    gpio_num_t echo_pin;    /**< GPIO pin used as Echo. */
} hc_sr04_t;

/**
 * @brief Structure containing distance data from sensors.
 */
typedef struct {
    float dist_av;  /**< Distance measured by the Front sensor (cm). */
    float dist_g;   /**< Distance measured by the Left sensor (cm). */
    float dist_d;   /**< Distance measured by the Right sensor (cm). */
    float dist_avg; /**< Distance measured by the Front Left sensor (cm). */
    float dist_avd; /**< Distance measured by the Front Right sensor (cm). */
    float dist_ar;  /**< Distance measured by the Rear sensor (cm). */
    bool data_ready; /**< Indicates whether the data is ready to be processed. */
} sensor_data_t;

/**
 * @brief Initializes an HC-SR04 sensor.
 *
 * Configures the Trigger and Echo pins of the sensor.
 *
 * @param sensor Pointer to the `hc_sr04_t` structure representing the sensor.
 * @return int Returns 0 on success, or an error code on failure.
 */
int hc_sr04_init(hc_sr04_t *sensor);

/**
 * @brief Measures the distance using an HC-SR04 sensor.
 *
 * Sends a trigger signal and measures the time taken for the echo signal to return.
 *
 * @param sensor Pointer to the `hc_sr04_t` structure representing the sensor.
 * @return float Distance measured in centimeters. Returns negative values on errors.
 */
float measure_distance_cm(hc_sr04_t *sensor);

/**
 * @brief Task for continuously reading sensor data.
 *
 * Reads data from multiple sensors and processes them periodically.
 *
 * @param pvParameters Pointer to the `sensor_data_t` structure for storing sensor data.
 */
static void sensor_task(void *pvParameters);

/**
 * @brief Validates the measured distance.
 *
 * Checks if the distance is within the valid range for the HC-SR04 sensor.
 *
 * @param distance Measured distance in centimeters.
 * @return bool Returns true if the distance is valid, false otherwise.
 */
static bool is_valid_measurement(float distance);

#endif // HC_SR04_H
