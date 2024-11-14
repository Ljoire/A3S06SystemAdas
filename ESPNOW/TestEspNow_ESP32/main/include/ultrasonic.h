/**
 * @file ultrasonic.h
 * @brief Header file for ultrasonic sensor module.
 * 
 * This file provides the necessary functions and definitions to interface with an ultrasonic sensor for distance measurement.
 * It includes initialization, measurement, and error handling routines.
 */

/**
 * @defgroup Ultrasonic Sensor Module
 * @{
 */

/** 
 * @def TRIGGER_LOW_DELAY
 * @brief Duration of the TRIGGER pin signal in LOW state (in microseconds).
 */
#define TRIGGER_LOW_DELAY 4

/** 
 * @def TRIGGER_HIGH_DELAY
 * @brief Duration of the TRIGGER pin signal in HIGH state (in microseconds).
 */
#define TRIGGER_HIGH_DELAY 10

/** 
 * @def PING_TIMEOUT
 * @brief Maximum time to wait for the echo signal, in microseconds.
 */
#define PING_TIMEOUT 6000

/** 
 * @def ROUNDTRIP_M
 * @brief Distance traveled by the sound wave in one round trip in 1 second, in millimeters.
 */
#define ROUNDTRIP_M 5800.0f

/** 
 * @def ROUNDTRIP_CM
 * @brief Distance traveled by the sound wave in one round trip in centimeters.
 */
#define ROUNDTRIP_CM 58

/** 
 * @def SPEED_OF_SOUND_AT_0C_M_S
 * @brief Speed of sound in meters per second at 0°C.
 */
#define SPEED_OF_SOUND_AT_0C_M_S 331.4

/** 
 * @def ESP_ERR_ULTRASONIC_PING
 * @brief Error code for invalid state, when a previous ping is not ended.
 */
#define ESP_ERR_ULTRASONIC_PING         0x200

/** 
 * @def ESP_ERR_ULTRASONIC_PING_TIMEOUT
 * @brief Error code for ping timeout, when the device does not respond.
 */
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201

/** 
 * @def ESP_ERR_ULTRASONIC_ECHO_TIMEOUT
 * @brief Error code for echo timeout, when the distance is too big or the wave is scattered.
 */
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202

/**
 * @brief Device descriptor for the ultrasonic sensor.
 *
 * This structure contains the GPIO pin assignments for the ultrasonic sensor,
 * including the trigger and echo pins.
 */
typedef struct
{
    gpio_num_t trigger_pin; //!< GPIO output pin for the TRIGGER signal
    gpio_num_t echo_pin;    //!< GPIO input pin for the ECHO signal
} ultrasonic_sensor_t;

/**
 * @brief Initialize the ultrasonic sensor module.
 * 
 * This function configures the GPIO pins for TRIGGER and ECHO, setting up the sensor for distance measurements.
 *
 * @param dev Pointer to the device descriptor that holds the configuration for the ultrasonic sensor.
 * 
 * @return ESP_OK on success, or a specific error code on failure.
 */
esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev);   

/**
 * @brief Measure the time between the ping and the echo signal.
 * 
 * This function measures the time taken by the ultrasonic pulse to travel to an object and back.
 * The time is returned in microseconds.
 *
 * @param dev Pointer to the device descriptor containing the configuration of the ultrasonic sensor.
 * @param max_time_us Maximum time to wait for the echo, in microseconds.
 * @param[out] time_us Pointer to a variable where the measured time will be stored, in microseconds.
 * 
 * @return ESP_OK on success, or an error code on failure:
 *         - ESP_ERR_ULTRASONIC_PING: Invalid state (a previous ping is still ongoing).
 *         - ESP_ERR_ULTRASONIC_PING_TIMEOUT: The sensor did not respond within the expected time.
 *         - ESP_ERR_ULTRASONIC_ECHO_TIMEOUT: The echo took too long or the wave was scattered.
 */
esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, uint32_t max_time_us, uint32_t *time_us);  

/**
 * @brief Measure the distance to an object in meters.
 * 
 * This function calculates the distance to an object using the time it takes for the ultrasonic pulse to travel to the object
 * and back, based on the speed of sound.
 *
 * @param dev Pointer to the device descriptor containing the configuration of the ultrasonic sensor.
 * @param max_distance The maximum distance to measure, in meters.
 * @param[out] distance Pointer to a variable where the measured distance will be stored, in meters.
 * 
 * @return ESP_OK on success, or an error code on failure:
 *         - ESP_ERR_ULTRASONIC_PING: Invalid state (a previous ping is still ongoing).
 *         - ESP_ERR_ULTRASONIC_PING_TIMEOUT: The sensor did not respond within the expected time.
 *         - ESP_ERR_ULTRASONIC_ECHO_TIMEOUT: The distance is too large or the wave was scattered.
 */
esp_err_t ultrasonic_measure(const ultrasonic_sensor_t *dev, float max_distance, float *distance);

/**
 * @brief Task to handle ultrasonic sensor measurements asynchronously.
 * 
 * This task is responsible for managing ultrasonic sensor measurements in a non-blocking manner. It can be used in a 
 * multitasking environment to continuously measure distances using the ultrasonic sensor.
 *
 * @param pvParameters Parameters passed to the task, typically used to configure the task or to provide sensor settings.
 */
void ultrasonic_task(void *pvParameters); 

/** @} */  // End of Ultrasonic Sensor Module
