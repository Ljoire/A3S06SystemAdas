#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 6000
#define ROUNDTRIP_M 5800.0f
#define ROUNDTRIP_CM 58
#define SPEED_OF_SOUND_AT_0C_M_S 331.4 // Speed of sound in m/s at 0 degrees Celsius

#define ESP_ERR_ULTRASONIC_PING         0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202

static QueueHandle_t sensor_data_queue;

/**
 * Device descriptor
 */
typedef struct
{
    gpio_num_t trigger_pin; //!< GPIO output pin for trigger
    gpio_num_t echo_pin;    //!< GPIO input pin for echo
} ultrasonic_sensor_t;

/**
 * @brief Init ranging module
 *
 * @param dev Pointer to the device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev);   

/**
 * @brief Measure time between ping and echo
 *
 * @param dev Pointer to the device descriptor
 * @param max_time_us Maximal time to wait for echo
 * @param[out] time_us Time, us
 * @return `ESP_OK` on success, otherwise:
 *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
 *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
 *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
 */
esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, uint32_t max_time_us, uint32_t *time_us);  

/**
 * @brief Measure distance in meters
 *
 * @param dev Pointer to the device descriptor
 * @param max_distance Maximal distance to measure, meters
 * @param[out] distance Distance in meters
 * @return `ESP_OK` on success, otherwise:
 *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
 *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
 *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
 */
esp_err_t ultrasonic_measure(const ultrasonic_sensor_t *dev, float max_distance, float *distance);

void ultrasonic_task(void *pvParameters);