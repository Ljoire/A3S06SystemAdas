/**
 * @file espnow_example.h
 * @brief Header file for ESPNOW communication example.
 * 
 * This file provides the necessary includes and initial definitions for
 * configuring and using ESPNOW protocol with FreeRTOS and ESP-IDF. The code 
 * is designed for reliable communication between ESP32 devices using ESPNOW,
 * a connectionless communication protocol by Espressif.
 *
 * @note This example code is in the Public Domain (or CC0 licensed, at your option.)
 *       The software is provided "AS IS", without any warranties or guarantees.
 */


#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

/* Standard Libraries */
#include <stdlib.h>          /**< Standard library for memory management. */
#include <time.h>            /**< Time library for timing operations and delays. */
#include <string.h>          /**< String library for memory manipulation functions. */
#include <assert.h>          /**< Assert library for debugging and condition validation. */

/* FreeRTOS Libraries */
#include "freertos/FreeRTOS.h"   /**< FreeRTOS API for real-time operating system tasks and queues. */
#include "freertos/semphr.h"     /**< FreeRTOS API for semaphore and mutex operations. */
#include "freertos/timers.h"     /**< FreeRTOS API for software timers. */
#include "freertos/task.h"       /**< FreeRTOS API for task management. */

/* ESP-IDF Libraries */
#include "nvs_flash.h"           /**< Non-volatile storage (NVS) library for flash memory management. */
#include "esp_random.h"          /**< ESP-IDF library for generating random numbers. */
#include "esp_event.h"           /**< Event handling library for managing system events. */
#include "esp_netif.h"           /**< Network interface library for network configuration and control. */
#include "esp_wifi.h"            /**< Wi-Fi library for configuring Wi-Fi connections. */
#include "esp_log.h"             /**< Logging library for message logging and debugging. */
#include "esp_mac.h"             /**< MAC address handling library. */
#include "esp_now.h"             /**< ESPNOW library for connectionless communication. */
#include "esp_crc.h"             /**< CRC library for cyclic redundancy check calculations. */

/* Peripheral Libraries */
#include "driver/gpio.h"         /**< GPIO library for handling general-purpose input/output. */
#include "esp_timer.h"           /**< Timer library for high-resolution timing and delays. */
#include "esp_rom_sys.h"         /**< System library for low-level ROM functions. */





/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

/**
 * @brief Taille maximale de la file d'attente pour l'envoi des données des capteurs.
 *
 * Cette valeur détermine le nombre maximal de messages qui peuvent être stockés dans la file d'attente
 * `sensor_data_queue` avant d'être traités ou envoyés par ESPNOW.
 */
#define SENSOR_SEND_QUEUE_SIZE      10

/**
 * @brief Taille maximale de la file d'attente pour les événements ESPNOW.
 *
 * Définit le nombre maximal d'événements qui peuvent être en attente dans `s_example_espnow_queue`
 * avant d'être traités. Cela inclut les événements de type envoi et réception.
 */
#define ESPNOW_QUEUE_SIZE           6

/**
 * @brief Taille maximale de la charge utile (payload) des données ESPNOW.
 *
 * Cette constante fixe la taille de la charge utile `payload` dans la structure `example_espnow_data_t`.
 * Elle est utilisée pour s'assurer que la charge utile ne dépasse pas la taille spécifiée.
 */
#define MAX_PAYLOAD_SIZE            8

/**
 * @brief Délai maximal pour l'envoi des paquets ESPNOW, en millisecondes.
 *
 * Ce délai limite le temps d'attente avant qu'une tentative d'envoi ne soit abandonnée
 * en cas de congestion ou d'échec de communication.
 */
#define ESPNOW_MAXDELAY             512

/**
 * @brief Test si une adresse MAC correspond à une adresse de diffusion (broadcast).
 *
 * Cette macro compare une adresse MAC donnée à une adresse de diffusion préconfigurée.
 *
 * @param addr Adresse MAC à vérifier.
 * @return true si l'adresse MAC est une adresse de diffusion, false sinon.
 */
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

/**
 * @brief Adresse MAC de diffusion pour l'ESPNOW.
 *
 * Cette adresse est utilisée pour identifier les messages envoyés en mode broadcast.
 */
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/**
 * @brief File d'attente utilisée pour gérer les événements de callback ESPNOW.
 *
 * Les événements de callback (envoi et réception) sont ajoutés à cette file d'attente
 * pour être traités dans une tâche de plus faible priorité.
 */
static QueueHandle_t s_example_espnow_queue;

/**
 * @brief File d'attente entre les données des capteurs et la tâche d'envoi ESPNOW.
 *
 * Déclarée en externe pour pouvoir être utilisée dans d'autres fichiers source.
 */
extern QueueHandle_t sensor_data_queue;

extern QueueHandle_t receive_calback_queu;

/**
 * @brief Identifie le type d'événement dans le callback ESPNOW.
 *
 * Cette énumération est utilisée pour différencier les événements de callback d'envoi et de réception.
 * - 0 : Callback d'envoi.
 * - 1 : Callback de réception.
 */
typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,   /**< 0 : Événement déclenché pour le callback d'envoi de données. */
    EXAMPLE_ESPNOW_RECV_CB    /**< 1 : Événement déclenché pour le callback de réception de données. */
} example_espnow_event_id_t;

/**
 * @brief Structure contenant les informations pour le callback d'envoi ESPNOW.
 */
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];   /**< Adresse MAC du destinataire. */
    esp_now_send_status_t status;         /**< Statut de l'envoi ESPNOW (succès ou échec). */
} example_espnow_event_send_cb_t;

/**
 * @brief Structure contenant les informations pour le callback de réception ESPNOW.
 */
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];   /**< Adresse MAC de l'émetteur. */
    uint8_t *data;                        /**< Données reçues. */
    int data_len;                         /**< Longueur des données reçues. */
} example_espnow_event_recv_cb_t;

/**
 * @brief Union contenant les informations des événements d'envoi et de réception.
 */
typedef union {
    example_espnow_event_send_cb_t send_cb;   /**< Informations du callback d'envoi. */
    example_espnow_event_recv_cb_t recv_cb;   /**< Informations du callback de réception. */
} example_espnow_event_info_t;

/**
 * @brief Structure d'un événement ESPNOW envoyé à la file d'attente.
 *
 * Cette structure encapsule le type d'événement et ses informations associées
 * pour être traitée dans une tâche ESPNOW.
 */
typedef struct {
    example_espnow_event_id_t id;         /**< Type d'événement (envoi ou réception). */
    example_espnow_event_info_t info;     /**< Informations sur l'événement. */
} example_espnow_event_t;

/**
 * @brief Enumération définissant le type de données ESPNOW.
 *
 * - 0 : Données de diffusion (broadcast).
 * - 1 : Données de transmission directe (unicast).
 * - 2 : Maximum (utilisé comme limite).
 */
enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,   /**< 0 : Type de données de diffusion. */
    EXAMPLE_ESPNOW_DATA_UNICAST,     /**< 1 : Type de données de transmission directe. */
    EXAMPLE_ESPNOW_DATA_MAX          /**< 2 : Type maximum, utilisé comme limite. */
};

/**
 * @brief Structure de la trame de données envoyée via ESPNOW.
 *
 * Cette structure représente la trame de données, stockée dans le champ `buffer` de `example_espnow_send_param_t`,
 * et envoyée aux autres dispositifs. La structure est compacte avec l'attribut `__attribute__((packed))`.
 *
 * **Champs**:
 * - `type` : Indique si les données sont pour une diffusion (broadcast) ou une transmission directe (unicast).
 * - `state` : Indique si des données de diffusion ont été reçues.
 * - `seq_num` : Numéro de séquence de la trame, utile pour suivre l'ordre des paquets.
 * - `crc` : Somme de contrôle CRC16 pour vérifier l'intégrité des données.
 * - `magic` : Nombre magique pour déterminer le dispositif cible de la transmission directe.
 * - `payload` : Charge utile (données réelles) de la trame, d'une longueur fixe de 8 octets.
 */
typedef struct {
    uint8_t type;                         /**< Indique si les données ESPNOW sont pour diffusion ou unicast. */
    uint8_t state;                        /**< État de réception des données de diffusion. */
    uint16_t seq_num;                     /**< Numéro de séquence de la trame ESPNOW. */
    uint16_t crc;                         /**< Somme de contrôle CRC16 des données ESPNOW. */
    uint8_t magic;                        /**< Nombre magique pour sélectionner le dispositif en unicast. */
    uint8_t payload[8];                   /**< Charge utile réelle des données ESPNOW. */
} __attribute__((packed)) example_espnow_data_t;




/**
 * @brief Structure containing all parameters for sending and receiving ESPNOW data.
 *
 * This structure defines the configuration parameters required for sending or receiving data over ESPNOW.
 * All devices involved in the ESPNOW communication should have identical structures for consistency.
 * Each element can be modified based on the specific needs of the application.
 *
 * **Members**:
 * - `dest_mac` : MAC address of the destination device. Used to specify the receiver's address in unicast transmissions.
 * - `count` : Total count of unicast ESPNOW packets to be sent. Defines the number of packets to be transmitted in a unicast session.
 * - `delay` : Delay (in milliseconds) between sending consecutive ESPNOW packets. Controls the rate of transmission.
 * - `state` : Indicates if a broadcast ESPNOW data packet has been received. Helps manage received broadcast events.
 * - `magic` : Magic number used to uniquely identify devices for selective unicast transmission.
 * - `len` : Length of the ESPNOW data packet to be sent, in bytes.
 * - `buffer` : Pointer to the data buffer containing the ESPNOW data to be sent.
 * - `unicast` : Boolean flag indicating whether the data should be sent as a unicast transmission.
 * - `broadcast` : Boolean flag indicating whether the data should be sent as a broadcast transmission.
 * - `pingpong` : Boolean flag defining the sender and receiver roles, allowing the devices to toggle between these roles for alternating communication.
 * - `error` : Boolean flag indicating if an error occurred during data transmission, useful for error handling and retransmission logic.
 *
 * **Usage**:
 * This structure is used to configure ESPNOW communication parameters, allowing flexibility for both unicast and broadcast transmissions.
 * It supports toggling between devices using the `pingpong` parameter and also manages packet sending delays for controlled transmission rates.
 */
typedef struct {
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   /**< MAC address of the destination device. */
    uint16_t count;                       /**< Total count of unicast ESPNOW packets to be sent. */
    uint16_t delay;                       /**< Delay between sending two ESPNOW packets, in milliseconds. */
    uint8_t state;                        /**< Indicates if broadcast ESPNOW data has been received. */
    uint8_t magic;                        /**< Magic number used to determine the device to send unicast ESPNOW data to. */
    uint8_t len;                          /**< Length of the ESPNOW data to be sent, in bytes. */
    uint8_t *buffer;                      /**< Pointer to the buffer containing the ESPNOW data to be sent. */
    bool unicast;                         /**< Indicates if unicast ESPNOW data should be sent. */
    bool broadcast;                       /**< Indicates if broadcast ESPNOW data should be sent. */
    bool pingpong;                        /**< Defines the sender and receiver roles, toggling between devices. */
    bool error;                           /**< Indicates if an error occurred during data transmission. */
} example_espnow_send_param_t;

#endif

/**
 * @brief Initializes Wi-Fi for ESPNOW.
 *
 * This function performs the essential initialization steps for Wi-Fi, which must be completed
 * before using ESPNOW. It configures the Wi-Fi mode, sets the channel, and enables long-range mode if configured.
 *
 * **Steps**:
 * - Initializes the network interface with `esp_netif_init`.
 * - Creates the default event loop with `esp_event_loop_create_default`.
 * - Configures Wi-Fi with default settings, sets storage mode, and starts Wi-Fi in the specified mode (`ESPNOW_WIFI_MODE`).
 * - Sets the channel for ESPNOW communication as defined by `CONFIG_ESPNOW_CHANNEL`.
 * - Optionally enables long-range protocol if `CONFIG_ESPNOW_ENABLE_LONG_RANGE` is set in the configuration.
 *
 * **Notes**:
 * - This initialization function is a prerequisite for using ESPNOW. It must be called before any ESPNOW operations.
 * - The `CONFIG_ESPNOW_CHANNEL` parameter defines the Wi-Fi channel used for ESPNOW communications.
 *
 * **Configuration Parameters**:
 * - `CONFIG_ESPNOW_CHANNEL`: The Wi-Fi channel used for ESPNOW.
 * - `CONFIG_ESPNOW_ENABLE_LONG_RANGE`: Enables long-range Wi-Fi protocols (802.11 LR) for extended range communication.
 */
void example_wifi_init(void);


/**
 * @brief Callback function for ESPNOW send events.
 *
 * This function is triggered upon the completion of a data transmission via ESPNOW. It runs within the WiFi task context.
 * Lengthy operations should not be executed here to avoid delaying the WiFi task. Instead, necessary information
 * should be posted to a queue for further processing in a separate lower-priority task.
 *
 * **Parameters**:
 * @param mac_addr Pointer to the MAC address of the destination device that received the data.
 * @param status Status of the send operation. It indicates whether the transmission was successful (`ESP_NOW_SEND_SUCCESS`)
 *               or unsuccessful (`ESP_NOW_SEND_FAIL`).
 *
 * **Notes**:
 * - This callback provides essential feedback on the transmission status, allowing the application to handle
 *   retransmission or error logging as needed.
 * - Users should minimize operations in this callback to prevent delays in the WiFi task.
 */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

/**
 * @brief Callback function for ESPNOW receive events.
 *
 * This function is triggered when data is received via ESPNOW. It also runs within the WiFi task context,
 * so it should avoid lengthy operations. Instead, received data is posted to a queue for further processing
 * in a separate task.
 *
 * **Parameters**:
 * @param recv_info Pointer to the `esp_now_recv_info_t` structure containing reception information.
 *                  This includes the source MAC address of the sender and the destination MAC address.
 *                  If the source MAC is unknown, it can be added to the peer list for future communications.
 * @param data Pointer to the data received. The data is expected to follow the `espnow_data_t` format
 *             if the sender is running the same program.
 * @param len Length of the received data. This length is used for assertions and validation.
 *
 * **Notes**:
 * - This callback is intended to handle the reception event briefly and to offload detailed processing to a queue-based task.
 * - By posting received data to a queue, the application can process it asynchronously and efficiently.
 * - If the sender's MAC address is not recognized, it can be dynamically added to the peer list, enabling automatic
 *   pairing with new devices within range.
 */
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);




/**
 * @brief Parses received ESPNOW data and extracts various components from the data buffer.
 *
 * This function is used to parse the received ESPNOW data buffer. It extracts different parts of the data,
 * including the state, sequence number, magic number, and payload (the actual message sent by the sender).
 * Additionally, the function performs CRC validation to ensure the integrity of the received data.
 *
 * **Function Flow**:
 * 1. The function first checks if the received data length is valid by ensuring it is at least as large as the size of the `example_espnow_data_t` structure.
 * 2. It then extracts the state, sequence number, magic number, and payload from the data buffer.
 * 3. The function calculates the CRC checksum for the data buffer and compares it to the received CRC to ensure data integrity.
 * 4. If the calculated CRC matches the received CRC, the function returns the type of data (broadcast or unicast).
 * 5. If the CRC does not match or the data is invalid, the function returns `-1` to indicate an error.
 *
 * **Parameters**:
 * @param data Pointer to the received data buffer, which contains the ESPNOW message.
 * @param data_len The total length of the received data buffer to process and verify.
 * @param state Pointer to a variable where the state of the sender (broadcast or unicast) will be stored.
 * @param seq Pointer to a variable where the sequence number will be stored.
 * @param magic Pointer to a variable where the magic number from the sender will be stored.
 * @param payload Pointer to a buffer where the payload (the actual message) will be copied.
 * @param payload_len The length of the buffer where the payload will be copied.
 *
 * **Return Value**:
 * - Returns the type of data: `EXAMPLE_ESPNOW_DATA_BROADCAST` (for broadcast) or `EXAMPLE_ESPNOW_DATA_UNICAST` (for unicast) if the CRC is valid.
 * - Returns `-1` if the CRC validation fails or if the data is invalid.
 */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint8_t *magic, uint8_t *payload, uint8_t payload_len);




/**
 * @brief Prepares the ESPNOW data to be sent.
 *
 * This function prepares the data for transmission by constructing a data structure that will hold the message.
 * The function first copies relevant information, such as the type of message (broadcast or unicast), sequence number, 
 * state, magic number, and payload. It then calculates the CRC checksum for the data to ensure integrity during transmission.
 * The data will be sent using `espnow_send` after being prepared.
 *
 * **Function Flow**:
 * 1. The `send_param->buffer` is treated as a pointer to the `example_espnow_data_t` structure, which holds the message.
 * 2. The type of data (broadcast or unicast) is determined by checking the destination MAC address (`send_param->dest_mac`).
 * 3. The sequence number is assigned based on the data type (either broadcast or unicast).
 * 4. The magic number and other parameters are copied into the buffer.
 * 5. The payload (message data) is copied into the buffer.
 * 6. The function calculates a CRC checksum for the data structure to ensure integrity.
 *
 * **Key Features**:
 * - Handles both broadcast and unicast data.
 * - Calculates a CRC checksum to ensure the integrity of the data.
 * - Copies the payload into the prepared buffer before sending.
 * - Optionally fills remaining bytes of the payload with random values (though commented out in the code).
 *
 * **Parameters**:
 * @param send_param Pointer to the sending parameters structure (`example_espnow_send_param_t`), which contains the configuration for the transmission.
 * @param mes2send Pointer to the message (data) to be sent, represented as `uint8_t*`. The size of the message is known and passed through the pointer.
 */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param, uint8_t *mes2send);






/**
 * @brief Concatenates the data preparation and sending, with error checking.
 *
 * This function prepares the data using `espnow_data_prepare` and then sends it using `espnow_send`. 
 * It also performs error checking to ensure that the sending process was successful.
 * The function also introduces a delay before sending the data if specified in the sending parameters.
 * 
 * **Function Flow**:
 * 1. If the sending parameters include a delay (`send_param->delay`), the function waits for the specified time.
 * 2. The destination MAC address is copied into the `send_param` structure.
 * 3. The message is prepared for transmission using `espnow_data_prepare`.
 * 4. The prepared message is sent using `esp_now_send`.
 * 5. If the sending operation fails, an error message is logged, and the task is deinitialized and deleted.
 * 6. If the sending is successful, the function returns `0`. Otherwise, it returns `-1` to indicate failure.
 *
 * **Key Features**:
 * - Handles the preparation of the data and its transmission through ESPNOW.
 * - Includes error handling to ensure the sending process completes successfully.
 * - Allows configurable delays between sending operations through the `send_param` structure.
 * 
 * **Parameters**:
 * @param send_param Pointer to the sending parameters structure (`example_espnow_send_param_t`), which contains the configuration for the transmission.
 * @param message Pointer to the message (data) to be sent, represented as `uint8_t*`. The message size is known and passed through the pointer.
 * @param desMAC Pointer to the destination MAC address for sending the data. This is typically received from a previous callback or set to broadcast if needed.
 * 
 * @return int Returns `0` if the sending operation was successful, and `-1` if there was an error during the sending process.
 */
int espnow_datasending(example_espnow_send_param_t *send_param, uint8_t *message, const uint8_t *desMAC);





/**
 * @brief Task for managing both ESPNOW sending and receiving callbacks.
 * 
 * This task is responsible for handling the events for both sending and receiving data 
 * via ESPNOW. It manages the different types of events, including the callback for successful 
 * sending (EXAMPLE_ESPNOW_SEND_CB) and receiving (EXAMPLE_ESPNOW_RECV_CB).
 *
 * **Task Flow**:
 * 1. The task waits for events from the ESPNOW queue.
 * 2. If a send callback is received (EXAMPLE_ESPNOW_SEND_CB), it checks the status of the transmission and whether the data is a broadcast.
 *    - If there was an error in sending or if the data is broadcast but the sending parameter is not configured for broadcasting, the task handles the error accordingly.
 *    - Otherwise, it either continues sending the data or stops transmission if the count reaches zero.
 * 3. If a receive callback is received (EXAMPLE_ESPNOW_RECV_CB), the task processes the received data:
 *    - It checks if the received data is a broadcast or unicast and takes appropriate actions, such as adding a new peer or sending an acknowledgment (ACK).
 *    - For unicast data, it also parses the message and prepares a response.
 *    - If the received data corresponds to a broadcast, the task may initiate further unicast transmission.
 * 
 * The task uses a queue to handle events and ensures synchronization and error checking when sending or receiving data.
 *
 * **Key Features**:
 * - Handles both broadcast and unicast data.
 * - Supports ping-pong communication by checking the state and controlling when to send.
 * - Can dynamically add peers based on received broadcast data.
 * - Performs error handling for send failures and transmission status.
 *
 * @param pvParameter Pointer to the sending parameters (`example_espnow_send_param_t`), 
 *                    used to configure the ESPNOW data for transmission. Typically, 
 *                    this is the result of the `SendingParamCreator` function.
 */
static void example_espnow_task(void *pvParameter);


/**
 * @brief Task used for scheduling the sending through ESPNOW.
 * 
 * This task is responsible for receiving data from a queue and sending it through ESPNOW, 
 * with a specific delay between each sending. It is currently used for the sensor task, 
 * where sensor data is queued and sent with a fixed delay. The task also checks if the 
 * queue creation fails and ensures that the pointer is not NULL before proceeding.
 *
 * The task follows this flow:
 * 1. It creates a queue to hold the sensor data that needs to be sent.
 * 2. It waits for data to be received from the queue using `xQueueReceive`.
 * 3. When data is available, it calls `espnow_datasending` to send the data via ESPNOW.
 * 4. After sending, it waits for a specified delay before attempting to send more data.
 *
 * @param pvParameter Pointer to the sending parameters (`example_espnow_send_param_t`), 
 *                    used to configure the ESPNOW data for transmission. This is typically 
 *                    the result of `SendingParamCreator` function.
 */
void espnow_sending_task(void *pvParameter);


/**
 * @brief Initializes ESPNOW and sets up the peer list.
 * 
 * This function initializes the ESPNOW protocol, registers the sending and receiving callback functions, 
 * and sets the primary master key (PMK). It also creates a queue for handling events and adds a broadcast peer 
 * to the peer list. If the peer addition is successful, it creates tasks for handling the sending and receiving of data.
 * This function is typically called from a task related to ESPNOW to set up the communication environment.
 *
 * The following steps are performed in this function:
 * - A queue is created for ESPNOW event handling.
 * - ESPNOW is initialized and the send and receive callback functions are registered.
 * - Power-saving options are configured if enabled in the ESP32 settings.
 * - The primary master key is set for the encryption of ESPNOW communication.
 * - A broadcast peer (with a predefined MAC address) is added to the peer list.
 * - Two tasks are created: one for receiving ESPNOW data and one for sending data.
 *
 * @param pvParameter Pointer to the sending parameters (`example_espnow_send_param_t`), used to configure the ESPNOW data. This should be passed to other functions requiring the sending parameters.
 *
 * @return esp_err_t Returns `ESP_OK` if initialization is successful, and `ESP_FAIL` if any step fails.
 */
esp_err_t example_espnow_init(void *pvParameter);


/* Unitialize the espnow and free all the data*/

/**
 * @brief Deinitializes the sending parameters and releases resources.
 *
 * This function frees the allocated memory for the sending parameters and buffer, 
 * deletes the semaphore, and deinitializes the ESP-NOW interface.
 *
 * @param send_param Pointer to the sending parameters structure (`example_espnow_send_param_t`) to be deinitialized.
 */
void example_espnow_deinit(example_espnow_send_param_t *send_param);


/**
 * @brief Initializes and creates the sending parameters for use by other functions.
 *
 * This function allocates memory for the sending parameters (`example_espnow_send_param_t`), 
 * initializes the fields, and prepares the data to be sent. It also checks for memory allocation failures.
 *
 * @return example_espnow_send_param_t* Pointer to the initialized sending parameters structure.
 *         Returns `NULL` if there is a failure in memory allocation.
 */
example_espnow_send_param_t *SendingParamCreator(void);


