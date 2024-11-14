/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"



/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define SENSOR_SEND_QUEUE_SIZE      10
#define ESPNOW_QUEUE_SIZE           6
#define MAX_PAYLOAD_SIZE            8
#define ESPNOW_MAXDELAY             512

/**
 * @brief Testing if the MAC adress correspond to a broadcast adress
 * 
 */
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


/**
 * @brief The queue used for the callback function 
 * 
 */
static QueueHandle_t s_example_espnow_queue;

/**
 * @brief the queue between the sensor queue and the sending task. 
 *        Decalared as extern for use it on other source file
 * 
 */
extern QueueHandle_t sensor_data_queue;

/**
 * @brief Identifies the type of ESPNOW event in the callback.
 *
 * This enumeration is used to determine if the callback corresponds to a sending or receiving event.
 * - 0: Send callback.
 * - 1: Receive callback.
 */
typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,   /**< 0: Event triggered for sending data callback. */
    EXAMPLE_ESPNOW_RECV_CB    /**< 1: Event triggered for receiving data callback. */
} example_espnow_event_id_t;


typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} example_espnow_event_recv_cb_t;

typedef union {
    example_espnow_event_send_cb_t send_cb;
    example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    example_espnow_event_id_t id;
    example_espnow_event_info_t info;
} example_espnow_event_t;

/**
 * @brief Enumeration defining the type of ESPNOW data.
 *
 * The data type can be:
 * - 0: Broadcast data.
 * - 1: Unicast data.
 * - 2: Maximum data (used as a boundary or limit).
 */
enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,   /**< 0: Broadcast data type. */
    EXAMPLE_ESPNOW_DATA_UNICAST,     /**< 1: Unicast data type. */
    EXAMPLE_ESPNOW_DATA_MAX          /**< 2: Maximum data type, used as a limit indicator. */
};


/**
 * @brief The data frame structure sent to other devices, stored in the *buffer field of the structure @var example_espnow_send_param_t.
 */
typedef struct {
    uint8_t type;                         /**< Indicates whether the ESPNOW data is for broadcast or unicast. */
    uint8_t state;                        /**< Indicates if broadcast ESPNOW data has been received. */
    uint16_t seq_num;                     /**< Sequence number of the ESPNOW data frame. */
    uint16_t crc;                         /**< CRC16 checksum of the ESPNOW data to ensure data integrity. */
    uint8_t magic;                        /**< Magic number to determine the target device for unicast ESPNOW data. */
    uint8_t payload[8];                   /**< Actual payload of the ESPNOW data frame. */
} __attribute__((packed)) example_espnow_data_t;



/**
 * @brief The structure containing all parameters for sending and receiving ESPNOW data. Must be the same on all devices
 *        Element element can being modified
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

/*--------- PARAMETER CONFIGURATION ---------*/
/* WiFi should start before using ESPNOW */

/**
 * @brief Initilization of the wifi.The wifi must be started before use ESPNOW 
 * 
 */
void example_wifi_init(void);

/**
 * @brief  ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task.
 * 
 * @param mac_addr : MAC adress destination 
 * @param status 
 */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);


/**
 * @brief ESPNOW sending or receiving callback function is called in WiFi task.
 *        Users should not do lengthy operations from this task. 
 *        Callback of a reception via ESP now.
 *        When data is received the callback post data to a queue for treatment
 * 
 * @param recv_info : information about the reception contain :the source MAC and destination MAC. Defined in espnow.h. If the MAC is unknow it's added on the peer list
 * @param data : data received in the form of espnow_data_t if the sender got the same program
 * @param len : len of the data used for assertion 
 */
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);



/**
 * @brief Parse received ESPNOW data. The buffer is splitted in different part from the espnow_data stuct define in espnow.h
 * 
 * @param data : the data buffer we received from the ESPNOW communication
 * @param data_len : The total size of the data buffer for processing to verification
 * @param state : The state of the sender if he his in broadcast 0x00 and if in unicast 0x01
 * @param seq : TBD 
 * @param magic : a magic number implented in the example code 
 * @param payload : The "real message" sended by the sender
 * @param payload_len : The lenght of the message
 * @return int 
 */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint8_t *magic, uint8_t *payload, uint8_t payload_len);



/* Prepare ESPNOW data to be sent. */
/**
 * @brief This function prepare the data for being send. 
 * If you want to use it the data must be sent by espnow_send. data copied with memcpy for limit border effect.
 * 
 * @param send_param : Sending parameter of ESP_now defined. The value is passed with *send_parma 
 * @param mes2send : The message to send on the format u_int8_t *. passed throught a pointer because we know the size
 */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param,u_int8_t *mes2send);




/**
 * @brief This function concatenate the prepararation via esp_dataprepare and the sending with err check
 * 
 * @param send_param : Sending parameter of ESP_now defined. The value is passed with *send_parma
 * @param message : The message to send on the format u_int8_t *. passed throught a pointer because we know the 
 * @param desMAC : The MAC destinations for sending to multiple ESP23. We took it back from the 1st BROADCAST recv_cb
 * @return int : Return 0 if the send is OK and -1 if the sending is NOK
 */
int espnow_datasending(example_espnow_send_param_t *send_param, uint8_t *message, const uint8_t *desMAC);



/**
 * @brief : Task for the espnow sending andreceive callback. Specify a MAC ADRESS FOR BROADCAST DATA before calling this function if you want to send broadcast data
* The second call will took the adress registred before for sending unidata
 * 
 * @param pvParameter the sending parameter for ESPNOW
 */
static void example_espnow_task(void *pvParameter);

/**
 * @brief Task used for schedulling the sending throught ESPNOW. The task received data throught a queue  
 *        and sending with a specific delay between each sending. Currently used only for the sensor task.
 *        
 * 
 * @param pvParameter the sending parameter for ESPNOW 
 */
void espnow_sending_task(void *pvParameter);

//add *pvParameter
/**
 * @brief initialisation of ESPNOW. We initialize inside the task related to ESPNOW
 * 
 * @param send_param the sending parameter for ESPNOW
 * @return esp_err_t 
 */
esp_err_t example_espnow_init(void *send_param);

/* Unitialize the espnow and free all the data*/

/**
 * @brief Deinitialization of the sending parameter. Delete the task and free the allocation of send_param
 * 
 * @param send_param : the sending parameter for ESPNOW
 */
void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* Create the sendingParameter for being used for the other function*/
/**
 * @brief Initiator function for allow other function to access to the sending parameter and send data to other devices 
 * 
 * @return example_espnow_send_param_t* use the return for sending data
 */
example_espnow_send_param_t *SendingParamCreator(void);

