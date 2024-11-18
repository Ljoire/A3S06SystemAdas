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

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)
static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static QueueHandle_t s_example_espnow_queue;

extern QueueHandle_t sensor_data_queue;

typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
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

enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint8_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[8];                   //Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint8_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    u_int8_t len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    bool pingpong;                       //Define the sender and receiver 
    bool error;
} example_espnow_send_param_t;

#endif

/*--------- PARAMETER CONFIGURATION ---------*/
/* WiFi should start before using ESPNOW */
void example_wifi_init(void);

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

// Traitment function of ESPNOW. The functions threat the data, if it was added on the peer list
/*
*  Traitment of the error case and liberate the malloc if data received NOK
*/
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);


/* Parse received ESPNOW data. return the state for know how to deal with it */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint8_t *magic, uint8_t *payload, uint8_t payload_len);



/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param,u_int8_t *mes2send);


/* DESCRIPTION OF THE FUNCTION
*This function concatenate multiple thing
*Prepare the data in regards of what's specified in parameter
*Parameter :    
*               send_param -> the destinations parameter
*               message -> the message to send. If str use (uint_8t*) Str2Send
*               desMAC -> the MAC destinations MAC adress usually from send_cb
*Send the data to the MAC adress specified in parameter
*Return is OK=0 NOK=1*/
int espnow_datasending(example_espnow_send_param_t *send_param, uint8_t *message, const uint8_t *desMAC);

/* 
* Task for ESP_NOW sending and receive function
* Specify a MAC ADRESS FOR BROADCAST DATA before calling this function if you want to send broadcast data
* The second call will took the adress registred before for sending unidata
*/
static void example_espnow_task(void *pvParameter);

void espnow_sending_task(void *pvParameter);

//add *pvParameter
esp_err_t example_espnow_init(void *send_param);

/* Unitialize the espnow and free all the data*/
void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* Create the sendingParameter for being used for the other function*/
example_espnow_send_param_t *SendingParamCreator(void);

