/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
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
#include "espnow_example.h"

/*********************** CUSTOM DEFINE*/

#define FRAMELEN 18
#define MAX_PAYLOAD_SIZE 8
#define CUSTOM_SEND_COUNT 2

#define FRAMECOUNTER 5

#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;

    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}
/* Traitment function of ESPNOW. The functions threat the data, if it was added on the peer list/
*  Traitment of the error case and liberate the malloc if data received NOK
*/
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. return the state for know how to deal with it */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, uint8_t *magic, uint8_t *payload, uint8_t payload_len)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }
    
    
    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    /*******DEBUG
    payload = buf->payload;//assign to it
    for (int i = 0; i < sizeof(buf->payload); i++) {
        printf("%02X ", buf->payload[i]);
    }
    printf(" is the data parsed inside the parser \n");
    // Copy the payload with boundary checking
    //printf("%d len", payload_len);*/

    size_t copy_len = (payload_len < sizeof(buf->payload)) ? payload_len : sizeof(buf->payload);
    memcpy(payload, buf->payload, copy_len);

    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);



    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param,u_int8_t *mes2send)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    //testing the type of MAC adress inside. type take EXAMPLE_ESPNOW_DATA_BROADCAST if broadcast 
    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    
    /* Fill all remaining bytes after the data with random values */
    //esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    /* Copie du payload dans la structure */
    printf("the message to send is ");
    for (uint8_t i = 0; i < sizeof(buf->payload); i++)
    {
        printf("%c",mes2send[i]);
    }
    printf("\n\n");
    memcpy(buf->payload, mes2send, sizeof(buf->payload));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);

}

u_int8_t message[8] = "toto";
u_int8_t *Pmessage = message;

/* DESCRIPTION OF THE FUNCTION
*This function concatenate multiple thing
*Prepare the data in regards of what's specified in parameter
*Parameter :    
*               send_param -> the destinations parameter
*               message -> the message to send. If str use (uint_8t*) Str2Send
*               desMAC -> the MAC destinations MAC adress usually from send_cb
*Send the data to the MAC adress specified in parameter
*Return is OK=0 NOK=1*/
int espnow_datasending(example_espnow_send_param_t *send_param, uint8_t *message, const uint8_t *desMAC){
    /* Delay a while before sending the next data. */
    if (send_param->delay > 0) {
        vTaskDelay(send_param->delay / portTICK_PERIOD_MS);
    }

    /* Copy the destination MAC address into the send parameter. */
    memcpy(send_param->dest_mac, desMAC, ESP_NOW_ETH_ALEN);
    u_int8_t messageboundchecked[MAX_PAYLOAD_SIZE];
    memcpy(messageboundchecked,message,MAX_PAYLOAD_SIZE);
    ESP_LOGI(TAG, "Sending data to "MACSTR" the message is %s: ", MAC2STR(desMAC),messageboundchecked);
    /* Prepare the ESPNOW data using the provided message. */
    example_espnow_data_prepare(send_param, messageboundchecked);

    /* Send the data after preparation. */
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);  // Optionally terminate the task.
        return -1;
    }
    return 0;
}
/* 
* Task for ESP_NOW sending and receive function
* Specify a MAC ADRESS FOR BROADCAST DATA before calling this function if you want to send broadcast data
* The second call will took the adress registred before for sending unidata
*/
static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint8_t recv_magic = 0;
    //the 4 last byte pointed on some memory have to check the pointers and mem allocations
    uint8_t recv_payload[MAX_PAYLOAD_SIZE];//data of the frames became OK when put an U16
    
    
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    //launch an event on the Queue
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    u_int8_t Receiver_counter = CUSTOM_SEND_COUNT;
    u_int8_t Frame_counter = FRAMECOUNTER;
    


    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) 
    {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:{
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);
                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);
                if (is_broadcast && (send_param->broadcast == false)) 
                {
                    break;
                }

                // if in UNICAST we decount 
                if (!is_broadcast) 
                {
                    if(send_param->pingpong == false){
                        printf("not my turn to send \n");
                        break;
                    }
                    printf("Sending counter = %u\n",send_param->count);
                    //if transmit is over we put pingpong at false and uncount the frame counter
                    if (send_param->count == 0) 
                    {
                        ESP_LOGI(TAG, "send data for handshaking to "MACSTR"", MAC2STR(send_cb->mac_addr));

                        espnow_datasending(send_param,(uint8_t*) "SdLead,",send_cb->mac_addr);  
                        
                        send_param->pingpong = false;
                        Frame_counter--;
                        Receiver_counter = CUSTOM_SEND_COUNT;//reset the value to CUSTOM_SEND_COUNT for the receiver mode
                        printf("pass from sender to receiver Frame_counter = %u Recive counter= %u\n",Frame_counter,Receiver_counter);

                        if(Frame_counter == 0){
                            ESP_LOGI(TAG, "Frame sent turn off the frame");
                            espnow_datasending(send_param,(uint8_t *)"ENDING",send_cb->mac_addr);
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                    }
                        break;
                    }
                    send_param->count--;
                    espnow_datasending(send_param,(u_int8_t *)"UNIDATA",send_cb->mac_addr);
                    break;
                }
                
                espnow_datasending(send_param,(u_int8_t *)"BRODATA",send_cb->mac_addr);
                break;          
            }

            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic,recv_payload,sizeof(recv_payload));
                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                        if (recv_state == 1){
                            ESP_LOGI(TAG,"Both ESP have received broadcast data wait for being call back on the main");
                            vTaskDelete(1);
                        }
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1 && IS_BROADCAST_ADDR(send_param->dest_mac) != 0) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                            printf("My magic number is : %d and the received is :%d",send_param->magic,recv_magic);
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));
                            printf("we passed here \n");
                            send_param->pingpong = true;
                            send_param->unicast = true;
                            send_param->broadcast = false;
                    	    /* Start sending unicast ESPNOW data. */
                            espnow_datasending(send_param,(u_int8_t *) "StrUniM",recv_cb->mac_addr);    
                            break;
                        }            
                        else{
                            send_param->broadcast = false;
                            send_param->unicast = true;
                            send_param->pingpong = false;
                            //
                            break;
                        }
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);
                    uint8_t parserMessage [sizeof(recv_payload)];
                    for (int i = 0; i < sizeof(recv_payload); i++) {
                        printf("%c ", recv_payload[i]);
                        parserMessage[i] = recv_payload[i];
                    }

                    printf(" is the data parsed \n");
                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                    
                    uint8_t table[] = "SdLead ";
                    //if we receive the acquitement message we pass to sender
                    if(parserMessage[0] == table[0]){
                        printf("Receive all the data\n");
                        send_param->pingpong = true;
                        //testing if the frame is finished
                        Frame_counter--;
                        if(Frame_counter == 0){
                            ESP_LOGI(TAG, "receive done");
                            espnow_datasending(send_param,(uint8_t *)"ENDING",recv_cb->mac_addr);
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                            }
                        else{//pass on sender
                            //we add +1 du the configuration in the case "SEND_CB"
                            send_param->count = CUSTOM_SEND_COUNT + 1;// we refill the counter for the send mode
                            printf("pass from receiver to sender Frame_counter = %u Count= %u\n",Frame_counter,send_param->count);
                            espnow_datasending(send_param,(u_int8_t *) "TakLead",recv_cb->mac_addr);

                        }
                    }
                    
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
    }
    printf("out of the while \n");
}
static esp_err_t example_espnow_init(void)//add *pvParameter
{
    example_espnow_send_param_t *send_param; //= (example_espnow_send_param_t *)pvParameter;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );
    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
//will be commented
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CUSTOM_SEND_COUNT;//CONFIG_ESPNOW_SEND_COUNT;//down to 25
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = FRAMELEN;//CONFIG_ESPNOW_SEND_LEN; modif a 18
    send_param->pingpong = true;
    send_param->buffer = malloc(FRAMELEN);
// Will be commented
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }


    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param,(uint8_t*) "DBRODAT");
    // put from 2048 to 3062 cancel the stack overflow no overflow with 2064
    //This task will took the brodcast data
    xTaskCreate(example_espnow_task, "example_espnow_task", 2064, send_param, 4, NULL);
    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    printf("wifi initialized");
    example_espnow_init();
    while(1){
        if()
    }
    printf("ESP now init");
    
}
