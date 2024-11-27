#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Définition de la structure du message
#define MAX_MESSAGE_LENGTH 32

typedef struct {
    char message[MAX_MESSAGE_LENGTH];
} espnow_message_t;

// File d'attente pour les messages reçus
extern QueueHandle_t espnow_receive_queue;

// Initialisation du WiFi et ESP-NOW
esp_err_t espnow_init(uint8_t channel);

// Ajout d'un pair ESP-NOW
esp_err_t espnow_add_peer(const uint8_t *peer_addr);

// Envoi d'un message via ESP-NOW
esp_err_t espnow_send_message(const uint8_t *peer_addr, const char *message);

// Gestionnaire des messages reçus
void espnow_receive_task(void *pvParameter);

#endif // ESPNOW_HANDLER_H