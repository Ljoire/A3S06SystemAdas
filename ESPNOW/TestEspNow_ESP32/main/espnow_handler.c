#include "espnow_handler.h"

static const char *TAG = "ESPNOW_HANDLER";

// Création de la file d'attente pour les messages reçus
QueueHandle_t espnow_receive_queue;

// Callback pour la réception des messages
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len > MAX_MESSAGE_LENGTH - 1) {
        ESP_LOGW(TAG, "Message reçu trop long");
        return;
    }

    espnow_message_t msg;
    memcpy(msg.message, data, len);
    msg.message[len] = '\0';  // Assurer la terminaison de la chaîne

    // Envoyer le message à la file d'attente
    if (xQueueSend(espnow_receive_queue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "File d'attente de réception pleine");
    }
}

// Callback pour l'envoi des messages
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Message envoyé avec succès");
    } else {
        ESP_LOGW(TAG, "Échec de l'envoi du message");
    }
}

esp_err_t espnow_init(uint8_t channel) {
    // Initialiser le WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));

    // Initialiser ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // Créer la file d'attente pour les messages reçus
    espnow_receive_queue = xQueueCreate(10, sizeof(espnow_message_t));
    if (espnow_receive_queue == NULL) {
        ESP_LOGE(TAG, "Échec de création de la file d'attente");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t espnow_add_peer(const uint8_t *peer_addr) {
    esp_now_peer_info_t peer = {
        .channel = 0,  // Utilise le même canal que le WiFi
        .ifidx = ESP_IF_WIFI_STA,
        .encrypt = false,
        .peer_addr = {0},
    };
    memcpy(peer.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
    
    return esp_now_add_peer(&peer);
}

esp_err_t espnow_send_message(const uint8_t *peer_addr, const char *message) {
    size_t len = strlen(message);
    if (len > MAX_MESSAGE_LENGTH - 1) {
        ESP_LOGW(TAG, "Message trop long");
        return ESP_ERR_INVALID_ARG;
    }

    return esp_now_send(peer_addr, (const uint8_t *)message, len);
}

void espnow_receive_task(void *pvParameter) {
    espnow_message_t msg;

    while (1) {
        if (xQueueReceive(espnow_receive_queue, &msg, portMAX_DELAY) == pdTRUE) {
            // Traiter le message reçu ici
            ESP_LOGI(TAG, "Message reçu : %s", msg.message);
            // Note : Cette fonction sera modifiée pour afficher sur l'écran LCD
        }
    }
}