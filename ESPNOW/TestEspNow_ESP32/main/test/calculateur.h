#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/**
 * @brief Liste des alertes
 *  
 */
#define BSW_G 1
#define BSW_D 2
#define DNPW_G 3
#define DNPW_D 4
#define EEBL 5
#define FCW_LOW 6
#define FCW_ADV 7
#define FCW_CRT 8
#define DEPACEMENT_G 9
#define DEPACEMENT_D 10
// A implementer
#define ROAD_LIGHT_LOC 11
#define ROAD_LIGHT_DIST 12

/**
 * @brief Initialisation de tout les tâches et Queue 
 * 
 * @return esp_err_t 
 */
#define CALCULATOR_QUEUE_LENGHT 15
#define ALERT_DATA_FORMAT sizeof(uint16_t)
#define CALCULATOR_STACK_SIZE 2


esp_err_t CalculatorTaskQueueInitiator(void);

static void task_calculateur(void *pvParameters);

extern QueueHandle_t queueCapteur_rx;
extern QueueHandle_t queueMoteur_rx;
extern QueueHandle_t queueESPNOW_rx;

// Queue par lesquelles le calculateur transmet les code erreur
extern QueueHandle_t queueLCD_tx;
extern QueueHandle_t queueMoteur_tx;
extern QueueHandle_t queueESPNOW_tx;