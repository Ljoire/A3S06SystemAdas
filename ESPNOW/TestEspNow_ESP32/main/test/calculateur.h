#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "esp_log.h"

    /***** @brief Liste des entrée ****/
    /** @brief ESPNOW **/
//EEBL
#define ESPNOW_EEBL_MID 2
#define ESPNOW_EEBL_HIGH 4
#define ESPNOW_EEBL_CRIT 6

//BSW
#define ESPNOW_BSW_G 8
#define ESPNOW_BSW_D 10

//DNPWN
#define ESPNOW_DNPW_G 12
#define ESPNOW_DNPW_D 14

#define ESPNOW_FCW_HIGH 17
#define ESPNOW_FCW_CRIT 19

//Plein phare
#define ESPNOW_PLEIN_PHARE 21

    /** @brief capteur **/
#define CAPTEUR_NO_ERROR 0
//EEBL
#define CAPTEUR_EEBL_MID 1
#define CAPTEUR_EEBL_HIGH 3
#define CAPTEUR_EEBL_CRIT 5
//BSW
#define CAPTEUR_BSW_GAUCHE 7
#define CAPTEUR_BSW_DROITE 9
//DNPW
#define CAPTEUR_DNPW_G 11
#define CAPTEUR_DNPW_D 13
//FCW
#define CAPTEUR_FCW_MID 15
#define CAPTEUR_FCW_HIGH 16
#define CAPTEUR_FCW_CRIT 18

    /** @brief LUMINOSITE */
#define LUMINOSITE_PLEIN_PHARE 20 

//Trame du capteur [Code Alerte, Distance1, Distance2,Distance3,Distance4,Distance5,Distance6]
#define SENSOR_FRAME_LENGH 6 
//en comptant le 0


    /***** @brief Liste des sortie ****/
    /** @brief Sortie Moteur **/
#define MOTEUR_AVANT_LENT 2
#define MOTEUR_STOP 3
#define SERVO_CENTRE 4
    /** @brief buzzer*/
#define SONNERIE_STOP 0
#define SONNERIE_LOW 1
#define SONNERIE_INTERMEDIAIRE 2
#define SONNERIE_FORT 3
/** @brief LCD */
#define DISTANCE_A_RECEVOIR 0x0000
#define RESET_AFFICHAGE 0x1F
#define DELAY_FOR_SEND_RESET 30000
/**
 * @brief Initialisation de tout les tâches et Queue 
 * 
 * @return esp_err_t 
 */
#define CALCULATOR_QUEUE_LENGHT 15
#define ALERT_DATA_FORMAT uint8_t
#define DISTANCE_DATA_FORMAT uint16_t
#define CALCULATOR_STACK_SIZE 2048


esp_err_t CalculatorTaskQueueInitiator(void);

void task_calculateur(void *pvParameters);

extern QueueHandle_t queueCapteur_rx;
extern QueueHandle_t queueLuminosite_rx;
extern QueueHandle_t queueESPNOW_rx;
extern QueueHandle_t queueluminosité_rx;

// Queue par lesquelles le calculateur transmet les code erreur
extern QueueHandle_t queueLCD_tx;
extern QueueHandle_t queueMoteur_tx;
extern QueueHandle_t queueESPNOW_tx;
extern QueueHandle_t queueluminosité_tx;

/**
 * @brief Fonction de traitement des alertes reçu par l'ESPNOW
 * Se réfère aux informations présent dans la table des alertes 
 * @return true données présente et traitée
 * @return false Donées non présente
 */
bool ProcessEspNowData(void);

/**
 * @brief Traitement des alertes liés aux capteurs 
 * 
 * @param capteur_data 
 * @return true alerte traité
 * @return false pas d'alerte
 */
bool ProcessCapteurData();