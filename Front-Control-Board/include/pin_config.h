/**
 * @file pinConfig.h
 * @author dominic gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 1.0
 * @date 2023-03-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */


// includes
#include <esp_pm.h>

/*
===========================================================
                    Power Configuration
===========================================================
*/

esp_pm_config_esp32_t power_configuration {
    .max_freq_mhz = 240,   
    .min_freq_mhz = 240,
    .light_sleep_enable = false,
};


/*
===========================================================
                    Pin Definitions
===========================================================
*/


// Sensors
#define WHEEL_SPEED_FR_SENSOR               36
#define WHEEL_SPEED_FL_SENSOR               39

#define WHEEL_HEIGHT_FR_SENSOR              34
#define WHEEL_HEIGHT_FL_SENSOR              35

#define STEERING_WHEEL_POT                  32

#define PEDAL_0_PIN                         33
#define PEDAL_1_PIN                         25
#define BRAKE_PIN                           4

// Inputs
#define RTD_BUTTON_PIN                      17

// CAN
#define CAN_RX_PIN                          19
#define CAN_TX_PIN                          23
#define CAN_ENABLE_PIN                      5

// Outputs
#define RTD_LED_PIN                         21
#define WCB_CONNECTION_LED                  26

#define BMS_LED_PIN                         1
#define IMD_LED_PIN                         3

#define BUZZER_PIN                          16

#define ARDAN_MISO_PIN                      12
#define ARDAN_MOSI_PIN                      13
#define ARDAN_CS_PIN                        10