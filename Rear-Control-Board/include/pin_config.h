/**
 * @file pin_config.h
 * @author Dominic Gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 1.0
 * @date 2023-02-21
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
#define WHEEL_SPEED_BR_SENSOR               36
#define WHEEL_SPEED_BL_SENSOR               39

#define WHEEL_HEIGHT_BR_SENSOR              34
#define WHEEL_HEIGHT_BL_SENSOR              33


// Inputs
#define RAD_TEMP_IN_PIN                     4
#define RAD_TEMP_OUT_PIN                    0

#define IMD_FAULT_PIN                       32
#define BMS_FAULT_PIN                       35

// CAN
#define CAN_RX_PIN                          23
#define CAN_TX_PIN                          19


// Outputs
#define FAN_ENABLE_PIN                      26
#define PUMP_ENABLE_PIN                     2
#define BRAKE_LIGHT_PIN                     25

#define SD_CMD_PIN                          14
#define SD_CLK_PIN                          12
#define SD_D0_PIN                           27
#define SD_D1_PIN                           28
#define SD_D2_PIN                           30
#define SD_D3_PIN                           29
#define SD_DETECT_PIN                       39