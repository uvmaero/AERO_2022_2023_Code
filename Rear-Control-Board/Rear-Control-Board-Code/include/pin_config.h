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
#include "debugger.h"
#include <soc/adc_channel.h>


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
                    ESP-NOW Definitions
===========================================================
*/


uint8_t fcbAddress[] = {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81};
uint8_t deviceAddress[] = {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a};


/*
===========================================================
                    Pin Definitions
===========================================================
*/


// Sensors
#define WHEEL_SPEED_BR_SENSOR               ADC1_CHANNEL_0
#define WHEEL_SPEED_BL_SENSOR               ADC1_CHANNEL_0

#define WHEEL_HEIGHT_BR_SENSOR              ADC1_CHANNEL_0
#define WHEEL_HEIGHT_BL_SENSOR              ADC1_CHANNEL_0

#define IMD_FAULT_PIN                       32
#define BMS_FAULT_PIN                       33


// Inputs
#define RAD_TEMP_IN_PIN                     ADC1_CHANNEL_0
#define RAD_TEMP_OUT_PIN                    ADC1_CHANNEL_0

#define CAN_CS_PIN                          18
#define CAN_MESSAGE_INTERRUPT_PIN           15   
#define CAN_TX_PIN                          23
#define CAN_RX_PIN                          19


// Outputs
#define FAN_ENABLE_PIN                      25
#define PUMP_ENABLE_PIN                     2
#define BRAKE_LIGHT_PIN                     26

#define SD_CMD_PIN                          14
#define SD_CLK_PIN                          12
#define SD_D0_PIN                           27
#define SD_D1_PIN                           28
#define SD_D2_PIN                           30
#define SD_D3_PIN                           29
#define SD_DETECT_PIN                  39
#define SD_MOUNT_POINT                      "/sdcard"