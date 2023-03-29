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


uint8_t wcbAddress[] = {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x80};
uint8_t rcbAddress[] = {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81};
uint8_t deviceAddress[] = {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a};


/*
===========================================================
                    Pin Definitions
===========================================================
*/


// Sensors
#define WHEEL_SPEED_FR_SENSOR               ADC1_CHANNEL_0
#define WHEEL_SPEED_FL_SENSOR               ADC1_CHANNEL_3

#define WHEEL_HEIGHT_FR_SENSOR              ADC1_CHANNEL_6
#define WHEEL_HEIGHT_FL_SENSOR              ADC1_CHANNEL_7

#define STEERING_WHEEL_POT                  ADC1_CHANNEL_4

#define PEDAL_0_PIN                         ADC1_CHANNEL_5
#define PEDAL_1_PIN                         ADC2_CHANNEL_8
#define BRAKE_0_PIN                         ADC2_CHANNEL_0
#define BRAKE_1_PIN                         ADC2_CHANNEL_1

// Inputs
#define RTD_BUTTON_PIN                      17

// CAN
#define CAN_TX_PIN                          23
#define CAN_RX_PIN                          19
#define CAN_ENABLE_PIN                      5

// Outputs
#define RTD_BUTTON_LED_PIN                  21
#define WCB_CONNECTION_LED                  26
#define BUZZER_PIN                          16
#define ARDAN_MISO_PIN                      12
#define ARDAN_MOSI_PIN                      13
#define ARDAN_CS_PIN                        10