/**
 * @file pinConfig.h
 * @author Dominic Gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 0.1
 * @date 2022-07-27
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


uint8_t fcbAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t deviceAddress[] = {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a};


/*
===========================================================
                    Pin Definitions
===========================================================
*/


// Sensors
#define WHEEL_SPEED_FR_SENSOR               ADC1_GPIO32_CHANNEL
#define WHEEL_SPEED_FL_SENSOR               ADC1_GPIO32_CHANNEL

#define WHEEL_HEIGHT_FR_SENSOR              ADC1_GPIO32_CHANNEL
#define WHEEL_HEIGHT_FL_SENSOR              ADC1_GPIO32_CHANNEL

#define STEERING_WHEEL_POT                  ADC1_GPIO32_CHANNEL

#define PEDAL_0_PIN                         ADC1_GPIO32_CHANNEL // use these things to config pins based on gpio #
#define PEDAL_1_PIN                         ADC1_GPIO32_CHANNEL
#define BRAKE_0_PIN                         ADC1_GPIO32_CHANNEL
#define BRAKE_1_PIN                         ADC1_GPIO32_CHANNEL

// Inputs
#define CAN_MESSAGE_INTERRUPT_PIN           15             

// Outputs
