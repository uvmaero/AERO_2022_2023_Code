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


// LCD Pins
#define LCD_MISO_PIN                        ADC1_GPIO32_CHANNEL // use these things to config pins based on gpio #
#define LCD_MOSI_PIN                        ADC1_GPIO32_CHANNEL
#define LCD_SCK_PIN                         ADC1_GPIO32_CHANNEL
#define LCD_CHIP_SELECT_PIN                 ADC1_GPIO32_CHANNEL

// Inputs
#define COAST_REGEN_KNOB                    ADC1_CHANNEL_0_GPIO_NUM
#define BRAKE_REGEN_KNOB                    ADC1_CHANNEL_3_GPIO_NUM

#define LCD_MODE_SELECT_BUTTON              34
#define DRIVE_MODE_SELECT_BUTTON            33

// Outputs
#define FCB_CONNECTION_STATUS_LED           ADC1_GPIO32_CHANNEL
