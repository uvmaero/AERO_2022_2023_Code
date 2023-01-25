/**
 * @file pinConfig.h
 * @author dominic gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 0.1
 * @date 2022-07-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Sensors
#define WHEEL_SPEED_FR_SENSOR               1
#define WHEEL_SPEED_FL_SENSOR               2

#define WHEEL_HEIGHT_FR_SENSOR              5
#define WHEEL_HEIGHT_FL_SENSOR              6

#define STEERING_WHEEL_POT                  9

#define PEDAL_0_PIN                         12
#define PEDAL_1_PIN                         13
#define BRAKE_0_PIN                         23
#define BRAKE_1_PIN                         22

// Inputs
#define CAN_MESSAGE_INTERRUPT_PIN           15             

// Outputs
#define BUZZER_PIN                          16
#define ARDAN_SS_PIN                        10
#define ARDAN_RST_PIN                       11
#define ARDAN_DIO_PIN                       14
