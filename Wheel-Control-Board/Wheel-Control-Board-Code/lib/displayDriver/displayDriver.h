/**
 * @file displayDriver.h
 * @author Dominic Gasperini
 * @brief this is a custom library designed to interface with a display via the TFT_eSPI library 
 * @version 0.1
 * @date 2023-02-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/

#include <TFT_eSPI.h>                 // the graphics library (includes sprite functions)


/*
===============================================================================================
                                    Type Definitions 
===============================================================================================
*/

typedef enum {
    INIT_DISPLAY = 1,
    INIT_SENSORS = 2,
    INIT_ESP_NOW = 3
} Display_BootScreenStates_t;


/*
===============================================================================================
                                    Function Definitions 
===============================================================================================
*/

void Display_BootScreen(Display_BootScreenStates_t state);

void Display_MainScreen();

void Display_MechanicalScreen();

void Display_ElectricalScreen();
