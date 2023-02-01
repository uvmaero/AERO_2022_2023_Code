/**
 * @file displayDriver.c
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

#include <displayDriver.h>


/*
===============================================================================================
                                    Function Definitions 
===============================================================================================
*/


/**
 * @brief the screen that is displayed during the boot sequence
 * - shows Bessie
 * - shows booting information based on the state passed to the function
 * 
 * @param state the state of the boot sequence 
 */
void Display_BootScreen(Display_BootScreenStates_t state) {
    // inits


    // state machine
    switch (state) {
        case INIT_DISPLAY:
            /* code */
            break;

        case INIT_SENSORS:
            /* code */
            break;

        case INIT_ESP_NOW:
            /* code */
            break;
        
        default:
            break;
    }

}


/**
 * @brief 
 * 
 */
void Display_MainScreen() {

}


/**
 * @brief 
 * 
 */
void Display_MechanicalScreen() {

}


/**
 * @brief 
 * 
 */
void Display_ElectricalScreen() {

}
