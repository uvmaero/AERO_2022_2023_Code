/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief this drives the dash control board on clean speed 5.5
 * @version 0.1
 * @date 2022-07-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// includes
#include <Arduino.h>

// defines

// function declarations

// *** setup *** //
void setup() {
  // put your setup code here, to run once:
}

// *** loop *** // 
void loop() {

  // initialize sensor struct
  struct Sensors
  {
    int wheelSpeedFR;
    int wheelSpeedFL;
    int wheelHeightFR;
    int wheelHeightFL;
  };

  // initialize timer interrupts

  // initialize LCD

  // initialize bluetooth

  // initialize CAN

  // main loop
  while(1)
  {
    // update wheel dash

    // update aero remote data aquisition system

  }
}