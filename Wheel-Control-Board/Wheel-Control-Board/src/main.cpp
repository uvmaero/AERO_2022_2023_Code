/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief this drives the wheel control board on clean speed 5.5
 * @version 0.1
 * @date 2022-07-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// *** includes *** // 
#include <Arduino.h>
#include "pinConfig.h"


// *** defines *** // 


// *** global variables *** //


// ESP-Now Connection


// Drive Mode enumeration
enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
};

// Car Data Struct
struct DashData
{

  struct DrivingData
  {
    bool readyToDrive = false;
    bool enableInverter = false;

    bool imdFault = false;
    bool bmsFault = false;

    uint16_t commandedTorque = 0;
    bool driveDirection = true;             // true = forward | false = reverse
    DriveModes driveMode = ECO;
  } drivingData;
  
  struct BatteryStatus
  {
    uint16_t batteryChargeState = 0;
    int16_t busVoltage = 0;
    int16_t rinehartVoltage = 0;
    float pack1Temp = 0;
    float pack2Temp = 0;
  } batteryStatus;

  struct Sensors
  {
    uint16_t wheelSpeedFR = 0;
    uint16_t wheelSpeedFL = 0;
    uint16_t wheelSpeedBR = 0;
    uint16_t wheelSpeedBL = 0;

    uint16_t wheelHeightFR = 0;
    uint16_t wheelHeightFL = 0;
    uint16_t wheelHeightBR = 0;
    uint16_t wheelHeightBL = 0;

    uint16_t steeringWheelAngle = 0;
  } sensors;

  struct Inputs
  {
    uint16_t pedal0 = 0;
    uint16_t pedal1 = 0;
    uint16_t brake0 = 0;
    uint16_t brake1 = 0;
    uint16_t brakeRegen = 0;
    uint16_t coastRegen = 0;
    float vicoreTemp = 0;
  } inputs;

  struct Outputs
  {
    bool buzzerState = false;
    uint8_t buzzerCounter = 0;
    bool brakeLight = false;
  } outputs;
  
};
CarData dashData;


// *** function declarations *** //


// *** setup *** //
void setup()
{
  // initialize serial
  Serial.begin(9600);

  // initialize sensors


  // initalize outputs


  // initialize LCD


  // initialize ESP-Now
  

  // initialize timer interrupts
  
}

// *** loop *** // 
void loop() {
  // main loop
  while(1)
  {
    // update wheel dash LCD


    // update FCB with dash data

  }
}
