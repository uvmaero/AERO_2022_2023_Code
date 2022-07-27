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

// *** includes *** // 
#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <Canbus.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
//#include <ArduinoBLE.h>
#include "pinConfig.h"


// *** defines *** // 
#define SENSOR_POLL_INTERVAL            5
#define CAN_READ_WRITE_INTERVAL         10


// *** global variables *** // 
// initialize sensor struct
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
};
volatile Sensors sensors;

// initialize inputs struct
struct Inputs
{
  uint16_t brakeRegen = 0;
  uint16_t coastRegen = 0;
  bool readyToDrive = false;
};
volatile Inputs inputs;


// *** function declarations *** //
void PollSensorData();
void CANReadWrite();

// *** setup *** //
void setup()
{
  // initialize serial
  Serial.begin(9600);

  // initialize sensors
  pinMode(wheelSpeedFRSensor, INPUT);
  pinMode(wheelSpeedFLSensor, INPUT);
  pinMode(wheelSpeedBRSensor, INPUT);
  pinMode(wheelSpeedBLSensor, INPUT);
  pinMode(wheelHeightFRSensor, INPUT);
  pinMode(wheelHeightFLSensor, INPUT);
  pinMode(wheelHeightBRSensor, INPUT);
  pinMode(wheelHeightBLSensor, INPUT);
  pinMode(steeringWheelPot, INPUT);

  // initialize inputs
  pinMode(brakeRegenPot, INPUT);
  pinMode(coastRegenPot, INPUT);
  pinMode(readyToDriveButton, INPUT);

  // initalize outputs

  // initalize CAN
  if (Canbus.init(CANSPEED_500))    // set bit rate to 500
    Serial.println("CAN INIT");
  else
    Serial.println("FAIlED CAN INIT");

  // initialize bluetooth low energy

  // initialize timer interrupts
  // timer for polling sensors
  Timer1.initialize(50000);     // 0.05 seconds in microseconds
  Timer1.attachInterrupt(PollSensorData);

  // timer for reading and writing to CAN
  Timer3.initialize(100000);    // 0.1 seconds in microseconds
  Timer3.attachInterrupt(CANReadWrite);
}

// *** loop *** // 
void loop() {
  // main loop
  while(1)
  {
    // update wheel dash

    // update aero remote data aquisition system

  }
}



void PollSensorData()
{

}


void CANReadWrite()
{
  // read data off the CAN bus
  tCAN incomingMessage;
  if (mcp2515_check_message())
  {
    if (mcp2515_get_message(&incomingMessage))
    {
      // filter messages
      switch (incomingMessage.id)
      {
        // example address 1
        case 0x001:
        // receive data
        break;

        // example address 2
        case 0x002:
        // receive data
        break;

        // if the message is not from an address of interest
        default:
        // do nothing!
        break;
      }
    }
  }


  // write to CAN bus
  tCAN outgoingMessage;

  // build the message
  outgoingMessage.id = 0x001;           // formatted in hex
  outgoingMessage.header.rtr = 0;
  outgoingMessage.header.length = 8;    // formatted in decimal
  outgoingMessage.data[0] = 0x00;
  outgoingMessage.data[1] = 0x01;
  outgoingMessage.data[2] = 0x02;
  outgoingMessage.data[3] = 0x03;
  outgoingMessage.data[4] = 0x04;
  outgoingMessage.data[5] = 0x05;
  outgoingMessage.data[6] = 0x06;
  outgoingMessage.data[7] = 0x07;

  // send the messagee
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2) | (1<<REQOP1) | (1<<REQOP0), 0);
  mcp2515_send_message(&outgoingMessage);
}