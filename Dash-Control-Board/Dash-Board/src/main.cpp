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
#include <LoRa_E32.h>
#include "pinConfig.h"


// *** defines *** // 
#define SENSOR_POLL_INTERVAL            50000       // 0.05 seconds in microseconds
#define CAN_READ_WRITE_INTERVAL         100000      // 0.1 seconds in microseconds


// *** global variables *** //

// Bluetooth Low Energy Connection

// aero remote data aquisition network (ARDAN)
LoRa_E32 ardan(ARDAN_RX_PIN, ARDAN_TX_PIN);

// Drive Mode enumeration
enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
};

// Car Data Struct
struct CarData
{
  // for sensor data
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

    uint16_t batteryState = 0;
  } sensors;

  // for input data
  struct Inputs
  {
    uint16_t brakeRegen = 0;
    uint16_t coastRegen = 0;
    bool readyToDrive = false;
    DriveModes driveMode = ECO;
  } inputs;
};
CarData carData;


// *** function declarations *** //
void PollSensorData();
void CANReadWrite();
void UpdateARDAN();
void SendUpdatedDashData();
void ReceiveDashData();

// *** setup *** //
void setup()
{
  // initialize serial
  Serial.begin(9600);

  // initialize sensors
  pinMode(WHEEL_SPEED_FR_SENSOR, INPUT);
  pinMode(WHEEL_SPEED_FL_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_FR_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_FL_SENSOR, INPUT);
  pinMode(STEERING_WHEEL_POT, INPUT);

  // initalize outputs

  // initalize CAN
  if (Canbus.init(CANSPEED_500))    // set bit rate to 500
    Serial.println("CAN INIT [ SUCCESS ]");
  else
    Serial.println("CAN INIT [ FAILED ]");

  // initialize bluetooth low energy
  

  // initialize timer interrupts
  // timer for polling sensors
  Timer1.initialize(SENSOR_POLL_INTERVAL);
  Timer1.attachInterrupt(PollSensorData);

  // timer for reading and writing to CAN
  Timer3.initialize(CAN_READ_WRITE_INTERVAL);
  Timer3.attachInterrupt(CANReadWrite);

  // initialize ARDAN
  ardan.begin();
  ResponseStructContainer ardanConfigContainer = ardan.getConfiguration();
  Configuration ardanConfig = *(Configuration*) ardanConfigContainer.data;
  ResponseStructContainer ardanModuleContainer = ardan.getModuleInformation();
  ModuleInformation ardanModuleInfo = *(ModuleInformation*) ardanModuleContainer.data;
  ResponseStatus responseStatus = ardan.sendMessage("ARDAN ONLINE");
  Serial.print("ARDAN INIT STATUS: ");
  Serial.println(responseStatus.getResponseDescription());
}

// *** loop *** // 
void loop() {
  // main loop
  while(1)
  {
    // update wheel dash
    if (true)                    // MAKE THIS CHECK FOR ACTIVE BLE CONNECTION
    {
      SendUpdatedDashData();
      ReceiveDashData();
    }

    // update ARDAN
    if (ardan.available() > 1)
    {
      // send message
      ResponseStatus response = ardan.sendFixedMessage(0, 3, 4, &carData, sizeof(carData));
      Serial.print("ARDAN MESSAGE SENT STATUS: ");
      Serial1.println(response.getResponseDescription());
    }
  }
}

/**
 * @brief Interrupt Handler for Timer 1
 * This ISR is for reading sensor data from the car 
 * Is called every 0.05 seconds
 */
void PollSensorData()
{
  // update wheel speed values
  carData.sensors.wheelSpeedFR = analogRead(WHEEL_SPEED_FR_SENSOR);
  carData.sensors.wheelSpeedFL = analogRead(WHEEL_HEIGHT_FL_SENSOR);

  // update wheel ride height value s
  carData.sensors.wheelHeightFR = analogRead(WHEEL_HEIGHT_FR_SENSOR);
  carData.sensors.wheelHeightFL = analogRead(WHEEL_HEIGHT_FL_SENSOR);

  // update steering wheel position
  carData.sensors.steeringWheelAngle = analogRead(STEERING_WHEEL_POT);
}

/**
 * @brief Interrupt Handler for Timer 2
 * This ISR is for reading and writing to the CAN bus
 * Is called every 0.1 seconds
 * 
 */
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

/**
 * @brief 
 * 
 */
void UpdateARDAN()
{
}

/**
 * @brief 
 * 
 */
void SendUpdatedDashData()
{
  // send data
}

/**
 * @brief 
 * 
 */
void ReceiveDashData()
{
  // update values
}