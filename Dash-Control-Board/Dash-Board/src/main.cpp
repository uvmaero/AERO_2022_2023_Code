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
#include <ArduinoBLE.h>
#include <Canbus.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include "pinConfig.h"
#include "uuidConfig.h"


// *** defines *** // 
#define SENSOR_POLL_INTERVAL            50000       // 0.05 seconds in microseconds
#define CAN_READ_WRITE_INTERVAL         100000      // 0.1 seconds in microseconds


// *** global variables *** //

enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
};

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
};
volatile Sensors sensors;

struct Inputs
{
  uint16_t brakeRegen = 0;
  uint16_t coastRegen = 0;
  bool readyToDrive = false;
  DriveModes driveMode = ECO;
};
volatile Inputs inputs;

// Bluetooth Low Energy address
BLEService wheelDashUpdateService(PERIPHERAL_UUID);

// Bluetooth Low Energy Charactaristics, using 16 bit UUIDs
BLEIntCharacteristic wheelSpeedFR_BLE(WHEEL_SPEED_FR_UUID, BLERead | BLENotify);
BLEIntCharacteristic wheelSpeedFL_BLE(WHEEL_SPEED_FL_UUID, BLERead | BLENotify);
BLEIntCharacteristic wheelSpeedBR_BLE(WHEEL_SPEED_BR_UUID, BLERead | BLENotify);
BLEIntCharacteristic wheelSpeedBL_BLE(WHEEL_SPEED_BL_UUID, BLERead | BLENotify);

BLEIntCharacteristic wheelHeightFR_BLE(WHEEL_HEIGHT_FR_UUID, BLERead | BLENotify);
BLEIntCharacteristic wheelHeightFL_BLE(WHEEL_HEIGHT_FL_UUID, BLERead | BLENotify);
BLEIntCharacteristic wheelHeightBR_BLE(WHEEL_HEIGHT_BR_UUID, BLERead | BLENotify);
BLEIntCharacteristic wheelHeightBL_BLE(WHEEL_HEIGHT_BL_UUID, BLERead | BLENotify);

BLEIntCharacteristic coastRegen_BLE(COAST_REGEN_UUID, BLERead | BLEWrite);
BLEIntCharacteristic brakeRegen_BLE(BRAKE_REGEN_UUID, BLERead | BLEWrite);

BLEBoolCharacteristic readyToDrive_BLE(READY_TO_DRIVE_UUID, BLERead | BLEWrite);
BLEIntCharacteristic batteryState_BLE(BATTERY_STATE_UUID, BLERead | BLENotify);
BLEIntCharacteristic driveMode_BLE(DRIVE_MODE_UUID, BLERead | BLEWrite);


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
  if (BLE.begin())
  {
    Serial.println("BLE INIT [ SUCCESS ]");

    // setup
    BLE.setLocalName("DashBoard");
    BLE.setAdvertisedService(wheelDashUpdateService);
    wheelDashUpdateService.addCharacteristic(wheelSpeedFR_BLE);
    wheelDashUpdateService.addCharacteristic(wheelSpeedFL_BLE);
    wheelDashUpdateService.addCharacteristic(wheelSpeedBR_BLE);
    wheelDashUpdateService.addCharacteristic(wheelSpeedBL_BLE);
    wheelDashUpdateService.addCharacteristic(wheelHeightFR_BLE);
    wheelDashUpdateService.addCharacteristic(wheelHeightFL_BLE);
    wheelDashUpdateService.addCharacteristic(wheelHeightBR_BLE);
    wheelDashUpdateService.addCharacteristic(wheelHeightBL_BLE);
    wheelDashUpdateService.addCharacteristic(coastRegen_BLE);
    wheelDashUpdateService.addCharacteristic(brakeRegen_BLE);
    wheelDashUpdateService.addCharacteristic(readyToDrive_BLE);
    wheelDashUpdateService.addCharacteristic(driveMode_BLE);
    wheelDashUpdateService.addCharacteristic(batteryState_BLE);
    SendUpdatedDashData();
    BLE.advertise();
  }
  else 
    Serial.println("BLE INIT [ FAILED ]");

  // initialize timer interrupts
  // timer for polling sensors
  Timer1.initialize(SENSOR_POLL_INTERVAL);
  Timer1.attachInterrupt(PollSensorData);

  // timer for reading and writing to CAN
  Timer3.initialize(CAN_READ_WRITE_INTERVAL);
  Timer3.attachInterrupt(CANReadWrite);

  // initialize aero remote data aquisition network (ARDAN)
  
}

// *** loop *** // 
void loop() {
  // main loop
  while(1)
  {
    // update wheel dash
    BLEDevice central = BLE.central();
    if (central && central.connected())
    {
      SendUpdatedDashData();
      ReceiveDashData();
    }

    // update ARDAN

  }
}

/**
 * @brief Interrupt Handler for Timer 1
 * This ISR is for reading sensor data from the car 
 * Is called every 0.05 seconds
 */
void PollSensorData()
{
  sensors.wheelSpeedFR = analogRead(WHEEL_SPEED_FR_SENSOR);
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
  wheelSpeedFR_BLE.writeValue(sensors.wheelSpeedFR);
  wheelSpeedFL_BLE.writeValue(sensors.wheelSpeedFL);
  wheelSpeedBR_BLE.writeValue(sensors.wheelSpeedBR);
  wheelSpeedBL_BLE.writeValue(sensors.wheelSpeedBL);
  wheelHeightFR_BLE.writeValue(sensors.wheelHeightFR);
  wheelHeightFL_BLE.writeValue(sensors.wheelHeightFL);
  wheelHeightBR_BLE.writeValue(sensors.wheelHeightBR);
  wheelHeightBL_BLE.writeValue(sensors.wheelHeightBL);
  coastRegen_BLE.writeValue(inputs.coastRegen);
  brakeRegen_BLE.writeValue(inputs.brakeRegen);
  readyToDrive_BLE.writeValue(inputs.readyToDrive);
  driveMode_BLE.writeValue(inputs.driveMode);
  batteryState_BLE.writeValue(sensors.batteryState);
}

/**
 * @brief 
 * 
 */
void ReceiveDashData()
{
  // check for a connection
  if (peripheral.connected())
  {
    // update values
    inputs.coastRegen = coastRegen_BLE.value();
    inputs.brakeRegen = brakeRegen_BLE.value();
    inputs.readyToDrive = readyToDrive_BLE.value();
    inputs.driveMode = (DriveModes)driveMode_BLE.value();
  }
}