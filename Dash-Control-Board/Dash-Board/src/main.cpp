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
#define BRAKE_LIGHT_THRESHOLD           10
#define PEDAL_DEADBAND                  10
#define PEDAL_MIN                       128
#define PEDAL_MAX                       600
#define TORQUE_DEADBAND                 5
#define MAX_TORQUE                      220         // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED (230)

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
CarData carData;


// *** function declarations *** //
void PollSensorData();
void CANReadWrite();
void SendUpdatedDashData();
void ReceiveDashData();
void GetCommandedTorque();
long MapValue(long x, long in_min, long in_max, long out_min, long out_max);

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
  // get pedal positions
  carData.inputs.pedal0 = analogRead(PEDAL_0_PIN);
  carData.inputs.pedal1 = analogRead(PEDAL_1_PIN);
  GetCommandedTorque();

  // update wheel speed values
  carData.sensors.wheelSpeedFR = analogRead(WHEEL_SPEED_FR_SENSOR);
  carData.sensors.wheelSpeedFL = analogRead(WHEEL_HEIGHT_FL_SENSOR);

  // update wheel ride height value s
  carData.sensors.wheelHeightFR = analogRead(WHEEL_HEIGHT_FR_SENSOR);
  carData.sensors.wheelHeightFL = analogRead(WHEEL_HEIGHT_FL_SENSOR);

  // update steering wheel position
  carData.sensors.steeringWheelAngle = analogRead(STEERING_WHEEL_POT);

  // buzzer logic
  if (carData.outputs.buzzerState == 1)
  {
    carData.outputs.buzzerCounter++;
    if (carData.outputs.buzzerCounter >= 40)           // buzzerCounter is being updated on a 5Hz interval, so after 40 cycles, 2 seconds have passed
    {
      carData.outputs.buzzerState = 0;
      carData.outputs.buzzerCounter = 0;
      carData.drivingData.enableInverter = true;       // enable the inverter so that we can tell rinehart to turn inverter on
    }
  }

  // brake light logic 
  int brakeAverage = (carData.inputs.brake0 + carData.inputs.brake1) / 2;
  if (brakeAverage >= BRAKE_LIGHT_THRESHOLD)
    carData.outputs.brakeLight = true;     // turn it on 

  else
    carData.outputs.brakeLight = false;     // turn it off
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

// function to re-map the pedal value to a torque value based on the drive mode
void GetCommandedTorque()
{
  // get the pedal average
  int pedalAverage = (carData.inputs.pedal0 + carData.inputs.pedal0) / 2;

  // drive mode logic
  switch (carData.drivingData.driveMode)
  {
    case SLOW:  // runs at 50% power
      carData.drivingData.commandedTorque = MapValue(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, MAX_TORQUE * 0.50);
    break;

    case ECO:   // runs at 75% power
      carData.drivingData.commandedTorque = MapValue(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, MAX_TORQUE * 0.75);
    break;

    case FAST:  // runs at 100% power
      carData.drivingData.commandedTorque = MapValue(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, MAX_TORQUE);
    break;
    
    // error state, set the mode to ECO
    default:
      // set the state to ECO for next time
      carData.drivingData.driveMode = ECO;

      // we don't want to send a torque if we are in an undefined state
      carData.drivingData.commandedTorque = 0;
    break;
  }

  // for throttle safety, we will have a deadband
  if (carData.drivingData.commandedTorque <= TORQUE_DEADBAND)   // if less than 5% power is requested, just call it 0
  {
    carData.drivingData.commandedTorque = 0;
  }
}


/**
 * @brief 
 * 
 * @param x             input value to be re-mapped
 * @param in_min        input value min of range
 * @param in_max        input value max of range
 * @param out_min       output value min of range
 * @param out_max       output value max of range
 * @return long         the remapped value
 */
long MapValue(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}