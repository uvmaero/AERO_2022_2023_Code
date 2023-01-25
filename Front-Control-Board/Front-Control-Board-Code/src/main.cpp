/**
 * @file main.cpp
 * @author Dominic Gasperini - UVM '23
 * @brief this drives the front control board on clean speed 5.5
 * @version 0.9
 * @date 2022-08-04
 */

// *** includes *** // 
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <mcp_can.h>
#include <LoRa.h>
#include "pinConfig.h"


// *** defines *** // 
#define TIMER_INTERRUPT_PRESCALER       80          // this is based off to the clock speed (assuming 80 MHz), gets us to microseconds
#define SENSOR_POLL_INTERVAL            50000       // 0.05 seconds in microseconds
#define CAN_WRITE_INTERVAL              100000      // 0.1 seconds in microseconds
#define WCB_UPDATE_INTERVAL             150000      // 0.15 seconds in microseconds
#define ARDAN_UPDATE_INTERVAL           250000      // 0.25 seconds in microseconds
#define BRAKE_LIGHT_THRESHOLD           10
#define PEDAL_DEADBAND                  10
#define PEDAL_MIN                       128
#define PEDAL_MAX                       600
#define TORQUE_DEADBAND                 5
#define MAX_TORQUE                      220         // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED (230)

// *** global variables *** //

// Debug information
typedef struct Debugger
{
  // debug toggle
  bool debugEnabled = false;
  bool CAN_debugEnabled = false;
  bool WCB_debugEnabled = false;
  bool IO_debugEnabled = false;

  // debug data
  byte CAN_sentStatus;
  byte CAN_outgoingMessage[8];

  esp_err_t WCB_updateResult;
  WCB_Data WCB_updateMessage;

  CarData IO_data;
};
Debugger debugger;

// Drive Mode Enumeration Type
enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
};

// Car Data Struct
typedef struct CarData
{
  struct DrivingData
  {
    bool readyToDrive = false;
    bool enableInverter = false;

    bool imdFault = false;
    bool bmsFault = false;

    uint16_t commandedTorque = 0;
    float currentSpeed = 0.0f;
    bool driveDirection = true;             // true = forward | false = reverse
    DriveModes driveMode = ECO;
  } drivingData;
  
  struct BatteryStatus
  {
    uint16_t batteryChargeState = 0;
    int16_t busVoltage = 0;
    int16_t rinehartVoltage = 0;
    float pack1Temp = 0.0f;
    float pack2Temp = 0.0f;
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

    float vicoreTemp = 0.0f;
    float pumpTempIn = 0.0f;
    float pimpTempOut = 0.0f;
  } inputs;

  struct Outputs
  {
    bool buzzerActive = false;
    uint8_t buzzerCounter = 0;
    bool brakeLight = false;
  } outputs;
  
};
CarData carData;

// ESP-Now Connection
uint8_t wcbAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info wcbInfo;

typedef struct WCB_Data
{
  struct DrivingData
  {
    bool readyToDrive = false;

    bool imdFault = false;
    bool bmsFault = false;

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
  } sensors;

  struct IO
  {
    // inputs
    uint8_t brakeRegen = 0;
    uint8_t coastRegen = 0;

    // outputs
    bool buzzerActive = false;
  } io;
};
WCB_Data wcbData; 

// CAN
MCP_CAN CAN0(10);       // set CS pin to 10

// Hardware ISR Timers
hw_timer_t* timer0 = NULL;
hw_timer_t* timer1 = NULL;
hw_timer_t* timer2 = NULL;
hw_timer_t* timer3 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// *** function declarations *** //
void PollSensorData();
void CANRead();
void CANWrite();
void UpdateARDAN();
void UpdateWCB();
void WCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length);
void GetCommandedTorque();
long MapValue(long x, long in_min, long in_max, long out_min, long out_max);
void PrintDebug();
void PrintCANDebug();
void PrintWCBDebug();
void PrintIODebug();

// *** setup *** //
void setup()
{
  // --- initialize serial --- //
  Serial.begin(9600);

  // --- initialize sensors --- //
  pinMode(WHEEL_SPEED_FR_SENSOR, INPUT);
  pinMode(WHEEL_SPEED_FL_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_FR_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_FL_SENSOR, INPUT);
  pinMode(STEERING_WHEEL_POT, INPUT);

  // --- initalize outputs --- //


  // --- initalize CAN --- //
  pinMode(CAN_MESSAGE_INTERRUPT_PIN, INPUT);
  // create interrupt when the message arrived pin is pulled low
  attachInterrupt(CAN_MESSAGE_INTERRUPT_PIN, CANRead, LOW);
  
  // init CAN
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("CAN INIT [ SUCCESS ]");
  else
    Serial.println("CAN INIT [ FAILED ]");
  
  // set mode to read and write
  CAN0.setMode(MCP_NORMAL);

  // setup mask and filter
  CAN0.init_Mask(0, 0, 0xFFFFFFFF);       // check all ID bits, excludes everything except select IDs
  CAN0.init_Filt(0, 0, 0x100);            // RCB ID 1
  CAN0.init_Filt(1, 0, 0x101);            // RCB ID 2
  CAN0.init_Filt(2, 0, 0x102);            // Rinehart ID

  // --- initialize ESP-NOW ---//
  // turn on wifi access point 
  WiFi.mode(WIFI_STA);

  // init ESP-NOW service
  if (esp_now_init() != ESP_OK)
    Serial.println("ESP-NOW INIT [ SUCCESS ]");
  else
    Serial.println("ESP-NOW INIT [ FAILED ]");
  
  // get peer informtion about WCB
  memcpy(wcbInfo.peer_addr, wcbAddress, sizeof(wcbAddress));
  wcbInfo.channel = 0;
  wcbInfo.encrypt = false;

  // add WCB as a peer
  if (esp_now_add_peer(&wcbInfo) != ESP_OK)
    Serial.println("ESP-NOW CONNECTION [ SUCCESS ]");
  else
    Serial.println("ESP-NOW CONNECTION [ FAILED ]");

  // attach message received callback to the data received function
  esp_now_register_recv_cb(WCBDataReceived);

  // --- initialize timer interrupts --- //
  timer0 = timerBegin(0, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer0, &PollSensorData, true);
  timerAlarmWrite(timer0, SENSOR_POLL_INTERVAL, true);
  timerAlarmEnable(timer0);

  timer1 = timerBegin(1, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer1, &CANWrite, true);
  timerAlarmWrite(timer1, CAN_WRITE_INTERVAL, true);
  timerAlarmEnable(timer1);

  timer2 = timerBegin(2, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer2, &UpdateWCB, true);
  timerAlarmWrite(timer2, WCB_UPDATE_INTERVAL, true);
  timerAlarmEnable(timer2);

  timer3 = timerBegin(3, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer3, &UpdateARDAN, true);
  timerAlarmWrite(timer3, ARDAN_UPDATE_INTERVAL, true);
  timerAlarmEnable(timer3);

  // --- initialize ARDAN ---//
  // set pins for the radio module
  LoRa.setPins(ARDAN_SS_PIN, ARDAN_RST_PIN, ARDAN_DIO_PIN);

  // init LoRa
  if (!LoRa.begin(915E6))         // 915E6 is for use in North America 
    Serial.println("ARDAN INIT [SUCCESSS ]");
  else 
    Serial.println("ARDAN INIT [ FAILED ]");

  // set the sync word so the car and monitoring station can communicate
  LoRa.setSyncWord(0xA1);         // the channel to be transmitting on (range: 0x00 - 0xFF)

  // --- End Setup Section in Serial Monitor --- //
  Serial.print("|--- END SETUP ---|\n\n\n");
}

// *** loop *** // 
void loop()
{
  // everything is on timers so nothing happens here! 

  // debugging
  if (debugger.debugEnabled) {
    PrintDebug();
  }
}


/**
 * @brief Interrupt Handler for Timer 0
 * This ISR is for reading sensor data from the car 
 */
void PollSensorData()
{
  // disable interrupts 
  portENTER_CRITICAL_ISR(&timerMux);

  // turn off wifi for ADC channel 2 to function
  WiFi.mode(WIFI_OFF);

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
  if (carData.outputs.buzzerActive)
  {
    digitalWrite(BUZZER_PIN, carData.outputs.buzzerActive);
    carData.outputs.buzzerCounter++;
    if (carData.outputs.buzzerCounter >= (2 * (100 / (SENSOR_POLL_INTERVAL / 10000))))    // get 2 seconds worth of interrupt counts
    {
      // update buzzer state and turn off the buzzer
      carData.outputs.buzzerActive = false;
      digitalWrite(BUZZER_PIN, carData.outputs.buzzerActive);

      carData.outputs.buzzerCounter = 0;                        // reset buzzer count
      carData.drivingData.enableInverter = true;                // enable the inverter so that we can tell rinehart to turn inverter on
    }
  }

  // brake light logic 
  int brakeAverage = (carData.inputs.brake0 + carData.inputs.brake1) / 2;
  if (brakeAverage >= BRAKE_LIGHT_THRESHOLD)
    carData.outputs.brakeLight = true;      // turn it on 

  else
    carData.outputs.brakeLight = false;     // turn it off

  // turn wifi back on to re-enable esp-now connection to wheel board
  WiFi.mode(WIFI_STA);

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Interrupt Handler for on CAN Message Received
 * This ISR is for reading an incoming message from the CAN bus
 */
void CANRead()
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  unsigned long receivingID = 0;
  byte messageLength = 0;
  byte receivingBuffer[8];

  // read the message
  CAN0.readMsgBuf(&receivingID, &messageLength, receivingBuffer);

  // filter for only the IDs we are interested in
  switch (receivingID)
  {
    // message from RCB: Sensor Data
    case 0x100:
    // do stuff with the data in the message
    break;

    // message from RCB: BMS and electrical data
    case 0x101:
    // do stuff with the data in the message
    break;

    default:
    // do nothing because we didn't get any messages of interest
    return;
    break;
  }

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Interrupt Handler for Timer 1
 * This ISR is for reading and writing to the CAN bus
 */
void CANWrite()
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  byte outgoingMessage[8];

  // build message
  outgoingMessage[0] = 0x00;
  outgoingMessage[1] = 0x01;
  outgoingMessage[2] = 0x02;
  outgoingMessage[3] = 0x03;
  outgoingMessage[4] = 0x04;
  outgoingMessage[5] = 0x05;
  outgoingMessage[6] = 0x06;
  outgoingMessage[7] = 0x07;

  // send the message and get its sent status
  byte sentStatus = CAN0.sendMsgBuf(0x100, 0, sizeof(outgoingMessage), outgoingMessage);    // (sender address, STD CAN frame, size of message, message)

  // if (sentStatus == CAN_OK)
  //   Serial.println("CAN Message Sent Status: Success");
  // else
  //   Serial.println("CAN Message Sent Status: Failed");

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Interrupt Handler for Timer 3
 * update the car data supplied to the FCB
 */
void UpdateWCB()
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // update battery & electrical data
  wcbData.batteryStatus.batteryChargeState = carData.batteryStatus.batteryChargeState;
  wcbData.batteryStatus.pack1Temp = carData.batteryStatus.pack1Temp;
  wcbData.batteryStatus.pack2Temp = carData.batteryStatus.pack2Temp;

  // update sensor data
  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;

  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  wcbData.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;

  // send message
  esp_err_t result = esp_now_send(wcbAddress, (uint8_t *) &wcbData, sizeof(wcbData));

  // if (result == ESP_OK)
  //   Serial.println("WCB Update: Successful");
  // else
  //   Serial.println("WCB Update: Failed");

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Interrupt Handler for Timer 4
 * update the ARDAN with live car data
 */
void UpdateARDAN()
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // send LoRa update
  LoRa.beginPacket();
  LoRa.write((uint8_t *) &carData, sizeof(carData));
  LoRa.endPacket();

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief Get the Commanded Torque from pedal values
 */
void GetCommandedTorque()
{
  // TODO: ensure the two pedal reads are within margin of error

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
      carData.drivingData.driveMode = SLOW;

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
 * @brief a callback function for when data is received from WCB
 * 
 * @param mac             the address of the WCB
 * @param incomingData    the structure of incoming data
 * @param length          size of the incoming data
 */
void WCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length)
{
  // disable interrupts
  portENTER_CRITICAL_ISR(&timerMux);

  // copy data to the wcbData struct 
  memcpy(&wcbData, incomingData, sizeof(wcbData));

  // get updated WCB data
  carData.drivingData.driveDirection = wcbData.drivingData.driveDirection;
  carData.drivingData.driveMode = wcbData.drivingData.driveMode;
  carData.inputs.coastRegen = wcbData.io.coastRegen;
  carData.inputs.brakeRegen = wcbData.io.brakeRegen;
  carData.outputs.buzzerActive = wcbData.io.buzzerActive;
  carData.drivingData.readyToDrive = wcbData.drivingData.readyToDrive;

  // re-enable interrupts
  portEXIT_CRITICAL_ISR(&timerMux);
}


/**
 * @brief scale a value inside a range of values to a new range of values
 * 
 * @param x             input value to be re-mapped
 * @param in_min        input value min of range
 * @param in_max        input value max of range
 * @param out_min       output value min of range
 * @param out_max       output value max of range
 * @return              the remapped value
 */
long MapValue(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * @brief 
 * 
 */
void PrintCANDebug() {

}


/**
 * @brief 
 * 
 */
void PrintWCBDebug() {

}


/**
 * @brief 
 * 
 */
void PrintIODebug() {

}


/**
 * @brief 
 * 
 */
void PrintDebug() {
    // CAN
    if (debugger.CAN_debugEnabled) {
        PrintCANDebug();
    }

    // WCB
    if (debugger.WCB_debugEnabled) {
      PrintWCBDebug();
    }

    // IO
    if (debugger.IO_debugEnabled) {
      PrintIODebug();
    }
}