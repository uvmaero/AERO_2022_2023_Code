/**
 * @file dataTypes.h
 * @author Dominic Gasperini
 * @brief all of the unique data types used to manage the state of the car
 * @version 1.0
 * @date 2023-04-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */


// includes
#include <esp_err.h>


/**
 * @brief the different throttle modes for to modify driving behavior
 * 
 */
typedef enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
} DriveModes;


/**
 * @brief each state that the precharge state machine can be in
 * 
 */
typedef enum PrechargeStates
{
  PRECHARGE_OFF = 0,
  PRECHARGE_ON = 1,
  PRECHARGE_DONE = 2,
  PRECHARGE_ERROR = 3,

} PrechargeStates;


/**
 * @brief the entire current state of the car
 * 
 */
typedef struct CarData
{
  struct DrivingData
  {
    bool readyToDrive;
    bool enableInverter;
    PrechargeStates prechargeState;

    bool imdFault;
    bool bmsFault;

    uint16_t commandedTorque;
    float currentSpeed;
    bool driveDirection;             // true = forward | false = reverse
    DriveModes driveMode;
  } drivingData;
  
  struct BatteryStatus
  {
    float batteryChargeState;
    float busVoltage;
    float rinehartVoltage;
    float pack1Temp;
    float pack2Temp;
    float packCurrent;
    float minCellVoltage;
    float maxCellVoltage;
  } batteryStatus;

  struct Sensors
  {
    uint8_t rpmCounterFR;
    uint8_t rpmCounterFL;
    uint8_t rpmCounterBR;
    uint8_t rpmCounterBL;
    uint64_t rpmTimeFR;     // the last time in microseconds that the wheel rpm was calculated at
    uint64_t rpmTimeFL;
    uint64_t rpmTimeBR;
    uint64_t rpmTimeBL;

    float wheelSpeedFR;
    float wheelSpeedFL;
    float wheelSpeedBR;
    float wheelSpeedBL;

    float wheelHeightFR;
    float wheelHeightFL;
    float wheelHeightBR;
    float wheelHeightBL;

    uint16_t steeringWheelAngle;

    float vicoreTemp;
    float pumpTempIn;
    float pumpTempOut;
  } sensors;

  struct Inputs
  {
    uint16_t pedal0;
    uint16_t pedal1;
    uint16_t brakeFront;
    uint16_t brakeRear;
    uint16_t brakeRegen;
    uint16_t coastRegen;
  } inputs;

  struct Outputs
  {
    bool buzzerActive;
    uint8_t buzzerCounter;
    bool brakeLight;
    bool fansActive;
    bool pumpActive;
  } outputs;

} CarData;



/**
 * @brief Debugger Structure
 * 
 */
typedef struct Debugger
{
  // debug toggle
  bool debugEnabled;
  bool CAN_debugEnabled;
  bool WCB_debugEnabled;
  bool IO_debugEnabled;
  bool scheduler_debugEnable;

  // debug data
  esp_err_t CAN_rineCtrlResult;
  esp_err_t CAN_rcbCtrlResult;
  uint8_t CAN_rineCtrlOutgoingMessage[8];
  uint8_t CAN_rcbCtrlOutgoingMessage[8];

  esp_err_t RCB_updateResult = ESP_OK;
  CarData RCB_updateMessage = {};

  esp_err_t WCB_updateResult;
  CarData WCB_updateMessage;

  CarData IO_data;

  int ardanTransmitResult;

  // scheduler data
  int sensorTaskCount;
  int canTaskCount;
  int ardanTaskCount;
  int espnowTaskCount;
} Debugger;


// debug functions
void PrintDebug();
void PrintCANDebug();
void PrintWCBDebug();
void PrintIODebug();