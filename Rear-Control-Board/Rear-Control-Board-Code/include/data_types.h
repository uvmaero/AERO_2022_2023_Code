/**
 * @file dataTypes.h
 * @author Dominic Gasperini
 * @brief all of the unique data types used to manage the state of the car
 * @version 1.0
 * @date 2023-02-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */


// Drive Mode Enumeration Type
typedef enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
} DriveModes;


/**
 * @brief 
 * 
 */
typedef struct CarData
{
  struct DrivingData
  {
    bool readyToDrive;
    bool enableInverter;

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
  } sensors;

  struct Inputs
  {
    uint16_t pedal0;
    uint16_t pedal1;
    uint16_t brake0;
    uint16_t brake1;
    uint16_t brakeRegen;
    uint16_t coastRegen;

    float vicoreTemp;
    float pumpTempIn;
    float pimpTempOut;
  } inputs;

  struct Outputs
  {
    bool buzzerActive;
    uint8_t buzzerCounter;
    bool brakeLight;
  } outputs;

} CarData;