/**
 * @file dataTypes.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-29
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
    uint16_t batteryChargeState;
    int16_t busVoltage;
    int16_t rinehartVoltage;
    float pack1Temp;
    float pack2Temp;
  } batteryStatus;

  struct Sensors
  {
    uint16_t wheelSpeedFR;
    uint16_t wheelSpeedFL;
    uint16_t wheelSpeedBR;
    uint16_t wheelSpeedBL;

    uint16_t wheelHeightFR;
    uint16_t wheelHeightFL;
    uint16_t wheelHeightBR;
    uint16_t wheelHeightBL;

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