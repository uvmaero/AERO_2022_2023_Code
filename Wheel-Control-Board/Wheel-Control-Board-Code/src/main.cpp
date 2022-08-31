/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief this drives the wheel control board on clean speed 5.5
 * @version 0.7
 * @date 2022-08-04
 */


// *** includes *** // 
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include "pinConfig.h"


// *** defines *** // 
#define TIMER_INTERRUPT_PRESCALER       80          // this is based off to the clock speed (assuming 80 MHz), gets us to microseconds
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define FCB_UPDATE_INTERVAL             125000      // 0.125 seconds in microseconds


// *** global variables *** //
// Drive Mode Enumeration
enum DriveModes
{
  SLOW = 0,
  ECO = 10,
  FAST = 20
};

// ESP-NOW Connection
uint8_t fcbAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info fcbInfo;

// ESP-NOW data transfer struct
struct FCB_Data
{
  struct DrivingData
  {
    bool readyToDrive = false;

    bool imdFault = false;
    bool bmsFault = false;

    bool driveDirection = true;             // true = forward | false = reverse
    DriveModes driveMode = ECO;

    uint8_t currentSpeed = 0;
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
FCB_Data fcbData; 

// Car Data Struct
struct DashData
{
  struct DrivingData
  {
    bool readyToDrive = false;

    bool imdFault = false;
    bool bmsFault = false;

    bool driveDirection = true;             // true = forward | false = reverse
    DriveModes driveMode = ECO;

    uint8_t currentSpeed = 0;
  } drivingData;
  
  struct BatteryStatus
  {
    uint8_t batteryChargeState = 0;
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
    uint8_t brakeRegen = 0;
    uint8_t coastRegen = 0;
  } inputs;

  struct Outputs
  {
    bool buzzerActive = false;
  } outputs;
};
DashData dashData;

// Hardware ISR Timers
hw_timer_t* timer0 = NULL;
hw_timer_t* timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// LCD 
LiquidCrystal_I2C lcd(0x27, 20, 2);    // set the LCD address to 0x27 for a 16 chars and 2 line display
enum LCDMode
{
  RACE_MODE = 0,
  ELECTRICAL_MODE = 10,
  MECHANICAL_MODE = 20
};
LCDMode lcdMode = RACE_MODE;
LCDMode previousLcdMode = lcdMode;


// *** function declarations *** //
void PollInputData();
void UpdateFCB();
void FCBDataSent(const uint8_t* macAddress, esp_now_send_status_t status);
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length);
void AfterInitLCD();
void UpdateLCD();
void DisplayRaceMode();
void DisplayElectricalMode();
void DisplayMechanicalMode();
void LCDButtonInterrupt();
void ReadyToDriveButtonInterrupt();
void DriveModeButtonInterrupt();


// *** setup *** //
void setup()
{
  // --- initialize serial --- //
  Serial.begin(9600);

  // --- initialize inputs --- //
  pinMode(COAST_REGEN, INPUT);
  pinMode(BRAKE_REGEN, INPUT);
  pinMode(READY_TO_DRIVE_BUTTON, INPUT);
  pinMode(DRIVE_DIRECTION_SWITCH, INPUT);
  pinMode(DRIVE_MODE_BUTTON, INPUT);
  pinMode(LCD_BUTTON, INPUT);

  // --- initalize outputs --- //
  pinMode(IMD_FAULT_LED, OUTPUT);
  pinMode(BMS_FAULT_LED, OUTPUT);
  pinMode(READY_TO_DRIVE_LED, OUTPUT);

  // --- initialize LCD --- //
  lcd.init();
  lcd.backlight();
  lcd.noCursor();
  lcd.noAutoscroll();
  AfterInitLCD();

  // --- initialize ESP-NOW ---//
  // turn on wifi access point 
  WiFi.mode(WIFI_STA);

  // init ESP-NOW service
  if (esp_now_init() != ESP_OK)
    Serial.println("ESP-NOW INIT [ SUCCESS ]");
  else
    Serial.println("ESP-NOW INIT [ FAILED ]");
  
  // attach the data send function to the message sent callback
  esp_now_register_send_cb(FCBDataSent);
  
  // get peer informtion about WCB
  memcpy(fcbInfo.peer_addr, fcbAddress, sizeof(fcbAddress));
  fcbInfo.channel = 0;
  fcbInfo.encrypt = false;

  // add WCB as a peer
  if (esp_now_add_peer(&fcbInfo) != ESP_OK)
    Serial.println("ESP-NOW CONNECTION [ SUCCESS ]");
  else
    Serial.println("ESP-NOW CONNECTION [ FAILED ]");

  // attach message received callback to the data received function
  esp_now_register_recv_cb(FCBDataReceived);

  // --- initialize timer interrupts --- //
  timer0 = timerBegin(0, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer0, &PollInputData, true);
  timerAlarmWrite(timer0, SENSOR_POLL_INTERVAL, true);
  timerAlarmEnable(timer0);

  timer1 = timerBegin(1, TIMER_INTERRUPT_PRESCALER, true);
  timerAttachInterrupt(timer1, &UpdateFCB, true);
  timerAlarmWrite(timer1, FCB_UPDATE_INTERVAL, true);
  timerAlarmEnable(timer1);

  // --- initialize hardware interrupts --- //
  attachInterrupt(LCD_BUTTON, &LCDButtonInterrupt, LOW);
  attachInterrupt(DRIVE_MODE_BUTTON, &DriveModeButtonInterrupt, LOW);
  attachInterrupt(READY_TO_DRIVE_BUTTON, &ReadyToDriveButtonInterrupt, LOW);
}


// *** loop *** // 
void loop()
{ 
  // update LEDs
  digitalWrite(IMD_FAULT_LED, dashData.drivingData.imdFault);
  digitalWrite(BMS_FAULT_LED, dashData.drivingData.bmsFault);

  // update wheel dash LCD
  UpdateLCD();
}


/**
 * @brief 
 * 
 */
void PollInputData()
{
  // regen knobs
  dashData.inputs.brakeRegen = analogRead(BRAKE_REGEN);
  dashData.inputs.coastRegen = analogRead(COAST_REGEN);
}


/**
 * @brief 
 * 
 */
void UpdateFCB()
{
  // update input data
  fcbData.drivingData.readyToDrive = dashData.drivingData.readyToDrive;
  fcbData.drivingData.driveDirection = dashData.drivingData.driveDirection;
  fcbData.io.brakeRegen = dashData.inputs.brakeRegen;
  fcbData.io.coastRegen = dashData.inputs.coastRegen;
  fcbData.io.buzzerActive = dashData.outputs.buzzerActive;

  // send mesasge
  esp_err_t result = esp_now_send(fcbAddress, (uint8_t *) &fcbData, sizeof(fcbData));

  if (result == ESP_OK)
    Serial.println("WCB Update: Successful");
  else
    Serial.println("WCB Update: Failed");
}


/**
 * @brief a callback function for when data is sent to FCB
 * 
 * @param macAddress      the address of the FCB
 * @param status          indicator of successful message sent
 */
void FCBDataSent(const uint8_t* macAddress, esp_now_send_status_t status)
{
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}


/**
 * @brief a callback function for when data is received from FCB
 * 
 * @param mac             the address of the FCB
 * @param incomingData    the structure of incoming data
 * @param length          size of the incoming data
 */
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length)
{
  // copy data to the fcbData struct 
  memcpy(&fcbData, incomingData, sizeof(fcbData));

  // updated mechanical data
  dashData.drivingData.currentSpeed = fcbData.drivingData.currentSpeed;

  // update battery & electrical data
  dashData.batteryStatus.batteryChargeState = fcbData.batteryStatus.batteryChargeState;
  dashData.batteryStatus.busVoltage = fcbData.batteryStatus.busVoltage;
  dashData.batteryStatus.rinehartVoltage = fcbData.batteryStatus.rinehartVoltage;
  dashData.batteryStatus.pack1Temp = fcbData.batteryStatus.pack1Temp;
  dashData.batteryStatus.pack2Temp = fcbData.batteryStatus.pack2Temp;

  // update sensor data
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedFR;
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedFL;
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedBR;
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedBL;
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedFR;
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedFL;
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedBR;
  dashData.sensors.wheelSpeedFR = fcbData.sensors.wheelSpeedBL;
}


/**
 * @brief 
 * 
 */
void ReadyToDriveButtonInterrupt()
{
  // update ready to drive status
  dashData.drivingData.readyToDrive = true;

  // update buzzer status
  dashData.outputs.buzzerActive = true;
}


/**
 * @brief 
 * 
 */
void DriveModeButtonInterrupt()
{
  int mode = dashData.drivingData.driveMode;

  // increment mode
  mode += 10;

  // make sure we aren't out of bounds
  if (mode > 20)
  {
    mode = 0;     // loop back around to start
  }

  dashData.drivingData.driveMode = (DriveModes)mode;
}


/**
 * @brief 
 * 
 */
void LCDButtonInterrupt()
{
  int mode = lcdMode;

  // increment mode
  mode += 10;

  // make sure we aren't out of bounds
  if (mode > 20)
  {
    mode = 0;       // loop back around to start
  }

  lcdMode = (LCDMode)mode;
}


/**
 * @brief 
 * 
 */
void UpdateLCD()
{
  // check to see if the display mode has changed
  if (lcdMode != previousLcdMode)
    lcd.clear();

  // update the values on the display
  switch (lcdMode)
  {
    case RACE_MODE:
    previousLcdMode = lcdMode;
    DisplayRaceMode();
    break;

    case ELECTRICAL_MODE:
    previousLcdMode = lcdMode;
    DisplayElectricalMode();
    break;

    case MECHANICAL_MODE:
    previousLcdMode = lcdMode;
    DisplayMechanicalMode();
    break;

    default:
    lcdMode = RACE_MODE;
    break;
  }
}


/**
 * @brief 
 * 
 */
void AfterInitLCD()
{
  lcd.clear();

  // on line 1
  lcd.setCursor(3, 0);
  lcd.printf("Booting Up...");

  // on line 2
  lcd.setCursor(4, 1);
  lcd.printf("~AERO~");
}


/**
 * @brief LCD display setup for when the car is being driven
 * 
 */
void DisplayRaceMode()
{
  // line 1 - mph
  lcd.setCursor(7, 0);
  lcd.print("    ");
  lcd.printf("%d mph", dashData.drivingData.currentSpeed);

  // line 2 - drive direction
  lcd.setCursor(0, 1);
  lcd.print("   ");
  if (dashData.drivingData.driveDirection)
  {
    lcd.print("FWD");
  }

  else
  {
    lcd.print("RVS");
  }

  // line 2 - drive mode
  lcd.setCursor(8, 1);
  lcd.print("    ");
  switch (dashData.drivingData.driveMode)
  {
    case SLOW:
    lcd.print("SLOW");
    break;

    case ECO:
    lcd.print("ECO");
    break;

    case FAST:
    lcd.print("FAST");
    break;

    default:
    lcd.print("MODE ERR");
    break;
  }

  // line 2 - battery percentage
  lcd.setCursor(16, 1);
  lcd.print("    ");
  lcd.printf("%d%%", dashData.batteryStatus.batteryChargeState);
}


/**
 * @brief 
 * 
 */
void DisplayElectricalMode()
{

}


/**
 * @brief 
 * 
 */
void DisplayMechanicalMode()
{

}
