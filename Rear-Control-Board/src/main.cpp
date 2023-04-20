/**
 * @file main.cpp
 * @author Dominic Gasperini - UVM '23
 * @brief this drives the rear control board on clean speed 5.5
 * @version 1.0
 * @date 2023-04-03
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/
// standard includes 
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "driver/can.h"
#include "rtc.h"
#include "rtc_clk_common.h"
#include "driver/can.h"

#include "FS.h"
#include "SD_MMC.h"

#include <pin_config.h>
#include <data_types.h>


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/


// SD Card
#define SD_BUFF_LEN                     128

// definitions
#define TIRE_DIAMETER                   20.0        // diameter of the vehicle's tires in inches
#define WHEEL_RPM_CALC_THRESHOLD        25          // the number of times the hall effect sensor is tripped before calculating vehicle speed
#define PRECHARGE_FLOOR                 0.8         // minimum percentage of acceptable voltage to run car
#define MIN_BUS_VOLTAGE                 150         // this should be a voltage that can only be met if 1 pack is connected

// CAN
#define NUM_CAN_READS                   6           // in general, the number of expected messages times 2, so 6 
#define FCB_CONTROL_ADDR                0x00A       // 
#define FCB_DATA_ADDR                   0x00B       // 
#define RCB_CONTROL_ADDR                0x00C       // address for critical data sharing
#define RCB_DATA_ADDR                   0x00D       // address for non-critical data sharing
#define RINE_MOTOR_INFO_ADDR            0x0A5       // get motor information from Rinehart 
#define RINE_VOLT_INFO_ADDR             0x0A7       // get rinehart electrical information
#define RINE_BUS_INFO_ADDR              0x0AA       // get rinehart relay information 
#define RINE_BUS_CONTROL_ADDR           0x0C1       // control rinehart relay states
#define BMS_GEN_DATA_ADDR               0x6B0       // important BMS data
#define BMS_CELL_DATA_ADDR              0x6B2       // cell data

// ESP-NOW
#define WCB_ESP_NOW_ADDRESS             {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x80}
#define FCB_ESP_NOW_ADDRESS             {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81}
#define RCB_ESP_NOW_ADDRESS             {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x82}

// tasks & timers
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define ESP_NOW_UPDATE_INTERVAL         200000      // 0.2 seconds in microseconds
#define CAN_UPDATE_INTERVAL             100000      // 0.1 seconds in microseconds
#define LOGGER_UPDATE_INTERVAL          200000      // 0.2 seconds in microseconds
#define TASK_STACK_SIZE                 4096        // in bytes
#define CAN_BLOCK_DELAY                 100         // time to block to complete function call in FreeRTOS ticks (milliseconds)

// debug
#define ENABLE_DEBUG                    true       // master debug control
#if ENABLE_DEBUG
  #define MAIN_LOOP_DELAY               1000
#else
  #define MAIN_LOOP_DELAY               1
#endif


/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/


/**
 * @brief debugger structure used for organizing debug information
 * 
 */
Debugger debugger = {
  // debug toggle
  .debugEnabled = ENABLE_DEBUG,
  .CAN_debugEnabled = false,
  .IO_debugEnabled = true,
  .Logger_debugEnabled = false,
  .scheduler_debugEnable = false,

  // debug data
  .prechargeResult = ESP_OK,
  .fcbCtrlResult = ESP_OK,
  .fcbDataResult = ESP_OK,
  
  .CAN_prechargeOutgoingMessage = {},
  .CAN_fcbDataOutgoingMessage = {},
  .CAN_fcbCtrlOutgoingMessage = {},

  .WCB_updateResult = ESP_OK,
  .WCB_updateMessage = {},

  .IO_data = {},

  .prechargeState = PRECHARGE_OFF,

  // scheduler data
  .sensorTaskCount = 0,
  .prechargeTaskCount = 0,
  .canReadTaskCount = 0,
  .canWriteTaskCount = 0,
  .loggerTaskCount = 0,
  .wcbTaskCount = 0,
};


/**
 * @brief the dataframe that describes the entire state of the car
 * 
 */
CarData carData = {
  // driving data
  .drivingData = {
    .readyToDrive = false,
    .enableInverter = false,
    .prechargeState = PRECHARGE_OFF,

    .imdFault = LOW,
    .bmsFault = LOW,

    .commandedTorque = 0,
    .currentSpeed = 0.0f,
    .driveDirection = true,
    .driveMode = ECO, 
  },

  // Battery Status
  .batteryStatus = {
    .batteryChargeState = 0,
    .busVoltage = 0,
    .rinehartVoltage = 0,
    .pack1Temp = 0.0f,
    .pack2Temp = 0.0f,
    .packCurrent = 0.0f,
    .minCellVoltage = 0.0f,
    .maxCellVoltage = 0.0f,
  },

  // Sensors
  .sensors = {
    .rpmCounterFR = 0,
    .rpmCounterFL = 0,
    .rpmCounterBR = 0,
    .rpmCounterBL = 0,
    .rpmTimeFR = 0,
    .rpmTimeFL = 0,
    .rpmTimeBR = 0,
    .rpmTimeBL = 0,

    .wheelSpeedFR = 0.0f,
    .wheelSpeedFL = 0.0f,
    .wheelSpeedBR = 0.0f,
    .wheelSpeedBL = 0.0f,

    .wheelHeightFR = 0.0f,
    .wheelHeightFL = 0.0f,
    .wheelHeightBR = 0.0f,
    .wheelHeightBL = 0.0f,

    .steeringWheelAngle = 0,

    .vicoreTemp = 0.0f,
    .pumpTempIn = 0.0f,
    .pumpTempOut = 0.0f,
  },

  // Inputs
  .inputs = {
    .pedal0 = 0,
    .pedal1 = 0,
    .brakeFront = 0,
    .brakeRear = 0,
    .brakeRegen = 0,
    .coastRegen = 0,
  },

  // Outputs
  .outputs = {
    .buzzerActive = false,
    .buzzerCounter = 0,
    .brakeLight = false,
    .fansActive = false,
  }
};


// Hardware Timers
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;
hw_timer_t *timer4 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// ESP-NOW Peers
esp_now_peer_info_t wcbInfo = {
  .peer_addr = WCB_ESP_NOW_ADDRESS,
  .channel = 1,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};

esp_now_peer_info_t fcbInfo = {
  .peer_addr = FCB_ESP_NOW_ADDRESS,
  .channel = 1,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};


// CAN
static const can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
// {
//   .acceptance_code = 0x000, // begin range
//   .acceptance_mask = 0x0C4, // end range
//   .single_filter = true
// };
static const can_general_config_t can_general_config = CAN_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, CAN_MODE_NORMAL);


// SD Card Interface
int sdLogFileNumber;
char sdTrackerFilename[SD_BUFF_LEN] = "/tracker.txt";
char sdLogFilname [SD_BUFF_LEN] = "";

/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void SensorCallback();
void CANCallback();
void ESPNOWCallback();
void LoggerCallback();
void BRWheelSensorCallback();
void BLWheelSensorCallback();

// tasks
void ReadSensorsTask(void* pvParameters);
void CANReadTask(void* pvParameters);
void CANWriteTask(void* pvParameters);
void UpdateLoggerTask(void* pvParameters);
void UpdateWCBTask(void* pvParameters);
void PrechargeTask(void* pvParameters);

// ISRs
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length);

// helpers
void GenerateFilename();


/*
===============================================================================================
                                            Setup 
===============================================================================================
*/


void setup()
{
  // set power configuration
  esp_pm_configure(&power_configuration);

  if (debugger.debugEnabled) {
    // delay startup by 3 seconds
    vTaskDelay(3000);
  }

  // -------------------------- initialize serial connection ------------------------ //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");

  // setup managment struct
  struct setup
  {
    bool ioActive = false;
    bool canActive = false;
    bool fcbActive = false;
    bool wcbActive = false;
    bool loggerActive = false;
  };
  setup setup;

  // -------------------------- initialize GPIO ------------------------------------- //
  analogReadResolution(12);

  // inputs
  pinMode(WHEEL_SPEED_BR_SENSOR, INPUT);
  pinMode(WHEEL_SPEED_BL_SENSOR, INPUT);

  pinMode(WHEEL_HEIGHT_BR_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_BL_SENSOR, INPUT);

  pinMode(RAD_TEMP_IN_PIN, INPUT);
  pinMode(RAD_TEMP_OUT_PIN, INPUT);

  pinMode(IMD_FAULT_PIN, INPUT);
  pinMode(BMS_FAULT_PIN, INPUT);

  // outputs
  pinMode(FAN_ENABLE_PIN, OUTPUT);
  pinMode(PUMP_ENABLE_PIN, OUTPUT);
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);

  // interrupts
  attachInterrupt(WHEEL_SPEED_BR_SENSOR, BRWheelSensorCallback, ONHIGH);
  attachInterrupt(WHEEL_SPEED_BL_SENSOR, BLWheelSensorCallback, ONHIGH);

  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //


  // --------------------- initialize CAN Controller -------------------------- //
  // install CAN driver
  if(can_driver_install(&can_general_config, &can_timing_config, &can_filter_config) == ESP_OK) {
    Serial.printf("CAN DRIVER INSTALL [ SUCCESS ]\n");

    // start CAN bus
    if (can_start() == ESP_OK) {
      Serial.printf("CAN INIT [ SUCCESS ]\n");

      setup.canActive = true;
    }

    else {
      Serial.printf("CAN INIT [ FAILED ]\n");
    }
  }

  else {
    Serial.printf("CAN DRIVER INSTALL [ FAILED ]\n");
  }
  // --------------------------------------------------------------------------- //


  // -------------------------- initialize ESP-NOW  ---------------------------- //
  // init wifi and config
  if (WiFi.mode(WIFI_STA)) {
    Serial.printf("WIFI INIT [ SUCCESS ]\n");

    // set custom mac address
    const uint8_t rcbAddress[6] = RCB_ESP_NOW_ADDRESS;
    if (esp_wifi_set_mac(WIFI_IF_STA, rcbAddress) == ESP_OK) {
      Serial.printf("WIFI SET MAC [ SUCCESS ]\n");
    
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
      Serial.print("WIFI MAC: "); Serial.println(WiFi.macAddress());
      Serial.print("WIFI CHANNEL: "); Serial.println(WiFi.channel());

      if (esp_now_init() == ESP_OK) {
        Serial.printf("ESP-NOW INIT [ SUCCESS ]\n");

        // add peers
        esp_err_t rcbResult = esp_now_add_peer(&fcbInfo);
        esp_err_t wcbResult = esp_now_add_peer(&wcbInfo);

        if (rcbResult == ESP_OK && wcbResult == ESP_OK) {
          Serial.printf("ESP-NOW ADD PEERS [ SUCCESS ]\n");

          setup.fcbActive = true;
          setup.wcbActive = true;
        }
        else {
          Serial.printf("ESP-NOW ADD PEERS [ FAILED ]\n");
        }
      }

      else {
        Serial.printf("ESP-NOW INIT [ FAILED ]\n");
      }
    }
    else {
      Serial.printf("WIFI SET MAC [ FAILED ]\n");
    }
  }

  else {
    Serial.printf("WIFI INIT [ FAILED ]\n");
  }


  // ------------------------------------------------------------------------ //


  // ---------------------- initialize SD Logger ---------------------------- //
  // init sd card
  if (SD_MMC.begin()) {
    Serial.printf("SD CARD INIT [ SUCCESS ]\n");
    // inits
    int trackerNumber;

    // check for sd card inserted
    uint8_t cardType = SD_MMC.cardType();
    if (cardType != CARD_NONE) {
      Serial.printf("SD CARD DETECT [ CONNECTED: ");
      // print card type
      if(cardType == CARD_MMC){
        Serial.print("MMC");
      } else if(cardType == CARD_SD){
          Serial.print("SDSC");
      } else if(cardType == CARD_SDHC){
          Serial.print("SDHC");
      } else {
          Serial.print("UNKNOWN");
      }
      Serial.printf(" ]\n");

      // print card data metrics
      Serial.printf("SD CARD SIZE: %lluMB\n", SD_MMC.cardSize());
      Serial.printf("SD CARD USED STORAGE: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

      // collect tracker information from SD & update it
      File trackerFile = SD_MMC.open(sdTrackerFilename, FILE_READ);
      if (trackerFile) {
        Serial.printf("SD CARD TRACKER FILE READ [ SUCCESS ]\n");

        while (trackerFile.available()) {
          trackerNumber = trackerFile.read();
          Serial.printf("tracker #: %d\n", trackerNumber);
        }

        // update tracker number
        File tmpFile = SD_MMC.open("/tmp.txt", FILE_WRITE);
        if (tmpFile) {
          trackerNumber++;
          tmpFile.print(trackerNumber);
          Serial.printf("SD CARD TRACKER FILE UPDATE [ SUCCESS ]\n");
        }
        else {
          Serial.printf("SD CARD TRACKER FILE UPDATE [ FAILED ]\n");
        }

        // delete old tracker file 
        SD_MMC.remove(sdTrackerFilename);

        // rename tmp file
        SD_MMC.rename("/tmp.txt", sdTrackerFilename);

        // create new log file
        GenerateFilename();

        File logFile = SD_MMC.open(sdLogFilname, FILE_WRITE);
        if (logFile) {
          Serial.printf("SD CARD LOG FILE CREATED [ SUCCESS ]\n");
        }
        else {
          Serial.printf("SD CARD LOG FILE CREATED [ FAILED ]\n");
        }

        setup.loggerActive = true;
      }

      else {
        Serial.printf("SD TRACKER FILE READ [ FAILED ]\n");
      }
    }
    
    else {
      Serial.printf("SD CARD DETECT [ FAILED ]\n");
    }
  }
  
  else {
    Serial.printf("SD CARD INIT [ FAILED ]\n");
  }
  // ------------------------------------------------------------------------ //


  // ---------------------- initialize timer interrupts ---------------------- //
  // timer 1 - Read Sensors 
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &SensorCallback, true);
  timerAlarmWrite(timer1, SENSOR_POLL_INTERVAL, true);

  // timer 2 - CAN Update
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &CANCallback, true);
  timerAlarmWrite(timer2, CAN_UPDATE_INTERVAL, true);

  // timer 3 - Logger Update
  timer3 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer3, &LoggerCallback, true);
  timerAlarmWrite(timer3, LOGGER_UPDATE_INTERVAL, true);

  // timer 4 - ESP-NOW Update
  timer4 = timerBegin(3, 80, true);
  timerAttachInterrupt(timer4, &ESPNOWCallback, true);
  timerAlarmWrite(timer4, ESP_NOW_UPDATE_INTERVAL, true);

  // start timers
  if (setup.ioActive)
    timerAlarmEnable(timer1);
  if (setup.canActive)
    timerAlarmEnable(timer2);
  if (setup.loggerActive)
    timerAlarmEnable(timer3);
  if (setup.fcbActive && setup.wcbActive)
    timerAlarmEnable(timer4);

  // ----------------------------------------------------------------------------------------- //


  // --------------------------------- Scheduler Status -------------------------------------- //
  Serial.printf("SENSOR TASK STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("PRECHARGE TASK STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("CAN READ STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("CAN WRITE STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("LOGGER TASK STATUS: %s\n", timerAlarmEnabled(timer3) ? "RUNNING" : "DISABLED");
  Serial.printf("ESP-NOW TASK STATUS: %s\n", timerAlarmEnabled(timer4) ? "RUNNING" : "DISABLED");

  if (xTaskGetSchedulerState() == 2) {
    Serial.printf("\nScheduler Status: RUNNING\n");

    // clock frequency
    rtc_cpu_freq_config_t clock_config;
    rtc_clk_cpu_freq_get_config(&clock_config);
    Serial.printf("CPU Frequency: %dMHz\n", clock_config.freq_mhz);
  }
  else {
    Serial.printf("\nScheduler STATUS: FAILED\nHALTING OPERATIONS");
    while (1) {};
  }
  Serial.printf("\n\n|--- END SETUP ---|\n\n");
  // ---------------------------------------------------------------------------------------- //
}


/*
===============================================================================================
                                    Callback Functions
===============================================================================================
*/


/**
 * @brief callback function for creating a new sensor poll task
 * 
 * @param args arguments to be passed to the task
 */
void SensorCallback() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  // inits
  static uint8_t ucParameterToPassSensor;
  TaskHandle_t xHandleSensor = NULL;

  static uint8_t ucParameterToPassPrecharge;
  TaskHandle_t xHandlePrecharge = NULL;

  // queue tasks 
  xTaskCreate(ReadSensorsTask, "Poll-Senser-Data", TASK_STACK_SIZE, &ucParameterToPassSensor, tskIDLE_PRIORITY, &xHandleSensor);
  xTaskCreate(PrechargeTask, "Precharge-Data", TASK_STACK_SIZE, &ucParameterToPassPrecharge, 10, &xHandlePrecharge);
  
  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for creating a new CAN Update task
 * 
 * @param args arguments to be passed to the task
 */
void CANCallback() {
  portENTER_CRITICAL_ISR(&timerMux);

  static uint8_t ucParameterToPassWrite;
  TaskHandle_t xHandleWrite = NULL;

  static uint8_t ucParameterToPassRead;
  TaskHandle_t xHandleRead = NULL;

  // queue tasks
  xTaskCreate(CANReadTask, "CAN-Read", TASK_STACK_SIZE, &ucParameterToPassRead, 25, &xHandleRead);
  xTaskCreate(CANWriteTask, "CAN-Write", TASK_STACK_SIZE, &ucParameterToPassWrite, 20, &xHandleWrite);

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for creating a new Logger Update task
 * 
 * @param args arguments to be passed to the task
 */
void LoggerCallback() { 
  portENTER_CRITICAL_ISR(&timerMux);
  
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateLoggerTask, "Logger-Update", TASK_STACK_SIZE, &ucParameterToPass, 2, &xHandle);
  
  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for creating a new WCB Update task
 * 
 * @param args arguments to be passed to the task
 */
void ESPNOWCallback() {
  portENTER_CRITICAL_ISR(&timerMux);

  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateWCBTask, "WCB-Update", TASK_STACK_SIZE, &ucParameterToPass, 2, &xHandle);

  portEXIT_CRITICAL_ISR(&timerMux);
  
  return;
}


/**
 * @brief a callback function for when data is received from FCB
 * 
 * @param mac             the address of the FCB
 * @param incomingData    the structure of incoming data
 * @param length          size of the incoming data
 */
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length) {
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  CarData tmp;

  // copy data to the tmp struct 
  memcpy((uint8_t *) &tmp, incomingData, sizeof(incomingData));

  // update relevant data
  carData.sensors.steeringWheelAngle = tmp.sensors.steeringWheelAngle;

  carData.sensors.wheelSpeedFR = tmp.sensors.wheelSpeedFR;
  carData.sensors.wheelSpeedFR = tmp.sensors.wheelSpeedFR;
  carData.sensors.wheelSpeedFR = tmp.sensors.wheelSpeedFR;
  carData.sensors.wheelSpeedFR = tmp.sensors.wheelSpeedFR;

  carData.sensors.wheelHeightFR = tmp.sensors.wheelHeightFR;
  carData.sensors.wheelHeightFR = tmp.sensors.wheelHeightFR;
  carData.sensors.wheelHeightFR = tmp.sensors.wheelHeightFR;
  carData.sensors.wheelHeightFR = tmp.sensors.wheelHeightFR;

  carData.drivingData.driveMode = tmp.drivingData.driveMode;

  carData.drivingData.currentSpeed = tmp.drivingData.currentSpeed;

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for when the hall effect sensor fires on front right wheel
 * 
 * @param args 
 */
void BRWheelSensorCallback() {
  portENTER_CRITICAL_ISR(&timerMux);

  // increment pass counter
  carData.sensors.rpmCounterFR++;

  // calculate wheel rpm
  if (carData.sensors.rpmCounterFR > WHEEL_RPM_CALC_THRESHOLD) {
    // get time difference
    float timeDiff = (float)esp_timer_get_time() - (float)carData.sensors.rpmTimeFR;

    // get rpm
    carData.sensors.wheelSpeedFR = ((float)carData.sensors.rpmCounterFR / (timeDiff / 1000000.0)) * 60.0;

    // update time keeping
    carData.sensors.rpmTimeFR = esp_timer_get_time();

    // reset counter
    carData.sensors.rpmCounterFR = 0;
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for when the hall effect sensor fires on front left wheel
 * 
 * @param args 
 */
void BLWheelSensorCallback() {
  portENTER_CRITICAL_ISR(&timerMux);

  // increment pass counter
  carData.sensors.rpmCounterFL++;

  // calculate wheel rpm
  if (carData.sensors.rpmCounterFL > WHEEL_RPM_CALC_THRESHOLD) {
    // get time difference
    float timeDiff = (float)esp_timer_get_time() - (float)carData.sensors.rpmTimeFL;

    // get rpm
    carData.sensors.wheelSpeedFL = ((float)carData.sensors.rpmCounterFL / (timeDiff / 1000000.0)) * 60.0;

    // update time keeping
    carData.sensors.rpmTimeFL = esp_timer_get_time();

    // reset counter
    carData.sensors.rpmCounterFL = 0;
  }

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/*
===============================================================================================
                                FreeRTOS Task Functions
===============================================================================================
*/


/**
 * @brief reads sensors and updates car data 
 * 
 * @param pvParameters parameters passed to task
 */
void ReadSensorsTask(void* pvParameters)
{
  // turn off wifi for ADC channel 2 to function
  esp_wifi_stop();

  // read radiator sensors
  carData.sensors.pumpTempIn = analogRead(RAD_TEMP_IN_PIN);
  carData.sensors.pumpTempOut = analogRead(RAD_TEMP_OUT_PIN);

  // update pump
  if (carData.outputs.pumpActive) {
    digitalWrite(PUMP_ENABLE_PIN, HIGH);             // turn on pump
  }
  else {
    digitalWrite(PUMP_ENABLE_PIN, LOW);              // turn off pump
  }

  // update brake light state
  if (carData.outputs.brakeLight) {
    digitalWrite(BRAKE_LIGHT_PIN, HIGH);            // turn on the brake light
  }
  else {
    digitalWrite(BRAKE_LIGHT_PIN, LOW);             // turn off the brake light
  }

  // turn wifi back on to re-enable esp-now connection to wheel board
  esp_wifi_start();

  // update wheel ride height values
  carData.sensors.wheelHeightFR = analogRead(WHEEL_HEIGHT_BR_SENSOR);
  carData.sensors.wheelHeightFL = analogRead(WHEEL_HEIGHT_BL_SENSOR);

  // read brake sensor
  // int tmpBrake = analogRead(BRAKE_PIN);
  // carData.inputs.brakeRear = map(tmpBrake, 0, 1024, 0, 255);


  // update fans
  if (carData.outputs.fansActive) {
    digitalWrite(FAN_ENABLE_PIN, HIGH);              // turn on fans
  }
  else {
    digitalWrite(FAN_ENABLE_PIN, LOW);               // turn off fans
  }

  // bms fault
  if (digitalRead(IMD_FAULT_PIN) == HIGH) {   // HIGH = CLEAR | LOW = FAULT
    carData.drivingData.imdFault = false;
  }
  else {
    carData.drivingData.imdFault = true;
  }

  // imd fault
  if (digitalRead(BMS_FAULT_PIN) == HIGH) {    // HIGH = FAULT | LOW = CLEAR
    carData.drivingData.bmsFault = true;
  }
  else {
    carData.drivingData.bmsFault = false;
  }

  // debugging
  if (debugger.debugEnabled) {
    debugger.IO_data = carData;
    debugger.sensorTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief run precharge
 * 
 * @param pvParameters 
 */
void PrechargeTask(void* pvParameters) {
  // inits
  can_message_t outgoingMessage;
  int result;

  // precharge state machine
  switch (carData.drivingData.prechargeState) {

    // prepare for and start precharge
    case PRECHARGE_OFF:

      // set ready to drive state
      carData.drivingData.readyToDrive = false;

      if (carData.drivingData.imdFault == false && carData.drivingData.bmsFault == false) { // IMD: HIGH = clear | LOW = fault ||| BMS: HIGH = fault | LOW = clear
        carData.drivingData.prechargeState = PRECHARGE_ON;
      }

    break;

    // do precharge
    case PRECHARGE_ON:

      // set ready to drive state
      carData.drivingData.readyToDrive = false;

      // ensure voltages are above correct values
      if ((carData.batteryStatus.rinehartVoltage >= (carData.batteryStatus.busVoltage * PRECHARGE_FLOOR)) &&
      (carData.batteryStatus.rinehartVoltage > MIN_BUS_VOLTAGE)) {
        carData.drivingData.prechargeState = PRECHARGE_DONE;
      }

    break;


    // precharge complete!
    case PRECHARGE_DONE:

      // set ready to drive state
      carData.drivingData.readyToDrive = true;

      // if rinehart voltage drops below battery, something's wrong, 
      if (carData.batteryStatus.rinehartVoltage < MIN_BUS_VOLTAGE) {
        carData.drivingData.prechargeState = PRECHARGE_ERROR;
      }

    break;


    // error state
    case PRECHARGE_ERROR:

      // ensure car cannot drive
      carData.drivingData.readyToDrive = false;
      carData.drivingData.commandedTorque = 0;

      // reset precharge cycle
      carData.drivingData.prechargeState = PRECHARGE_OFF;

    break;
    

    // handle undefined behavior
    default:

      // if we've entered an undefined state, go to error mode
      carData.drivingData.prechargeState = PRECHARGE_ERROR;

    break;
  }

  // debugging 
  if (debugger.debugEnabled) {
    debugger.prechargeState = carData.drivingData.prechargeState;
    debugger.prechargeTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief reads the CAN bus
 * 
 * @param pvParameters parameters passed to task
 */
void CANReadTask(void* pvParameters) 
{
  // inits
  can_message_t incomingMessage;
  int incomingId;
  uint8_t tmp1, tmp2;

  // --- receive messages --- //

  // if rx queue is full clear it (this is bad, implement can message filtering)
  uint32_t alerts;
  can_read_alerts(&alerts, pdMS_TO_TICKS(CAN_BLOCK_DELAY));
  if (alerts & CAN_ALERT_RX_QUEUE_FULL) {
    can_clear_receive_queue();
  }

  // check for new messages in the CAN buffer
  for (int i = 0; i < NUM_CAN_READS; ++i) {
    if (can_receive(&incomingMessage, pdMS_TO_TICKS(CAN_BLOCK_DELAY)) == ESP_OK) { // if there are messages to be read
      incomingId = incomingMessage.identifier;
      
      // parse out data
      switch (incomingId) {
        // get data from FCB 
        case RCB_CONTROL_ADDR:
          // brake light
          carData.outputs.brakeLight = incomingMessage.data[0];
        break;

        // Rinehart: voltage information
        case RINE_VOLT_INFO_ADDR:
          // rinehart voltage is spread across the first 2 bytes
          tmp1 = incomingMessage.data[0];
          tmp2 = incomingMessage.data[1];

          // combine the first two bytes and assign that to the rinehart voltage
          carData.batteryStatus.rinehartVoltage = ((tmp2 << 8) | tmp1) / 10;   // little endian combination: value = (byte2 << 8) | byte1;
        break;

        // BMS: general pack data
        case BMS_GEN_DATA_ADDR:
          // pack current
          tmp1 = incomingMessage.data[0]; 
          tmp2 = incomingMessage.data[1];
          carData.batteryStatus.packCurrent = (tmp1 << 8) | tmp2;   // big endian combination: value = (byte1 << 8) | byte2;

          // pack voltage
          tmp1 = incomingMessage.data[2];
          tmp2 = incomingMessage.data[3];
          carData.batteryStatus.busVoltage = ((tmp1 << 8) | tmp2) / 10;    // big endian combination: value = (byte1 << 8) | byte2;

          // state of charge
          carData.batteryStatus.batteryChargeState = incomingMessage.data[4];
        break;

        // BMS: cell data
        case BMS_CELL_DATA_ADDR:
          carData.batteryStatus.minCellVoltage = incomingMessage.data[0];
          carData.batteryStatus.maxCellVoltage = incomingMessage.data[1];
        break;

        default:
        break;
      }
    }
  }

  // debugging
  if (debugger.debugEnabled) {
    debugger.canReadTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief writes to the CAN bus
 * 
 * @param pvParameters parameters passed to task
 */
void CANWriteTask(void* pvParameters)
{
  // --- send messages --- // 
  can_message_t prechargeOutgoingMessage;

  // --- precharge messages --- // 
  // build rinehart message
  prechargeOutgoingMessage.identifier = RINE_BUS_CONTROL_ADDR;
  prechargeOutgoingMessage.flags = CAN_MSG_FLAG_NONE;
  prechargeOutgoingMessage.data_length_code = 8;

  esp_err_t prechargeMessageResult;

  // build rinehart message based on precharge state
  switch (carData.drivingData.prechargeState) {
    case PRECHARGE_OFF:
      // message is sent to rinehart to turn everything off
      prechargeOutgoingMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeOutgoingMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeOutgoingMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeOutgoingMessage.data[3] = 0x00;          // N/A
      prechargeOutgoingMessage.data[4] = 0x00;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeOutgoingMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeOutgoingMessage.data[6] = 0x00;          // N/A
      prechargeOutgoingMessage.data[7] = 0x00;          // N/A
    break;

    // do precharge
    case PRECHARGE_ON:
      // message is sent to rinehart to turn on precharge relay
      // precharge relay is on relay 1 in Rinehart
      prechargeOutgoingMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeOutgoingMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeOutgoingMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeOutgoingMessage.data[3] = 0x00;          // N/A
      prechargeOutgoingMessage.data[4] = 0x01;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeOutgoingMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeOutgoingMessage.data[6] = 0x00;          // N/A
      prechargeOutgoingMessage.data[7] = 0x00;          // N/A
    break;


    // precharge complete!
    case PRECHARGE_DONE:
      // message is sent to rinehart to turn everything on
      // Keep precharge relay on and turn on main contactor
      prechargeOutgoingMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeOutgoingMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeOutgoingMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeOutgoingMessage.data[3] = 0x00;          // N/A
      prechargeOutgoingMessage.data[4] = 0x03;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeOutgoingMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeOutgoingMessage.data[6] = 0x00;          // N/A
      prechargeOutgoingMessage.data[7] = 0x00;          // N/A
    break;


    // error state
    case PRECHARGE_ERROR:
      // message is sent to rinehart to turn everything off
      prechargeOutgoingMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeOutgoingMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeOutgoingMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeOutgoingMessage.data[3] = 0x00;          // N/A
      prechargeOutgoingMessage.data[4] = 0x00;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeOutgoingMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeOutgoingMessage.data[6] = 0x00;          // N/A
      prechargeOutgoingMessage.data[7] = 0x00;          // N/A
    break;
  }

  // queue rinehart message for transmission
  prechargeMessageResult = can_transmit(&prechargeOutgoingMessage, pdMS_TO_TICKS(CAN_BLOCK_DELAY));


  // --- other outgoing messages --- // 
  can_message_t fcbCtrlOutgoingMessage;
  can_message_t fcbDataOutgoingMessage;

  // build message for FCB 
  fcbCtrlOutgoingMessage.identifier = FCB_CONTROL_ADDR;
  fcbCtrlOutgoingMessage.flags = CAN_MSG_FLAG_NONE;
  fcbCtrlOutgoingMessage.data_length_code = 8;

  fcbCtrlOutgoingMessage.data[0] = carData.drivingData.readyToDrive;
  fcbCtrlOutgoingMessage.data[1] = carData.drivingData.imdFault;
  fcbCtrlOutgoingMessage.data[2] = carData.drivingData.bmsFault;
  fcbCtrlOutgoingMessage.data[3] = 0x00;
  fcbCtrlOutgoingMessage.data[4] = 0x00;
  fcbCtrlOutgoingMessage.data[5] = 0x00;
  fcbCtrlOutgoingMessage.data[6] = 0x00;
  fcbCtrlOutgoingMessage.data[7] = 0x00;

  // queue message for transmission
  esp_err_t fcbCtrlResult = can_transmit(&fcbCtrlOutgoingMessage, pdMS_TO_TICKS(CAN_BLOCK_DELAY));

  // build message for FCB 
  fcbDataOutgoingMessage.identifier = FCB_DATA_ADDR;
  fcbDataOutgoingMessage.flags = CAN_MSG_FLAG_NONE;
  fcbDataOutgoingMessage.data_length_code = 8;

  fcbDataOutgoingMessage.data[0] = carData.sensors.wheelSpeedBR;
  fcbDataOutgoingMessage.data[1] = carData.sensors.wheelSpeedBL;
  fcbDataOutgoingMessage.data[2] = carData.sensors.wheelHeightBR;
  fcbDataOutgoingMessage.data[3] = carData.sensors.wheelHeightBL;
  fcbDataOutgoingMessage.data[4] = 0x00;
  fcbDataOutgoingMessage.data[5] = 0x00;
  fcbDataOutgoingMessage.data[6] = 0x00;
  fcbDataOutgoingMessage.data[7] = 0x00;

  // queue message for transmission
  esp_err_t fcbDataResult = can_transmit(&fcbDataOutgoingMessage, pdMS_TO_TICKS(CAN_BLOCK_DELAY));

  // debugging
  if (debugger.debugEnabled) {
    debugger.prechargeResult = prechargeMessageResult;
    debugger.fcbCtrlResult = fcbCtrlResult;
    debugger.fcbDataResult = fcbDataResult;

    for (int i = 0; i < 8; ++i) {
      debugger.CAN_prechargeOutgoingMessage[i] = prechargeOutgoingMessage.data[i];
    }

    for (int i = 0; i < 8; ++i) {
      debugger.CAN_fcbCtrlOutgoingMessage[i] = fcbCtrlOutgoingMessage.data[i];
    }

    for (int i = 0; i < 8; ++i) {
      debugger.CAN_fcbDataOutgoingMessage[i] = fcbDataOutgoingMessage.data[i];
    }

    debugger.canWriteTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief updates wheel board with information from rcb
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateWCBTask(void* pvParameters) {
  // send message
  const uint8_t wcbAddress[6] = WCB_ESP_NOW_ADDRESS;
  esp_err_t result = esp_now_send(wcbAddress, (uint8_t *) &carData, sizeof(carData));
  
  // debugging 
  if (debugger.debugEnabled) {
    debugger.WCB_updateMessage = carData;
    debugger.WCB_updateResult = result;
    debugger.wcbTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief writes most recent stored data frame to the SD card
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateLoggerTask(void* pvParameters) {
  // inits
  char tmpStr[SD_BUFF_LEN];
  bool logWritten = false;
  float timeStamp = (float)esp_timer_get_time() * 1000000;    // convert uptime from microseconds to seconds

  // open file
  File logFile = SD_MMC.open(sdTrackerFilename, FILE_WRITE);

  // write start of block seperator
  logFile.printf("\n------------------------------\n");

  // write timestamp
  logFile.printf("\t\t[ %.2f seconds ]:\n\n", timeStamp);

  // write data
  logFile.printf("DRIVE STATE: RTD: %d | INVER-EN: %d | MODE: %d\n", carData.drivingData.readyToDrive, carData.drivingData.enableInverter, (int)carData.drivingData.driveMode);

  logFile.printf("FAULTS: IMD: %d | BMS: %d\n", carData.drivingData.imdFault, carData.drivingData.bmsFault);

  logFile.printf("DRIVE STATS: COMM-TORQ: %d | SPEED: %f | DIR: %d\n", carData.drivingData.commandedTorque, carData.drivingData.currentSpeed, carData.drivingData.driveDirection);
  
  logFile.printf("PEDALS: P1: %d | P2: %d | B FRONT: %d | B REAR: %d\n", carData.inputs.pedal0, carData.inputs.pedal1, carData.inputs.brakeFront, carData.inputs.brakeRear);
  
  logFile.printf("WHEEL SPEED: FR: %f | FL: %f | BR: %f | BL: %f\n", carData.sensors.wheelSpeedFR, carData.sensors.wheelSpeedFL, carData.sensors.wheelSpeedBR, carData.sensors.wheelSpeedBL);
  
  logFile.printf("WHEEL HEIGHT: FR: %f | FL: %f | BR: %f | BL: %f\n", carData.sensors.wheelHeightFR, carData.sensors.wheelHeightFL, carData.sensors.wheelHeightBR, carData.sensors.wheelHeightBL);
  
  logFile.printf("STEERING ANGLE: %d\n", carData.sensors.steeringWheelAngle);

  logFile.printf("PACKS: CHARGE: %f | BUS-V: %f | RINE-V: %f\n", carData.batteryStatus.batteryChargeState, carData.batteryStatus.busVoltage, carData.batteryStatus.rinehartVoltage);
  
  logFile.printf("TEMPS: PACK-1: %f | PACK-2: %f | VICORE: %f | PUMP-I: %f | PUMP-O: %f\n", carData.batteryStatus.pack1Temp, carData.batteryStatus.pack2Temp, carData.sensors.vicoreTemp, carData.sensors.pumpTempIn, carData.sensors.pumpTempOut);

  logFile.printf("OUTPUTS: BUZZER: %d | BRAKE: %d | FAN-EN: %d | PUMP-EN: %d\n", carData.outputs.buzzerActive, carData.outputs.brakeLight, carData.outputs.fansActive, carData.outputs.pumpActive);

  // write end of block seperator
  logFile.printf("\n------------------------------\n");

  // close log file
  logFile.close();


  // debugging
  if (debugger.debugEnabled) {
    debugger.loggerTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/*
===============================================================================================
                                    Helpers
===============================================================================================
*/


/**
 * @brief generate a new log filename based on tracker.txt
 * 
 */
void GenerateFilename() {
  // inits
  char logNum[SD_BUFF_LEN];
  char baseName[SD_BUFF_LEN] = "/poop_aids_";

  // strcat(baseName, atoi);    // TODO: fix this 
  strcat(sdTrackerFilename, ".txt");
}


/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/


/**
 * @brief the main loop of the program
 * 
 */
void loop()
{
  // everything is managed by RTOS, so nothing really happens here!
  vTaskDelay(MAIN_LOOP_DELAY);    // prevent watchdog from getting upset

  // debugging
  if (debugger.debugEnabled) {
    PrintDebug();
  }
}


/* 
===============================================================================================
                                    DEBUG FUNCTIONS
================================================================================================
*/


/**
 * @brief some nice in-depth debugging for CAN
 * 
 */
void PrintCANDebug() {
  Serial.printf("\n--- START CAN DEBUG ---\n\n");

  // incoming
  Serial.printf("Incoming Brake Light Status: %s\n\n", carData.outputs.brakeLight ? "on" : "off");

  // sent status
  Serial.printf("Precharge Send Status: 0x%X\n", debugger.prechargeResult);
  Serial.printf("FCB Ctrl Send Status: 0x%X\n", debugger.fcbCtrlResult);
  Serial.printf("FCB Data Send Status: 0x%X\n", debugger.fcbDataResult);

  // messages
  Serial.printf("\nPrecharge Data Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.CAN_prechargeOutgoingMessage[i]);
  }

  Serial.printf("\nFCB Data Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.CAN_fcbDataOutgoingMessage[i]);
  }

  Serial.printf("\nFCB Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.CAN_fcbCtrlOutgoingMessage[i]);
  }

  Serial.printf("\n\n--- END CAN DEBUG ---\n");
}

/**
 * @brief some nice in-depth debugging for WCB updates
 * 
 */
void PrintWCBDebug() {
  Serial.printf("\n--- START WCB DEBUG ---\n\n");

  // send status
  Serial.printf("Message Send Status: %s\n", debugger.WCB_updateResult ? "Success" : "Failed");
  
  Serial.printf("\n\n--- END WCB DEBUG ---\n");
}


void PrintLoggerDebug() {
  Serial.printf("\n--- START LOGGER DEBUG ---\n\n");

  // status
  Serial.printf("Log Written Status: %s\n", debugger.logWritten ? "Success" : "Failed");
  Serial.printf("Timestamp: %f\n", debugger.loggerTimestamp);
  
  Serial.printf("\n\n--- END LOGGER DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for I/O
 * 
 */
void PrintIODebug() {
  Serial.printf("\n--- START I/O DEBUG ---\n\n");

  // precharge
  Serial.printf("Precharge State: ");
  switch (carData.drivingData.prechargeState)
  {
  case PRECHARGE_OFF:
    Serial.printf("OFF\n");
  break;

  case PRECHARGE_ON:
    Serial.printf("ON\n");
  break;

  case PRECHARGE_DONE:
    Serial.printf("DONE\n");
  break;

  case PRECHARGE_ERROR:
    Serial.printf("ERROR\n");
  break;
  }

  // outputs  
  Serial.printf("\nOutputs:\n");
  Serial.printf("Brake Light: %s\n", carData.outputs.brakeLight ? "On" : "Off");
  Serial.printf("Pump: %s\n", carData.outputs.pumpActive ? "On" : "Off");
  Serial.printf("Fans: %s\n", carData.outputs.fansActive ? "On" : "Off");
  Serial.printf("Ready to Drive: %s\n", carData.drivingData.readyToDrive ? "READY" : "DEACTIVAED");
  Serial.printf("Rinehart Voltage: %f\n", carData.batteryStatus.rinehartVoltage);
  Serial.printf("Bus Voltage: %f\n", carData.batteryStatus.busVoltage);

  // inputs
  Serial.printf("\nInputs:\n");

  Serial.printf("BMS Fault: %s\n", carData.drivingData.bmsFault ? "Fault State" : "Cleared");
  Serial.printf("IMD Fault: %s\n", carData.drivingData.imdFault ? "Fault State" : "Cleared");

  Serial.printf("Radiator In Temp: %f\n", carData.sensors.pumpTempIn);
  Serial.printf("Radiator Out Temp: %f\n", carData.sensors.pumpTempOut);
  Serial.printf("Vicore Temp: %f\n", carData.sensors.vicoreTemp);

  Serial.printf("\n\n--- END I/O DEBUG ---\n");
}


/**
 * @brief manages toggle-able debug settings
 * 
 */
void PrintDebug() {
  // CAN
  if (debugger.CAN_debugEnabled) {
      PrintCANDebug();
  }

  // FCB
  if (debugger.FCB_debugEnabled) {
    PrintWCBDebug();
  }

  // Logger
  if (debugger.Logger_debugEnabled) {
    PrintLoggerDebug();
  }

  // I/O
  if (debugger.IO_debugEnabled) {
    PrintIODebug();
  }

  // Scheduler
  if (debugger.scheduler_debugEnable) {
    Serial.printf("sensor: %d | can read: %d | can write: %d | precharge: %d | logger: %d | wcb update: %d\n", debugger.sensorTaskCount, debugger.canReadTaskCount, debugger.canWriteTaskCount, debugger.prechargeTaskCount, debugger.loggerTaskCount, debugger.wcbTaskCount);
  }
}