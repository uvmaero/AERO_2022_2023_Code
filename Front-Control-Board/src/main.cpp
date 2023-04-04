/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief front control board 
 * @version 1.0
 * @date 2023-04-03
 * 
 * @copyright Copyright (c) 2023
 * 
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "driver/can.h"
#include "rtc.h"
#include "rtc_clk_common.h"

#include "LoRaLib.h"

#include <data_types.h>
#include <pin_config.h>

/*
===============================================================================================
                                    Definitions
===============================================================================================
*/

// definitions
#define TIRE_DIAMETER                   20.0        // diameter of the vehicle's tires in inches
#define WHEEL_RPM_CALC_THRESHOLD        100         // the number of times the hall effect sensor is tripped before calculating vehicle speed
#define BRAKE_LIGHT_THRESHOLD           10          // 
#define PEDAL_DEADBAND                  5
#define PEDAL_MIN                       0
#define PEDAL_MAX                       255
#define TORQUE_DEADBAND                 128         // 5% of 2550
#define MAX_TORQUE                      225         // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED 230!!!

// CAN
#define NUM_CAN_READS                   5           // the number of messages to read each time the CAN task is called
#define FCB_CONTROL_ADDR                0x0A
#define FCB_DATA_ADDR                   0x0B
#define RCB_CONTROL_ADDR                0x0C
#define RCB_DATA_ADDR                   0x0D
#define RINE_CONTROL_ADDR               0x0C0
#define RINE_MOTOR_INFO_ADDR            0x0A5
#define RINE_VOLT_INFO_ADDR             0x0A7

// esp now
#define WCB_ADDRESS                     {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x80}
#define RCB_ADDRESS                     {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81}
#define DEVICE_ADDRESS                  {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a}

// tasks & timers
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define CAN_WRITE_INTERVAL              10000       // 0.01 seconds in microseconds
#define ARDAN_UPDATE_INTERVAL           200000      // 0.2 seconds in microseconds
#define ESP_NOW_UPDATE_INTERVAL         200000      // 0.2 seconds in microseconds
#define TASK_STACK_SIZE                 4096        // in bytes

// debug
#define ENABLE_DEBUG                    true        // master debug message control
#if ENABLE_DEBUG
  #define MAIN_LOOP_DELAY               1000        // delay in main loop
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
  .CAN_debugEnabled = true,
  .WCB_debugEnabled = false,
  .IO_debugEnabled = false,
  .scheduler_debugEnable = false,

  // debug data
  .CAN_rineCtrlResult = ESP_OK,
  .CAN_rcbCtrlResult = ESP_OK,
  .CAN_rineCtrlOutgoingMessage = {},
  .CAN_rcbCtrlOutgoingMessage = {},

  .RCB_updateResult = ESP_OK,
  .RCB_updateMessage = {},

  .WCB_updateResult = ESP_OK,
  .WCB_updateMessage = {},

  .IO_data = {},

  .ardanTransmitResult = 0,

  // scheduler data
  .sensorTaskCount = 0,
  .canTaskCount = 0,
  .ardanTaskCount = 0,
  .espnowTaskCount = 0,
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

    .imdFault = false,
    .bmsFault = false,

    .commandedTorque = 0,
    .currentSpeed = 0.0f,
    .driveDirection = true,
    .driveMode = FAST, 
  },

  // Battery Status
  .batteryStatus = {
    .batteryChargeState = 0,
    .busVoltage = 0,
    .rinehartVoltage = 0,
    .pack1Temp = 0.0f,
    .pack2Temp = 0.0f,
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
    .brake0 = 0,
    .brake1 = 0,
    .brakeRegen = 0,
    .coastRegen = 0,
  },

  // Outputs
  .outputs = {
    .buzzerActive = false,
    .buzzerCounter = 0,
    .brakeLight = false,
    .fansActive = false,
    .pumpActive = false,
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
  .peer_addr = WCB_ADDRESS,
  .channel = 1,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};

esp_now_peer_info_t rcbInfo = {
  .peer_addr = RCB_ADDRESS,
  .channel = 1,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};


// CAN
static const can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_500KBITS();
//Filter all other IDs except MSG_ID
static const can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
// {
//   .acceptance_code = (MSG_ID << 21),
//   .acceptance_mask = ~(CAN_STD_ID_MASK << 21),
//   .single_filter = true
// };
//Set to NO_ACK mode due to self testing with single module
static const can_general_config_t can_general_config = CAN_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, CAN_MODE_NO_ACK);


// LoRa Interface
// RFM95 lora = new LoRa();


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void SensorCallback();
void CANCallback();
void ARDANCallback();
void ESPNOWCallback();
void FRWheelSensorCallback();
void FLWheelSensorCallback();

// tasks
void ReadSensorsTask(void* pvParameters);
void UpdateCANTask(void* pvParameters);
void UpdateARDANTask(void* pvParameters);
void UpdateESPNOWTask(void* pvParameters);

// ISRs
void WCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length);
void ReadyToDriveButtonPressed();

// helpers
void GetCommandedTorque();


/*
===============================================================================================
                                            Setup 
===============================================================================================
*/

void setup() {
  // set power configuration
  esp_pm_configure(&power_configuration);

  if (debugger.debugEnabled) {
    // delay startup by 5 seconds
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
    bool ardanActive = false;
    bool wcbActive = false;
    bool rcbActive = false;
  };
  setup setup;


  // -------------------------- initialize GPIO ------------------------------ //
  analogReadResolution(12);
  // analogSetAttenuation();
  
  // inputs
  pinMode(RTD_BUTTON_PIN ,INPUT);

  pinMode(PEDAL_0_PIN, INPUT);
  pinMode(PEDAL_1_PIN, INPUT);

  pinMode(BRAKE_0_PIN, INPUT);
  pinMode(BRAKE_1_PIN, INPUT);
  
  pinMode(WHEEL_SPEED_FL_SENSOR, INPUT);
  pinMode(WHEEL_SPEED_FR_SENSOR, INPUT);

  pinMode(WHEEL_HEIGHT_FL_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_FR_SENSOR, INPUT);

  pinMode(STEERING_WHEEL_POT, INPUT);

  // outputs
  pinMode(RTD_BUTTON_LED_PIN, OUTPUT);
  pinMode(WCB_CONNECTION_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CAN_ENABLE_PIN, OUTPUT);

  // interrupts
  attachInterrupt(RTD_BUTTON_PIN, ReadyToDriveButtonPressed, ONHIGH);

  attachInterrupt(WHEEL_HEIGHT_FR_SENSOR, FRWheelSensorCallback, ONHIGH);
  attachInterrupt(WHEEL_HEIGHT_FL_SENSOR, FLWheelSensorCallback, ONHIGH);

  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //


  // --------------------- initialize CAN Controller -------------------------- //
  digitalWrite(CAN_ENABLE_PIN, LOW);

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

    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    Serial.print("WIFI MAC: "); Serial.println(WiFi.macAddress());
    Serial.print("WIFI CHANNEL: "); Serial.println(WiFi.channel());

    if (esp_now_init() == ESP_OK) {
      Serial.printf("ESP-NOW INIT [ SUCCESS ]\n");

      // add peers
      esp_err_t rcbResult = esp_now_add_peer(&rcbInfo);
      esp_err_t wcbResult = esp_now_add_peer(&wcbInfo);

      if (rcbResult == ESP_OK && wcbResult == ESP_OK) {
        Serial.printf("ESP-NOW ADD PEERS [ SUCCESS ]\n");

        setup.rcbActive = true;
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
    Serial.printf("WIFI INIT [ FAILED ]\n");
  }

  // ------------------------------------------------------------------------ //


  // ------------------- initialize ARDAN Connection ------------------------ //
  // LoRa Interface

  // // init lora
  // if (lora.begin() == ERR_NONE) {
  //   Serial.printf("ARDAN INIT [SUCCESSS ]\n");

  //   // set the sync word so the car and monitoring station can communicate
  //   lora.setSyncWord(0xA1);         // the channel to be transmitting on (range: 0x00 - 0xFF)

  // setup.ardanActive = true;
  // }
  // else { 
  //   Serial.printf("ARDAN INIT [ FAILED ]\n");
  // }send message
  // ------------------------------------------------------------------------- //


  // ---------------------- initialize timer interrupts --------------------- //
  // timer 1 - Read Sensors 
  // Create semaphore to inform us when the timer has fired
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &SensorCallback, true);
  timerAlarmWrite(timer1, SENSOR_POLL_INTERVAL, true);

  // timer 2 - CAN Update
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &CANCallback, true);
  timerAlarmWrite(timer2, CAN_WRITE_INTERVAL, true);

  // timer 3 - ARDAN Update
  timer3 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer3, &ARDANCallback, true);
  timerAlarmWrite(timer3, ARDAN_UPDATE_INTERVAL, true);

  // timer 4 - ESP-NOW Update
  timer4 = timerBegin(3, 80, true);
  timerAttachInterrupt(timer4, &ESPNOWCallback, true);
  timerAlarmWrite(timer4, ESP_NOW_UPDATE_INTERVAL, true);

  // start timers
  if (setup.ioActive)
    timerAlarmEnable(timer1);
  if (setup.canActive)
    timerAlarmEnable(timer2);
  if (setup.ardanActive)
    timerAlarmEnable(timer3);
  if (setup.wcbActive && setup.rcbActive)
    timerAlarmEnable(timer4);

  Serial.printf("SENSOR TASK STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("CAN TASK STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("ARDAN TASK STATUS: %s\n", timerAlarmEnabled(timer3) ? "RUNNING" : "DISABLED");
  Serial.printf("ESP-NOW TASK STATUS: %s\n", timerAlarmEnabled(timer4) ? "RUNNING" : "DISABLED");
  // ----------------------------------------------------------------------------------------- //


  // ------------------- End Setup Section in Serial Monitor --------------------------------- //
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

  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(ReadSensorsTask, "Poll-Senser-Data", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
  
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

  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateCANTask, "CAN-Update", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);

  portEXIT_CRITICAL_ISR(&timerMux);
  
  return;
}


/**
 * @brief callback function for creating a new ARDAN Update task
 * 
 * @param args arguments to be passed to the task
 */
void ARDANCallback() {
  portENTER_CRITICAL_ISR(&timerMux);

  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateARDANTask, "ARDAN-Update", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);

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

  // queue wcb update
  static uint8_t ucParameterToPassWCB;
  TaskHandle_t xHandleWCB = NULL;
  xTaskCreate(UpdateESPNOWTask, "ESP-NOW-Update", TASK_STACK_SIZE, &ucParameterToPassWCB, tskIDLE_PRIORITY, &xHandleWCB);

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
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
  portENTER_CRITICAL_ISR(&timerMux);

  // copy data to the wcbData struct 
  memcpy((uint8_t *) &carData, incomingData, sizeof(carData));

  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for when the hall effect sensor fires on front right wheel
 * 
 * @param args arguments to be passed to the task
 */
void FRWheelSensorCallback() {
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
 * @param args arguments to be passed to the task
 */
void FLWheelSensorCallback() {
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


/**
 * @brief handle the ready to drive button press event
 * 
 * @param args arguments to be passed to the task
 */
void ReadyToDriveButtonPressed() {
  portENTER_CRITICAL_ISR(&timerMux);

  if (carData.drivingData.readyToDrive) {
    // turn on buzzer to indicate TSV is live
    carData.outputs.buzzerActive = true;
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

  // get pedal positions
  float tmpPedal0 = analogRead(PEDAL_0_PIN);
  carData.inputs.pedal0 = map(tmpPedal0, 575, 2810, 0, 255);   // starting min and max values must be found via testing!!! (0.59V - 2.75V)
  
  if (carData.inputs.pedal0 > 255) {
    carData.inputs.pedal0 = 255;
  }

  float tmpPedal1 = analogRead(PEDAL_1_PIN);
  carData.inputs.pedal1 = map(tmpPedal1, 250, 1400, 0, 255);   // starting min and max values must be found via testing!!! (0.29V - 1.379V)

  if (carData.inputs.pedal1 > 255) {
    carData.inputs.pedal1 = 255;
  }

  // Calculate commanded torque
  GetCommandedTorque();


  // get brake positions
  float tmpBrake0 = analogRead(BRAKE_0_PIN);
  carData.inputs.brake0 = map(tmpBrake0, 0, 1024, 0, 255);   // starting min and max values must be found via testing!!! 

  float tmpBrake1 = analogRead(BRAKE_1_PIN);
  carData.inputs.brake1 = map(tmpBrake1, 0, 1024, 0, 255);   // starting min and max values must be found via testing!!!

  // brake light logic 
  int brakeAverage = (carData.inputs.brake0 + carData.inputs.brake1) / 2;
  if (brakeAverage >= BRAKE_LIGHT_THRESHOLD) {
    carData.outputs.brakeLight = true;      // turn it on 
  }

  else {
    carData.outputs.brakeLight = false;     // turn it off
  }


  // update wheel ride height values
  carData.sensors.wheelHeightFR = analogRead(WHEEL_HEIGHT_FR_SENSOR);
  carData.sensors.wheelHeightFL = analogRead(WHEEL_HEIGHT_FL_SENSOR);

  // update steering wheel position
  carData.sensors.steeringWheelAngle = analogRead(STEERING_WHEEL_POT);

  // buzzer logic
  if (carData.outputs.buzzerActive)
  {
    digitalWrite(BUZZER_PIN, carData.outputs.buzzerActive);
    carData.outputs.buzzerCounter++;

    if (carData.outputs.buzzerCounter >= (2 * (SENSOR_POLL_INTERVAL / 10000)))    // convert to activations per second and multiply by 2
    {
      // update buzzer state and turn off the buzzer
      carData.outputs.buzzerActive = false;
      digitalWrite(BUZZER_PIN, carData.outputs.buzzerActive);

      carData.outputs.buzzerCounter = 0;                        // reset buzzer count
      carData.drivingData.enableInverter = true;                // enable the inverter so that we can tell rinehart to turn inverter on
    }
  }
  

  // debugging
  if (debugger.debugEnabled) {
    debugger.IO_data = carData;
    debugger.sensorTaskCount++;
  }

  // turn wifi back on to re-enable esp-now connection to wheel board
  esp_wifi_start();

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief reads and writes to the CAN bus
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateCANTask(void* pvParameters)
{
  // inits
  can_message_t incomingMessage;
  int id;

  // --- receive messages --- //
  // check for new messages in the CAN buffer
  for (int i = 0; i < NUM_CAN_READS; ++i) {
    if (can_receive(&incomingMessage, pdMS_TO_TICKS(10)) == ESP_OK) { // if there are messages to be read
      id = incomingMessage.identifier;
      
      // parse out data
      switch (id) {
        case RCB_CONTROL_ADDR:
          incomingMessage.data[0] = carData.drivingData.readyToDrive;
          incomingMessage.data[1] = carData.drivingData.imdFault;
          incomingMessage.data[2] = carData.drivingData.bmsFault;
        break;

        case RCB_DATA_ADDR:
          incomingMessage.data[0] = carData.sensors.wheelSpeedBR;
          incomingMessage.data[1] = carData.sensors.wheelSpeedBL;
          incomingMessage.data[2] = carData.sensors.wheelHeightBR;
          incomingMessage.data[3] = carData.sensors.wheelHeightBL;
        break;

        default:
        break;
      }
    }
  }

  // --- send messages --- // 
  bool sentStatus = false;  

  can_message_t outgoingMessage;
  outgoingMessage.identifier = RINE_CONTROL_ADDR;
  outgoingMessage.flags = CAN_MSG_FLAG_NONE;
  outgoingMessage.data_length_code = 8;

  // build message
  uint8_t firstTorqByte = carData.drivingData.commandedTorque & 0xFF;
  uint8_t secondTorqByte = carData.drivingData.commandedTorque >> 8;

  outgoingMessage.data[0] = firstTorqByte;                                    // commanded torque is sent across two bytes
  outgoingMessage.data[1] = secondTorqByte;
  outgoingMessage.data[2] = 0x00;                                             // speed command NOT USING
  outgoingMessage.data[3] = 0x00;                                             // speed command NOT USING
  outgoingMessage.data[4] = (uint8_t)(carData.drivingData.driveDirection);    // 1: forward | 0: reverse (we run in reverse!)
  outgoingMessage.data[5] = (uint8_t)(carData.drivingData.enableInverter);    // 
  outgoingMessage.data[6] = (uint8_t)MAX_TORQUE;                              // this is the max torque value that we are establishing t
  outgoingMessage.data[7] = 0x00;                                             // i think this one is min torque or it does nothing

  // queue message for transmission
  esp_err_t rineCtrlResult = can_transmit(&outgoingMessage, pdMS_TO_TICKS(10));


  // setup RCB message
  outgoingMessage.identifier = RCB_CONTROL_ADDR;
  outgoingMessage.flags = CAN_MSG_FLAG_NONE;
  outgoingMessage.data_length_code = 8;

  // build message for RCB - control
  outgoingMessage.data[0] = (uint8_t)carData.outputs.brakeLight;
  outgoingMessage.data[1] = 0x01;
  outgoingMessage.data[2] = 0x02;
  outgoingMessage.data[3] = 0x03;
  outgoingMessage.data[4] = 0x04;
  outgoingMessage.data[5] = 0x05;
  outgoingMessage.data[6] = 0x06;
  outgoingMessage.data[7] = 0x07;

  // queue message for transmission
  esp_err_t rcbCtrlResult = can_transmit(&outgoingMessage, pdMS_TO_TICKS(10));

  // debugging
  if (debugger.debugEnabled) {
    debugger.CAN_rineCtrlResult = rineCtrlResult;
    debugger.CAN_rcbCtrlResult = rcbCtrlResult;

    for (int i = 0; i < 8; ++i) {
      debugger.CAN_rcbCtrlOutgoingMessage[i] = outgoingMessage.data[i];
    }

    debugger.canTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief updates WCB with car data
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateESPNOWTask(void* pvParameters)
{
  // send message
  const uint8_t wcbAddress[6] = WCB_ADDRESS;
  const uint8_t rcbAddress[6] = RCB_ADDRESS;
  esp_err_t wcbResult = esp_now_send(wcbAddress, (uint8_t *) &carData, sizeof(carData));
  esp_err_t rcbResult = esp_now_send(rcbAddress, (uint8_t *) &carData, sizeof(carData));

  // debugging 
  if (debugger.debugEnabled) {
    debugger.WCB_updateMessage = carData;
    debugger.WCB_updateResult = wcbResult;

    debugger.RCB_updateMessage = carData;
    debugger.RCB_updateResult = rcbResult;
    debugger.espnowTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief updates the ARDAN 
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateARDANTask(void* pvParameters)
{
  // // convert car data to byte array
  // char* carDataBytes = reinterpret_cast<char*>(&carData);

  // // send LoRa update
  // int result = lora.transmit(carDataBytes, sizeof(carDataBytes));

  // // debugging
  // if (debugger.debugEnabled) {
  //   // debugger.ardanTransmitResult = result;
  //   debugger.ardanTaskCount++;
  // }

  // end task
  vTaskDelete(NULL);
}


/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/


/**
 * @brief 
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


/**
 * @brief Get the Commanded Torque from pedal values
 */
void GetCommandedTorque()
{
  // get the pedal average
  int pedalAverage = (carData.inputs.pedal0 + carData.inputs.pedal1) / 2;

  // drive mode logic
  switch (carData.drivingData.driveMode)
  {
    case SLOW:  // runs at 50% power
      carData.drivingData.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.50);
    break;

    case ECO:   // runs at 75% power
      carData.drivingData.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.75);
    break;

    case FAST:  // runs at 100% power
      carData.drivingData.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10));
    break;
    
    // error state, set the mode to ECO
    default:
      // set the state to ECO for next time
      carData.drivingData.driveMode = ECO;

      // we don't want to send a torque if we are in an undefined state
      carData.drivingData.commandedTorque = 0;
    break;
  }

  // --- safety checks --- //

  // pedal difference 
  int pedalDifference = carData.inputs.pedal0 - carData.inputs.pedal1;
  if (_abs(pedalDifference > (PEDAL_MAX * 0.1))) {
    carData.drivingData.commandedTorque = 0;
  }
  
  // buffer overflow / too much torque somehow
  if (carData.drivingData.commandedTorque > (MAX_TORQUE * 10)) {
    carData.drivingData.commandedTorque = 0;
  }

  // for throttle safety, we will have a deadband
  if (carData.drivingData.commandedTorque <= TORQUE_DEADBAND)   // if less than 5% power is requested, just call it 0
  {
    carData.drivingData.commandedTorque = 0;
  }

  // // check if ready to drive
  // if (!carData.drivingData.readyToDrive) {
  //   carData.drivingData.commandedTorque = 0;    // if not ready to drive then block all torque
  // }
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
  Serial.printf("\n--- START CAN DEBUG ---\n");

  // incoming data
  Serial.printf("Incoming RTD Status: %s\n", carData.drivingData.readyToDrive ? "true" : "false");
  Serial.printf("Incoming IMD Fault Status: %s\n", carData.drivingData.imdFault ? "cleared" : "fault state");
  Serial.printf("Incoming BMS Fault Status: %s\n", carData.drivingData.bmsFault ? "cleared" : "fault state");

  // sent status
  Serial.printf("Rine Ctrl Send Status: 0x%X\n", debugger.CAN_rineCtrlResult);
  Serial.printf("RCB Ctrl Send Status: 0x%X\n", debugger.CAN_rcbCtrlResult);

  // messages
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.CAN_rineCtrlOutgoingMessage[i]);
  }

  Serial.printf("\n");

  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.CAN_rcbCtrlOutgoingMessage[i]);
  }

  Serial.printf("\n");

  Serial.printf("\n--- END CAN DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for WCB updates
 * 
 */
void PrintWCBDebug() {
  Serial.printf("\n--- START WCB DEBUG ---\n");

  // send status
  Serial.printf("WCB ESP-NOW Update: %s\n", debugger.WCB_updateResult ? "Success" : "Failed");


  Serial.printf("\n--- END WCB DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for I/O
 * 
 */
void PrintIODebug() {
  Serial.printf("\n--- START I/O DEBUG ---\n");

  // INPUTS
  // pedal 0 & 1
  Serial.printf("Pedal 0: %d\tPedal 1: %d\n", debugger.IO_data.inputs.pedal0, debugger.IO_data.inputs.pedal1);	

  // brake 0 & 1
  Serial.printf("Brake 0: %d\tBrake 1: %d\n", debugger.IO_data.inputs.brake0, debugger.IO_data.inputs.brake1);

  // brake regen
  Serial.printf("Brake Regen: %d\n", debugger.IO_data.inputs.brakeRegen);

  // coast regen
  Serial.printf("Coast Regen: %d\n", debugger.IO_data.inputs.coastRegen);

  // OUTPUTS
  // buzzer status
  Serial.printf("Buzzer Status: %s, Buzzer Counter: %d\n", debugger.IO_data.outputs.buzzerActive ? "On" : "Off", debugger.IO_data.outputs.buzzerCounter);

  Serial.printf("Commanded Torque: %d\n", carData.drivingData.commandedTorque);
  
  Serial.printf("Drive Mode: %d\n", (int)carData.drivingData.driveMode);

  Serial.printf("\n--- END I/O DEBUG ---\n");
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

  // WCB
  if (debugger.WCB_debugEnabled) {
    PrintWCBDebug();
  }

  // I/O
  if (debugger.IO_debugEnabled) {
    PrintIODebug();
  }

  // Scheduler
  if (debugger.scheduler_debugEnable) {
    Serial.printf("sensor: %d | can: %d | wcb: %d | rcb: %d | ardan: %d\n", debugger.sensorTaskCount, debugger.canTaskCount,
    debugger.espnowTaskCount, debugger.espnowTaskCount, 
    debugger.ardanTaskCount);
  }
}