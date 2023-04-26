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
#define BRAKE_LIGHT_THRESHOLD           20          // the threshold that must be crossed for the brake to be considered active
#define PEDAL_MIN                       0           // minimum value the pedals can read as
#define PEDAL_MAX                       255         // maximum value a pedal can read as
#define PEDAL_DEADBAND                  15          // ~5% of PEDAL_MAX
#define MAX_TORQUE                      225         // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED 230!!!
#define MIN_BUS_VOLTAGE                 150         // min bus voltage

// CAN
#define NUM_CAN_READS                   25          // the number of messages to read each time the CAN task is called
#define FCB_CONTROL_ADDR                0x00A       // critical data for FCB
#define FCB_DATA_ADDR                   0x00B       // sensor information for FCB
#define RCB_CONTROL_ADDR                0x00C       // critical data for RCB
#define RCB_DATA_ADDR                   0x00D       // sensor information for RCB 
#define RINE_CONTROL_ADDR               0x0C0       // motor command address 
#define RINE_MOTOR_INFO_ADDR            0x0A5       // get rinehart motor infromation 
#define RINE_VOLT_INFO_ADDR             0x0A7       // get rinehart voltage information

// ESP-NOW
#define WCB_ESP_NOW_ADDRESS             {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x80}
#define FCB_ESP_NOW_ADDRESS             {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81}
#define RCB_ESP_NOW_ADDRESS             {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x82}

// tasks & timers
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define CAN_UPDATE_INTERVAL             100000      // 0.1 seconds in microseconds
#define ARDAN_UPDATE_INTERVAL           200000      // 0.2 seconds in microseconds
#define ESP_NOW_UPDATE_INTERVAL         200000      // 0.2 seconds in microseconds
#define TASK_STACK_SIZE                 4096        // in bytes
#define CAN_BLOCK_DELAY                 100         // time to block to complete function call in FreeRTOS ticks (milliseconds)

// debug
#define ENABLE_DEBUG                    false       // master debug message control
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
  .CAN_debugEnabled = false,
  .WCB_debugEnabled = false,
  .IO_debugEnabled = true,
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

    .imdFault = true,
    .bmsFault = true,

    .commandedTorque = 0,
    .currentSpeed = 0.0f,
    .driveDirection = false,
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
  .peer_addr = WCB_ESP_NOW_ADDRESS,
  .channel = 1,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};

esp_now_peer_info_t rcbInfo = {
  .peer_addr = RCB_ESP_NOW_ADDRESS,
  .channel = 1,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};


// CAN
static const can_general_config_t can_general_config = CAN_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, CAN_MODE_NORMAL);
static const can_timing_config_t can_timing_config = CAN_TIMING_CONFIG_500KBITS();
static const can_filter_config_t can_filter_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
// {
//   .acceptance_code = (MSG_ID << 21),
//   .acceptance_mask = ~(CAN_STD_ID_MASK << 21),
//   .single_filter = true
// };


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

// helpers
void GetCommandedTorque();
uint16_t CalculateThrottleResponse(uint16_t value);


/*
===============================================================================================
                                            Setup 
===============================================================================================
*/


void setup() {
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
    bool ardanActive = false;
    bool wcbActive = false;
    bool rcbActive = false;
  };
  setup setup;


  // -------------------------- initialize GPIO ------------------------------ //
  analogReadResolution(12);
  // analogSetAttenuation();
  
  // inputs
  pinMode(RTD_BUTTON_PIN, INPUT_PULLUP);

  pinMode(PEDAL_0_PIN, INPUT);
  pinMode(PEDAL_1_PIN, INPUT);

  pinMode(BRAKE_PIN, INPUT);
  
  pinMode(WHEEL_SPEED_FL_SENSOR, INPUT);
  pinMode(WHEEL_SPEED_FR_SENSOR, INPUT);

  pinMode(WHEEL_HEIGHT_FL_SENSOR, INPUT);
  pinMode(WHEEL_HEIGHT_FR_SENSOR, INPUT);

  pinMode(STEERING_WHEEL_POT, INPUT);

  // outputs
  pinMode(RTD_LED_PIN, OUTPUT);
  pinMode(WCB_CONNECTION_LED, OUTPUT);
  pinMode(BMS_LED_PIN, OUTPUT);
  pinMode(IMD_LED_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CAN_ENABLE_PIN, OUTPUT);

  // interrupts
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
      can_reconfigure_alerts(CAN_ALERT_ALL, NULL);

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
    const uint8_t fcbAddress[6] = FCB_ESP_NOW_ADDRESS;
    if (esp_wifi_set_mac(WIFI_IF_STA, fcbAddress) == ESP_OK) {
      Serial.printf("WIFI SET MAC [ SUCCESS ]\n");
    
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
      Serial.printf("WIFI SET MAC [ FAILED ]\n");
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
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &SensorCallback, true);
  timerAlarmWrite(timer1, SENSOR_POLL_INTERVAL, true);

  // timer 2 - CAN Update
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &CANCallback, true);
  timerAlarmWrite(timer2, CAN_UPDATE_INTERVAL, true);

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
  // ----------------------------------------------------------------------------------------- //


  // ------------------------------- Scheduler & Task Status --------------------------------- //
  Serial.printf("SENSOR TASK STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("CAN TASK STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("ARDAN TASK STATUS: %s\n", timerAlarmEnabled(timer3) ? "RUNNING" : "DISABLED");
  Serial.printf("ESP-NOW TASK STATUS: %s\n", timerAlarmEnabled(timer4) ? "RUNNING" : "DISABLED");
  
  // scheduler status
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
void SensorCallback() 
{
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
void CANCallback() 
{
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
void ARDANCallback() 
{
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
void ESPNOWCallback() 
{
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
void FRWheelSensorCallback() 
{
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
void FLWheelSensorCallback() 
{
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

  // get brake position
  float tmpBrake = analogReadMilliVolts(BRAKE_PIN);
  carData.inputs.brakeFront = map(tmpBrake, 260, 855, PEDAL_MIN, PEDAL_MAX);  // (0.26V - 0.855V) | values found via testing

  // read pedal potentiometer 1
  uint16_t tmpPedal1 = analogReadMilliVolts(PEDAL_1_PIN);
  tmpPedal1 = map(tmpPedal1, 290, 1375, PEDAL_MIN, PEDAL_MAX);                // (0.29V - 1.379V) | values found via testing
  carData.inputs.pedal1 = CalculateThrottleResponse(tmpPedal1);

  // wcb connection LED would also be in here

  // turn wifi back on to re-enable esp-now connection to wheel board
  esp_wifi_start();

  // get pedal positions
  uint16_t tmpPedal0 = analogReadMilliVolts(PEDAL_0_PIN);                     //  (0.59V - 2.75V) | values found via testing
  tmpPedal0 = map(tmpPedal0, 575, 2810, PEDAL_MIN, PEDAL_MAX);                // remap throttle response to 0 - 255 range
  carData.inputs.pedal0 = CalculateThrottleResponse(tmpPedal0);

  // brake light logic 
  if (carData.inputs.brakeFront >= BRAKE_LIGHT_THRESHOLD) {
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
    digitalWrite(BUZZER_PIN, HIGH);
    carData.outputs.buzzerCounter++;

    if (carData.outputs.buzzerCounter >= (2 * (SENSOR_POLL_INTERVAL / 10000)))    // convert to activations per second and multiply by 2
    {
      // update buzzer state and turn off the buzzer
      carData.outputs.buzzerActive = false;
      carData.outputs.buzzerCounter = 0;                        // reset buzzer count
      digitalWrite(BUZZER_PIN, LOW);

      carData.drivingData.enableInverter = true;                // enable the inverter so that we can tell rinehart to turn inverter on
    }
  }

  // ready to drive button
  if (digitalRead(RTD_BUTTON_PIN) == LOW) {
    if (carData.drivingData.readyToDrive) {
      // turn on buzzer to indicate TSV is live
      carData.outputs.buzzerActive = true;
    }
  }

  // Ready to Drive LED
  if (carData.drivingData.readyToDrive) {
    digitalWrite(RTD_LED_PIN, HIGH);
  }
  else {
    digitalWrite(RTD_LED_PIN, LOW);
  }

  // BMS fault LED
  if (!carData.drivingData.bmsFault) {
    digitalWrite(BMS_LED_PIN, LOW);
  }
  else {
    digitalWrite(BMS_LED_PIN, HIGH);
  }

  // IMD fault LED
  if (!carData.drivingData.imdFault) {
    digitalWrite(IMD_LED_PIN, LOW);
  }
  else {
    digitalWrite(IMD_LED_PIN, HIGH);
  }


  // calculate commanded torque
  GetCommandedTorque();


  // debugging
  if (debugger.debugEnabled) {
    debugger.IO_data = carData;
    debugger.sensorTaskCount++;
  }

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
      id = incomingMessage.identifier;
      
      // parse out data
      switch (id) {
        // Rinehart: voltage information
        case RINE_VOLT_INFO_ADDR:
          // rinehart voltage is spread across the first 2 bytes
          tmp1 = incomingMessage.data[0];
          tmp2 = incomingMessage.data[1];

          // combine the first two bytes and assign that to the rinehart voltage
          carData.batteryStatus.rinehartVoltage = (tmp2 << 8) | tmp1;   // little endian combination: value = (byte2 << 8) | byte1;
        break;

        case FCB_CONTROL_ADDR:
          carData.drivingData.readyToDrive =  incomingMessage.data[0];
          carData.drivingData.imdFault = incomingMessage.data[1];
          carData.drivingData.bmsFault = incomingMessage.data[2];
        break;

        case FCB_DATA_ADDR:
          carData.sensors.wheelSpeedBR = incomingMessage.data[0];
          carData.sensors.wheelSpeedBL = incomingMessage.data[1];
          carData.sensors.wheelHeightBR = incomingMessage.data[2];
          carData.sensors.wheelHeightBL = incomingMessage.data[3];
        break;

        default:
        break;
      }
    }
  }

  // --- send messages --- // 
  bool sentStatus = false;  

  can_message_t rineOutgoingMessage;
  rineOutgoingMessage.identifier = RINE_CONTROL_ADDR;
  rineOutgoingMessage.flags = CAN_MSG_FLAG_NONE;
  rineOutgoingMessage.data_length_code = 8;

  // build message
  rineOutgoingMessage.data[0] = carData.drivingData.commandedTorque & 0xFF;       // commanded torque is sent across two bytes
  rineOutgoingMessage.data[1] = carData.drivingData.commandedTorque >> 8;
  rineOutgoingMessage.data[2] = 0x00;                                             // speed command NOT USING
  rineOutgoingMessage.data[3] = 0x00;                                             // speed command NOT USING
  rineOutgoingMessage.data[4] = (uint8_t)(carData.drivingData.driveDirection);    // 1: forward | 0: reverse (we run in reverse!)
  rineOutgoingMessage.data[5] = (uint8_t)(carData.drivingData.enableInverter);    // enable inverter command
  rineOutgoingMessage.data[6] = (MAX_TORQUE * 10) & 0xFF;                         // this is the max torque value that rinehart will push
  rineOutgoingMessage.data[7] = (MAX_TORQUE * 10) >> 8;                           // rinehart expects 10x value spread across 2 bytes

  // queue message for transmission
  esp_err_t rineCtrlResult = can_transmit(&rineOutgoingMessage, pdMS_TO_TICKS(CAN_BLOCK_DELAY));


  // setup RCB message
  can_message_t rcbOutgoingMessage;
  rcbOutgoingMessage.identifier = RCB_CONTROL_ADDR;
  rcbOutgoingMessage.flags = CAN_MSG_FLAG_NONE;
  rcbOutgoingMessage.data_length_code = 8;

  // build message for RCB - control
  rcbOutgoingMessage.data[0] = (uint8_t)carData.outputs.brakeLight;
  rcbOutgoingMessage.data[1] = 0x00;
  rcbOutgoingMessage.data[2] = 0x02;
  rcbOutgoingMessage.data[3] = 0x03;
  rcbOutgoingMessage.data[4] = 0x04;
  rcbOutgoingMessage.data[5] = 0x05;
  rcbOutgoingMessage.data[6] = 0x06;
  rcbOutgoingMessage.data[7] = 0x07;

  // queue message for transmission
  esp_err_t rcbCtrlResult = can_transmit(&rcbOutgoingMessage, pdMS_TO_TICKS(CAN_BLOCK_DELAY));

  // debugging
  if (debugger.debugEnabled) {
    debugger.CAN_rineCtrlResult = rineCtrlResult;
    debugger.CAN_rcbCtrlResult = rcbCtrlResult;

    for (int i = 0; i < 8; ++i) {
      debugger.CAN_rineCtrlOutgoingMessage[i] = rineOutgoingMessage.data[i];
    }

    for (int i = 0; i < 8; ++i) {
      debugger.CAN_rcbCtrlOutgoingMessage[i] = rcbOutgoingMessage.data[i];
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
  const uint8_t wcbAddress[6] = WCB_ESP_NOW_ADDRESS;
  const uint8_t rcbAddress[6] = RCB_ESP_NOW_ADDRESS;
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

  // drive mode logic (values are 10x because that is the format for Rinehart)
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

      // we don't want to send a torque command if we were in an undefined state
      carData.drivingData.commandedTorque = 0;
    break;
  }

  // --- safety checks --- //

  // rinehart voltage check
  if (carData.batteryStatus.rinehartVoltage < MIN_BUS_VOLTAGE) {
    carData.drivingData.enableInverter = false;
  }

  // pedal difference 
  int pedalDifference = carData.inputs.pedal0 - carData.inputs.pedal1;
  if (_abs(pedalDifference > (PEDAL_MAX * 0.15))) {
    carData.drivingData.commandedTorque = 0;
  }
  
  // buffer overflow / too much torque somehow
  if ((carData.drivingData.commandedTorque > (MAX_TORQUE * 10)) || (carData.drivingData.commandedTorque < 0)) {
    carData.drivingData.commandedTorque = 0;
  }

  // if brake is engaged
  if (carData.outputs.brakeLight) {
    carData.drivingData.commandedTorque = 0;
  }

  // check if ready to drive
  if (!carData.drivingData.readyToDrive) {
    carData.drivingData.commandedTorque = 0;      // if not ready to drive then block all torque
  }
} 


/**
 * @brief calculate throttle response of pedal
 * 
 * @param value the raw pedal value
 * @return uint16_t the commanded torque value
 */
uint16_t CalculateThrottleResponse(uint16_t value) 
{
  // inits
  float calculatedResponse = 0;
  float exponent = 0;

  // check for buffer overflow
  if (value > 255 || value < 0) {
    return 0;
  }

  // account for deadband
  if (value < PEDAL_DEADBAND) {
    return 0;
  }

  // determine response curve based on drive mode
  switch (carData.drivingData.driveMode)
  {
    case SLOW:
      exponent = 4.0;
      calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / MAX_TORQUE);
    break;

    case ECO:
      exponent = 2.0;
      calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / MAX_TORQUE);
    break;

    case FAST:
      exponent = 0.75;
      calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / MAX_TORQUE);
    break;
    
    // if we are in an undefined state, pedals should do nothing
    default:
      return 0;
    break;
  }

  // cast final calculated response to an int
  return (int)calculatedResponse;
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
  // bus alerts            
  Serial.printf("CAN BUS Alerts:\n");
  uint32_t alerts;
  can_read_alerts(&alerts, pdMS_TO_TICKS(100));
  if (alerts & CAN_ALERT_TX_SUCCESS) {
    Serial.printf("CAN ALERT: TX Success\n");
  }
  if (alerts & CAN_ALERT_TX_FAILED) {
    Serial.printf("CAN ALERT: TX Failed\n");
  }
  if (alerts & CAN_ALERT_RX_QUEUE_FULL) {
    Serial.printf("CAN ALERT: RX Queue Full\n");
  }
  if (alerts & CAN_ALERT_ABOVE_ERR_WARN) {
    Serial.printf("CAN ALERT: Surpassed Error Warning Limit\n");
  }
  if (alerts & CAN_ALERT_ERR_PASS) {
    Serial.printf("CAN ALERT: Entered Error Passive state\n");
  }
  if (alerts & CAN_ALERT_BUS_OFF) {
    Serial.printf("CAN ALERT: Bus Off\n");
  }

  Serial.printf("\n");

  // incoming data
  Serial.printf("Incoming RTD Status: %s\n", carData.drivingData.readyToDrive ? "true" : "false");
  Serial.printf("Incoming IMD Fault Status: %s\n", carData.drivingData.imdFault ? "cleared" : "fault state");
  Serial.printf("Incoming BMS Fault Status: %s\n", carData.drivingData.bmsFault ? "fault state" : "cleared");

  // sent status
  Serial.printf("Rine Ctrl Send Status: 0x%X\n", debugger.CAN_rineCtrlResult);
  Serial.printf("RCB Ctrl Send Status: 0x%X\n", debugger.CAN_rcbCtrlResult);

  // messages
  Serial.printf("\n");
  Serial.printf("Rine Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.CAN_rineCtrlOutgoingMessage[i]);
  }

  Serial.printf("\n");

  Serial.printf("RCB Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.CAN_rcbCtrlOutgoingMessage[i]);
  }

  Serial.printf("\n\n--- END CAN DEBUG ---\n");
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
  Serial.printf("Brake Front: %d\tBrake Rear: %d\n", debugger.IO_data.inputs.brakeFront, debugger.IO_data.inputs.brakeRear);

  // brake regen
  Serial.printf("Brake Regen: %d\n", debugger.IO_data.inputs.brakeRegen);

  // coast regen
  Serial.printf("Coast Regen: %d\n", debugger.IO_data.inputs.coastRegen);

  // faults
  Serial.printf("Faults: IMD: %d | BMS: %d\n", carData.drivingData.imdFault, carData.drivingData.bmsFault);

  // rtd
  Serial.printf("Ready to Drive: %s\n", carData.drivingData.readyToDrive ? "READY" : "DEACTIVATED");

  // inverter
  Serial.printf("Inverter Enable: %s\n", carData.drivingData.enableInverter ? "ENABLED" : "DISABLED");

  // OUTPUTS
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