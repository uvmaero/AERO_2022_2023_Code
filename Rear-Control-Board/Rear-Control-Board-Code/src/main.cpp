/**
 * @file main.cpp
 * @author Dominic Gasperini - UVM '23
 * @brief this drives the rear control board on clean speed 5.5
 * @version 0.9
 * @date 2023-02-21
 */

/*
===============================================================================================
                                    Includes 
===============================================================================================
*/
// standard includes 
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_now.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "rtc.h"
#include "rtc_clk_common.h"
#include <esp_timer.h>
#include <esp_wifi.h>
#include "esp_netif.h"
#include "driver/adc.h"
#include "driver/can.h"
#include "esp_adc_cal.h"

// custom includes
#include "pin_config.h"


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/
// GPIO
#define GPIO_INPUT_PIN_SELECT           1       

// definitions
#define TIRE_DIAMETER                   20.0        // diameter of the vehicle's tires in inches
#define WHEEL_RPM_CALC_THRESHOLD        100         // the number of times the hall effect sensor is tripped before calculating vehicle speed

// tasks & timers
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define CAN_WRITE_INTERVAL              100000      // 0.1 seconds in microseconds
#define LOGGER_UPDATE_INTERVAL          250000      // 0.25 seconds in microseconds
#define TASK_STACK_SIZE                 3500        // in bytes

// debug
#define ENABLE_DEBUG                    true        // master debug message control
#define MAIN_LOOP_DELAY                 1           // delay in main loop (should be set to 1 when not testing)


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
  .CAN_sentStatus = 0,
  .CAN_outgoingMessage = {},

  .RCB_updateResult = ESP_OK,
  .RCB_updateMessage = {},

  .WCB_updateResult = ESP_OK,
  .WCB_updateMessage = {},

  .IO_data = {},

  // scheduler data
  .sensorTaskCount = 0,
  .canTaskCount = 0,
  .ardanTaskCount = 0,
  .wcbTaskCount = 0,
  .rcbTaskCount = 0,
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

    .imdFault = false,
    .bmsFault = false,

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
  },

  // Inputs
  .inputs = {
    .pedal0 = 0,
    .pedal1 = 0,
    .brake0 = 0,
    .brake1 = 0,
    .brakeRegen = 0,
    .coastRegen = 0,

    .vicoreTemp = 0.0f,
    .pumpTempIn = 0.0f,
    .pimpTempOut = 0.0f,
  },

  // Outputs
  .outputs = {
    .buzzerActive = false,
    .buzzerCounter = 0,
    .brakeLight = false,
  }
};


// ESP-Now Peers
esp_now_peer_info wcbInfo = {
  .peer_addr = {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x80},    // TODO: make this use the address defined in pinConfig.h
  .channel = 0,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};

esp_now_peer_info rcbInfo = {
  .peer_addr = {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81},    // TODO: make this use the address defined in pinConfig.h
  .channel = 0,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};


// CAN Interface
can_general_config_t canConfig = CAN_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, CAN_MODE_NORMAL);   // set pins controller will use
can_timing_config_t canTimingConfig = CAN_TIMING_CONFIG_500KBITS();         // the timing of the CAN bus
can_filter_config_t canFilterConfig = CAN_FILTER_CONFIG_ACCEPT_ALL();       // filter so we only receive certain messages


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void SensorCallback(void* args);
void CANCallback(void* args);
void ESPNOWCallback(void* args);
void LoggerCallback(void* args);
void BRWheelSensorCallback(void* args);
void BLWheelSensorCallback(void* args);

// tasks
void ReadSensorsTask(void* pvParameters);
void UpdateCANTask(void* pvParameters);
void UpdateLoggerTask(void* pvParameters);

// ISRs
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length);

// helpers
long MapValue(long x, long in_min, long in_max, long out_min, long out_max);


/*
===============================================================================================
                                            Setup 
===============================================================================================
*/


void setup()
{
  // set power configuration
  esp_pm_configure(&power_configuration);

  // delay startup by 5 seconds
  vTaskDelay(5000);

  // -------------------------- initialize serial connection ------------------------ //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");

  // setup managment struct
  struct setup
  {
    bool ioActive = false;
    bool canActive = false;
    bool fcbActive = false;
    bool loggerActive = false;
  };
  setup setup;

  // -------------------------- initialize GPIO ------------------------------------- //
  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  // setup front right wheel speed sensor
  gpio_set_direction((gpio_num_t)WHEEL_HEIGHT_BR_SENSOR, GPIO_MODE_INPUT);
  gpio_set_intr_type((gpio_num_t)WHEEL_HEIGHT_BR_SENSOR, GPIO_INTR_HIGH_LEVEL);
  gpio_isr_handler_add((gpio_num_t)WHEEL_HEIGHT_BR_SENSOR, BRWheelSensorCallback, (void*) (gpio_num_t)WHEEL_HEIGHT_BR_SENSOR);
  
  // setup front left wheel speed sensor
  gpio_set_direction((gpio_num_t)WHEEL_HEIGHT_BL_SENSOR, GPIO_MODE_INPUT);
  gpio_set_intr_type((gpio_num_t)WHEEL_HEIGHT_BL_SENSOR, GPIO_INTR_HIGH_LEVEL);
  gpio_isr_handler_add((gpio_num_t)WHEEL_HEIGHT_BL_SENSOR, BLWheelSensorCallback, (void*) (gpio_num_t)WHEEL_HEIGHT_BL_SENSOR);


  // setup adc pins
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db));

  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_1, ADC_ATTEN_0db));

  // outputs //

  setup.ioActive = true;
  // -------------------------------------------------------------------------- //


  // --------------------- initialize CAN Controller -------------------------- //
  if (can_driver_install(&canConfig, &canTimingConfig, &canFilterConfig) == ESP_OK) {
    Serial.printf("CAN INIT [ SUCCESS ]\n");

    // start CAN interface
    if (can_start() == ESP_OK) {
      Serial.printf("CAN STARTED [ SUCCESS ]\n");

      // track all alerts
      if (can_reconfigure_alerts(CAN_ALERT_ALL, NULL) == ESP_OK) {
        Serial.printf("CAN ALERTS [ SUCCESS ]\n");
      } 
      else {
        Serial.printf("CAN ALERTS [ FAILED ]\n");
      }

      setup.canActive = true;
    }
  }
  else {
    Serial.printf("CAN INIT [ FAILED ]\n");
  }
  // --------------------------------------------------------------------------- //


  // -------------------------- initialize ESP-NOW  ---------------------------- //

  // init wifi and config
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  
  if (esp_wifi_start() == ESP_OK) {
    Serial.print("WIFI INIT [ SUCCESS ]\n");

    // set custom device mac address
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, deviceAddress));
  }
  else {
    Serial.print("WIFI INIT [ FAILED ]\n");
  }

  // init ESP-NOW service
  if (esp_now_init() == ESP_OK) {
    Serial.printf("ESP-NOW INIT [ SUCCESS ]\n");

    // register callback functions
    // ESP_ERROR_CHECK(esp_now_register_send_cb());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(FCBDataReceived));

    // add peers
    if (esp_now_add_peer(&wcbInfo) == ESP_OK) {
      Serial.printf("ESP-NOW FCB CONNECTION [ SUCCESS ]\n");
      setup.fcbActive = true;
    }
    else {
      Serial.printf("ESP-NOW FCB CONNECTION [ FAILED ]\n");
    }
  }

  else {
    Serial.printf("ESP-NOW INIT [ FAILED ]\n");
  }
  // ------------------------------------------------------------------------ //


  // ---------------------- initialize SD Logger ---------------------------- //
  

  // ------------------------------------------------------------------------ //


  // ---------------------- initialize timer interrupts ---------------------- //
  // timer 1 - Read Sensors 
  const esp_timer_create_args_t timer1_args = {
    .callback = &SensorCallback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "Sensor Timer"
  };
  esp_timer_handle_t timer1;
  ESP_ERROR_CHECK(esp_timer_create(&timer1_args, &timer1));

  // timer 2 - CAN Update
  const esp_timer_create_args_t timer2_args = {
    .callback = &CANCallback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "CAN Write Timer"
  };
  esp_timer_handle_t timer2;
  ESP_ERROR_CHECK(esp_timer_create(&timer2_args, &timer2));

  // timer 3 - Logger Update
  const esp_timer_create_args_t timer3_args = {
    .callback = &LoggerCallback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "Logger Update Timer"
  };
  esp_timer_handle_t timer3;
  ESP_ERROR_CHECK(esp_timer_create(&timer3_args, &timer3));

  // start timers
  if (setup.ioActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer1, SENSOR_POLL_INTERVAL));
  if (setup.canActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer2, CAN_WRITE_INTERVAL));
  if (setup.loggerActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer3, LOGGER_UPDATE_INTERVAL));

  Serial.printf("SENSOR TASK STATUS: %s\n", esp_timer_is_active(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("CAN TASK STATUS: %s\n", esp_timer_is_active(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("LOGGER TASK STATUS: %s\n", esp_timer_is_active(timer3) ? "RUNNING" : "DISABLED");
  // ----------------------------------------------------------------------------------------- //


  // ------------------- End Setup Section in Serial Monitor --------------------------------- //
  if (xTaskGetSchedulerState() == 2) {
    Serial.printf("\nScheduler Status: RUNNING\n");

    // clock frequency
    rtc_cpu_freq_config_t conf;
    rtc_clk_cpu_freq_get_config(&conf);
    Serial.printf("CPU Frequency: %dMHz\n", conf.freq_mhz);
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
void SensorCallback(void* args) {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(ReadSensorsTask, "Poll-Senser-Data", TASK_STACK_SIZE, &ucParameterToPass, 5, &xHandle);
}


/**
 * @brief callback function for creating a new CAN Update task
 * 
 * @param args arguments to be passed to the task
 */
void CANCallback(void* args) {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateCANTask, "CAN-Update", TASK_STACK_SIZE, &ucParameterToPass, 5, &xHandle);
}


/**
 * @brief callback function for creating a new ARDAN Update task
 * 
 * @param args arguments to be passed to the task
 */
void LoggerCallback(void* args) { 
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateLoggerTask, "ARDAN-Update", TASK_STACK_SIZE, &ucParameterToPass, 3, &xHandle);
}


/**
 * @brief callback function for creating a new FCB Update task
 * 
 * @param args arguments to be passed to the task
 */
void ESPNOWCallback(void* args) {

}


/**
 * @brief a callback function for when data is received from FCB
 * 
 * @param mac             the address of the FCB
 * @param incomingData    the structure of incoming data
 * @param length          size of the incoming data
 */
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length) {
  // copy data to the wcbData struct 
  memcpy((uint8_t *) &carData, incomingData, sizeof(carData));

  return;
}


/**
 * @brief callback function for when the hall effect sensor fires on front right wheel
 * 
 * @param args 
 */
void BRWheelSensorCallback(void* args) {
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

  return;
}


/**
 * @brief callback function for when the hall effect sensor fires on front left wheel
 * 
 * @param args 
 */
void BLWheelSensorCallback(void* args) {
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

  // update wheel speed values
  carData.sensors.wheelSpeedFR = adc1_get_raw(WHEEL_SPEED_BR_SENSOR);
  carData.sensors.wheelSpeedFL = adc1_get_raw(WHEEL_HEIGHT_BL_SENSOR);

  // update wheel ride height values
  carData.sensors.wheelHeightFR = adc1_get_raw(WHEEL_HEIGHT_BR_SENSOR);
  carData.sensors.wheelHeightFL = adc1_get_raw(WHEEL_HEIGHT_BL_SENSOR);


  // update brake light state
  

  
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

  // --- receive messages --- //
  // check for new messages in the CAN buffer
  if (can_receive(&incomingMessage, pdMS_TO_TICKS(1000))) {

    if (incomingMessage.flags & CAN_MSG_FLAG_NONE) {
      // filter for only the IDs we are interested in
      switch (incomingMessage.identifier)
      {
        // message from RCB: Sensor Data
        case 0x100:
        if (!(incomingMessage.flags & CAN_MSG_FLAG_RTR)) {
          // do stuff with the data in the message
        }
        break;

        // message from RCB: BMS and electrical data
        case 0x101:
        if (!(incomingMessage.flags & CAN_MSG_FLAG_RTR)) {
          // do stuff with the data in the message
        }        
        break;

        default:
        // do nothing because we didn't get any messages of interest
        break;
      }
    }

  }

  // --- send message --- // 
  can_message_t outgoingMessage;
  outgoingMessage.identifier = 0xAA;
  outgoingMessage.flags = CAN_MSG_FLAG_SELF;
  outgoingMessage.data_length_code = 8;
  bool sentStatus = false;

  // build message
  outgoingMessage.data[0] = 0x00;
  outgoingMessage.data[1] = 0x01;
  outgoingMessage.data[2] = 0x02;
  outgoingMessage.data[3] = 0x03;
  outgoingMessage.data[4] = 0x04;
  outgoingMessage.data[5] = 0x05;
  outgoingMessage.data[6] = 0x06;
  outgoingMessage.data[7] = 0x07;

  // queue message for transmission
  int result = can_transmit(&outgoingMessage, pdMS_TO_TICKS(1000));
  switch (result)
  {
  case ESP_OK:
    sentStatus = true;
    break;

  case ESP_ERR_INVALID_ARG:
    Serial.printf("Arguments are invalid\n");
  break;

  case ESP_ERR_TIMEOUT:
    Serial.printf("Timed out waiting for space on TX queue\n");
  break;

  case ESP_FAIL:
    Serial.printf("TX queue is disabled and another message is currently transmitting\n");
  break;

  case ESP_ERR_INVALID_STATE:
    Serial.printf("TWAI driver is not in running state, or is not installed\n");
  break;

  default:
    break;
  }

  // debugging
  if (debugger.debugEnabled) {
    debugger.CAN_sentStatus = sentStatus;
    for (int i = 0; i < 8; ++i) {
      debugger.CAN_outgoingMessage[i] = outgoingMessage.data[i];
    }
    debugger.canTaskCount++;
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

  // alerts
  uint32_t alerts;
  can_read_alerts(&alerts, pdMS_TO_TICKS(1000));
  
  if (alerts & CAN_ALERT_ABOVE_ERR_WARN) {
      Serial.printf("ERROR: Surpassed Error Warning Limit\n");
  }
  if (alerts & CAN_ALERT_ERR_PASS) {
          Serial.printf("ERROR: Entered Error Passive state\n");
  }
  if (alerts & CAN_ALERT_BUS_OFF) {
      Serial.printf("ERROR: Bus Off state\n");
      //Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
      can_reconfigure_alerts(CAN_ALERT_BUS_RECOVERED, NULL);
      for (int i = 3; i > 0; i--) {
          Serial.printf("Initiate bus recovery in %d\n", i);
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      can_initiate_recovery();    //Needs 128 occurrences of bus free signal
      Serial.printf("Initiate bus recovery\n");
  }
  if (alerts & CAN_ALERT_BUS_RECOVERED) {
      //Bus recovery was successful, exit control task to uninstall driver
      Serial.printf("Bus Recovered!\n");
  }


  // sent status
  Serial.printf("CAN Message Send Status: %s\n", debugger.CAN_sentStatus ? "Success" : "Failed");

  // message
  for (int i = 0; i < 8; ++i) {
    Serial.printf("CAN Raw Data Byte %d: %d\t", i, debugger.CAN_outgoingMessage[i]);
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


  // message
  Serial.printf("ready to drive status: %d\n", debugger.WCB_updateMessage.drivingData.readyToDrive);
  

  Serial.printf("\n--- END WCB DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for I/O
 * 
 */
void PrintIODebug() {
  Serial.printf("\n--- START I/O DEBUG ---\n");

  // 

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
    debugger.wcbTaskCount, debugger.rcbTaskCount, 
    debugger.ardanTaskCount);
  }
}