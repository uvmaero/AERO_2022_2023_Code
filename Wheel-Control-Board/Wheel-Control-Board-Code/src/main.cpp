/**
 * @file main.cpp
 * @author Dominic Gasperini - UVM '23
 * @brief this drives the wheel control board on clean speed 5.5
 * @version 0.9
 * @date 2023-01-30
 */

/*
===============================================================================================
                                    Includes 
===============================================================================================
*/
// standard includes 
#include <stdio.h>
#include "esp_now.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "rtc.h"
#include "rtc_clk_common.h"
#include <esp_timer.h>
#include <esp_wifi.h>
#include "esp_netif.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// custom includes
#include "debugger.h"
#include "pinConfig.h"


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/
// GPIO
#define GPIO_INPUT_PIN_SELECT           1       

// definitions


// tasks & timers
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define DISPLAY_UPDATE_INTERVAL         250000      // 0.25 seconds in microseconds
#define FCB_UPDATE_INTERVAL             200000      // 0.2 seconds in microseconds
#define TASK_STACK_SIZE                 3500        // in bytes

// debug
#define ENABLE_DEBUG                    true       // master debug message control


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
  .display_debugEnabled = false,
  .FCB_debugEnabled = false,
  .IO_debugEnabled = false,
  .scheduler_debugEnable = true,

  // debug data
  .FCB_updateResult = ESP_OK,
  .FCB_updateMessage = {},

  .IO_data = {},

  // scheduler data
  .sensorTaskCount = 0,
  .displayTaskCount = 0,
  .fcbTaskCount = 0,
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
    .wheelSpeedFR = 0,
    .wheelSpeedFL = 0,
    .wheelSpeedBR = 0,
    .wheelSpeedBL = 0,
    .wheelHeightFR = 0,
    .wheelHeightFL = 0,
    .wheelHeightBR = 0,
    .wheelHeightBL = 0,

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


// ESP-Now Connection
esp_now_peer_info fcbInfo;


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void SensorCallback(void* args);
void DisplayCallback(void* args);
void FCBCallback(void* args);

// tasks
void ReadSensorsTask(void* pvParameters);
void UpdateDisplayTask(void* pvParameters);
void UpdateFCBTask(void* pvParameters);

// ISRs
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length);


/*
===============================================================================================
                                            Setup 
===============================================================================================
*/


void setup()
{
  // set power configuration
  esp_pm_configure(&power_configuration);

  // -------------------------- initialize serial connection ------------------------ //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");

  // setup managment struct
  struct setup
  {
    bool ioActive = false;
    bool displayActive = false;
    bool fcbActive = false;
  };
  setup setup;

  // -------------------------- initialize GPIO ------------------------------------- //

  // inputs / sensors // 
  gpio_config_t sensor_config = {
    .pin_bit_mask = GPIO_INPUT_PIN_SELECT,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&sensor_config));

  // setup adc 1
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db));

  // setup adc 2
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_1, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_2, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_5, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_0db));


  // outputs //

  setup.ioActive = true;
  // --------------------------------------------------------------------------- //


  // -------------------------- initialize Display  ---------------------------- //





  // --------------------------------------------------------------------------- //


  // -------------------------- initialize ESP-NOW  ---------------------------- //
  // turn on wifi access point 
  if (esp_netif_init() == ESP_OK) {
    Serial.printf("TCP/IP INIT: [ SUCCESS ]\n");
    
    // init wifi and config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) == ESP_OK) {
      Serial.printf("WIFI INIT: [ SUCCESS ]\n");

      ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
      ESP_ERROR_CHECK(esp_wifi_start());
      ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    }
    else {
      Serial.printf("WIFI INIT: [ FAILED ]\n");
    }
  }
  else {
    Serial.printf("ESP TCP/IP STATUS: [ FAILED ]\n");
  }
  
  // init ESP-NOW service
  if (esp_now_init() == ESP_OK) {
    Serial.printf("ESP-NOW INIT [ SUCCESS ]\n");
    
    if (esp_wifi_set_mac(WIFI_IF_STA, deviceAddress) == ESP_OK) {
      Serial.printf("MAC ADDRESS UPDATE: [ SUCCESS ]\n");
      Serial.printf("Address: %x\n", deviceAddress);
    }
  }
  else {
    Serial.printf("ESP-NOW INIT [ FAILED ]\n");
  }

  // get peer informtion about WCB
  memcpy(fcbInfo.peer_addr, fcbAddress, sizeof(fcbAddress));
  fcbInfo.channel = 0;
  fcbInfo.encrypt = false;

  // add WCB as a peer
  if (esp_now_add_peer(&fcbInfo) == ESP_OK) {
    Serial.printf("ESP-NOW CONNECTION [ SUCCESS ]\n");

    setup.fcbActive = true;
  }
  else {
    Serial.printf("ESP-NOW CONNECTION [ FAILED ]\n");
  }

  // attach message received ISR to the data received function
  esp_now_register_recv_cb(FCBDataReceived);
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

  // timer 2 - Display Update
  const esp_timer_create_args_t timer2_args = {
    .callback = &DisplayCallback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "Display Update Timer"
  };
  esp_timer_handle_t timer2;
  ESP_ERROR_CHECK(esp_timer_create(&timer2_args, &timer2));

  // timer 3 - FCB Update
  const esp_timer_create_args_t timer3_args = {
    .callback = &FCBCallback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "FCB Update Timer"
  };
  esp_timer_handle_t timer3;
  ESP_ERROR_CHECK(esp_timer_create(&timer3_args, &timer3));


  // start timers
  if (setup.ioActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer1, SENSOR_POLL_INTERVAL));
  if (setup.displayActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer2, DISPLAY_UPDATE_INTERVAL));
  if (setup.fcbActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer3, FCB_UPDATE_INTERVAL));

  Serial.printf("TIMER 1 STATUS: %s\n", esp_timer_is_active(timer1) ? "RUNNING" : "FAILED");
  Serial.printf("TIMER 2 STATUS: %s\n", esp_timer_is_active(timer2) ? "RUNNING" : "FAILED");
  Serial.printf("TIMER 3 STATUS: %s\n", esp_timer_is_active(timer3) ? "RUNNING" : "FAILED");
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
    Serial.printf("\nScheduler STATUS: FAILED\nHALTING OPERATIONS!");
    while (1) {};
  }
  Serial.printf("\n\n|--- END SETUP ---|\n\n\n");
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
  xTaskCreate(ReadSensorsTask, "Poll-Senser-Data", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
}


/**
 * @brief callback function for creating a new Display Update task
 * 
 * @param args arguments to be passed to the task
 */
void DisplayCallback(void* args) {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateDisplayTask, "Display-Update", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
}


/**
 * @brief callback function for creating a new WCB Update task
 * 
 * @param args arguments to be passed to the task
 */
void FCBCallback(void* args) {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateFCBTask, "WCB-Update", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
}


/**
 * @brief a callback function for when data is received from FCB
 * 
 * @param mac             the address of the WCB
 * @param incomingData    the structure of incoming data
 * @param length          size of the incoming data
 */
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length)
{
  // inits
  CarData tmp;

  // copy data to the wcbData struct 
  memcpy(&tmp, incomingData, sizeof(tmp));

  // get updated WCB data
  carData.drivingData.driveDirection = tmp.drivingData.driveDirection;
  carData.drivingData.driveMode = tmp.drivingData.driveMode;
  carData.inputs.coastRegen = tmp.inputs.coastRegen;
  carData.inputs.brakeRegen = tmp.inputs.brakeRegen;
  carData.outputs.buzzerActive = tmp.outputs.buzzerActive;
  carData.drivingData.readyToDrive = tmp.drivingData.readyToDrive;

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

  // TODO: read sensors
  
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
 * @brief updates the display 
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateDisplayTask(void* pvParameters)
{
  // TODO: update the display

  // debugging
  if (debugger.debugEnabled) {
    debugger.displayTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief updates FCB with car data
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateFCBTask(void* pvParameters)
{
  // inits
  CarData tmp;

  // update battery & electrical data
  tmp.batteryStatus.batteryChargeState = carData.batteryStatus.batteryChargeState;
  tmp.batteryStatus.pack1Temp = carData.batteryStatus.pack1Temp;
  tmp.batteryStatus.pack2Temp = carData.batteryStatus.pack2Temp;

  // update sensor data
  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;

  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;
  tmp.sensors.wheelSpeedFR = carData.sensors.wheelSpeedFR;

  // send message
  esp_err_t result = esp_now_send(fcbAddress, (uint8_t *) &tmp, sizeof(tmp));

  // debugging 
  if (debugger.debugEnabled) {
    debugger.FCB_updateResult = result;
    debugger.fcbTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/


/**
 * @brief main loop!
 * 
 */
void loop()
{
  // everything is managed by RTOS, so nothing really happens here!
  vTaskDelay(1);    // prevent watchdog from getting upset

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
 * @brief a nice interface for debugging display data
 * 
 */
void PrintDisplayDebug() {
  Serial.printf("\n--- START DISPLAY DEBUG ---\n");

  // TODO: add debug features

  Serial.printf("\n--- END DISPLAY DEBUG ---\n");
}


/**
 * @brief some nice in-depth debugging for FCB updates
 * 
 */
void PrintFCBDebug() {
  Serial.printf("\n--- START WCB DEBUG ---\n");

  // send status
  Serial.printf("WCB ESP-NOW Update: %s\n", debugger.FCB_updateResult ? "Success" : "Failed");


  // message
  // TODO: decide what to put here

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
  if (debugger.display_debugEnabled) {
      PrintDisplayDebug();
  }

  // WCB
  if (debugger.FCB_debugEnabled) {
    PrintFCBDebug();
  }

  // I/O
  if (debugger.IO_debugEnabled) {
    PrintIODebug();
  }

  // Scheduler
  if (debugger.scheduler_debugEnable) {
    Serial.printf("sensor: %d | display: %d | fcb: %d\n", debugger.sensorTaskCount, debugger.displayTaskCount, debugger.fcbTaskCount);
  }
}