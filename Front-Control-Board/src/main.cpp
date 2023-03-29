/**
 * @file main.cpp
 * @author Dominic Gasperini - UVM '23
 * @brief this drives the front control board on clean speed 5.5
 * @version 1.0
 * @date 2023-03-22
 */

/*
===============================================================================================
                                    Includes 
===============================================================================================
*/
// standard includes 
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
#include "esp_adc_cal.h"

// custom includes
#include "CAN.h"
#include "RadioLib.h"
#include "debugger.h"
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

// tasks & timers
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define CAN_WRITE_INTERVAL              100000      // 0.1 seconds in microseconds
#define ARDAN_UPDATE_INTERVAL           200000      // 0.2 seconds in microseconds
#define ESP_NOW_UPDATE_INTERVAL         200000      // 0.2 seconds in microseconds
#define TASK_STACK_SIZE                 4096        // in bytes

// debug
#define ENABLE_DEBUG                    true        // master debug message control
#define MAIN_LOOP_DELAY                 1000        // delay in main loop (should be set to 1 when not testing)


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


// LoRa Interface
RFM95 lora = new Module(15, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC);

/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void SensorCallback(void* args);
void CANCallback(void* args);
void ARDANCallback(void* args);
void ESPNOWCallback(void* args);
void FRWheelSensorCallback(void* args);
void FLWheelSensorCallback(void* args);

// tasks
void ReadSensorsTask(void* pvParameters);
void UpdateCANTask(void* pvParameters);
void UpdateARDANTask(void* pvParameters);
void UpdateESPNOWTask(void* pvParameters);

// ISRs
void WCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length);
void ReadyToDriveButtonPressed(void* args);

// helpers
void GetCommandedTorque();
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

  if (debugger.debugEnabled) {
    // delay startup by 5 seconds
    vTaskDelay(5000);
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

  // -------------------------- initialize GPIO ------------------------------------- //

  // * Let's say, GPIO_INPUT_IO_0=4, GPIO_INPUT_IO_1=5
  // * In binary representation,
  // * 1ULL<<GPIO_INPUT_IO_0 is equal to 0000000000000000000000000000000000010000 and
  // * 1ULL<<GPIO_INPUT_IO_1 is equal to 0000000000000000000000000000000000100000
  // * GPIO_INPUT_PIN_SEL                0000000000000000000000000000000000110000

  gpio_config_t input_interrupt_config = {
    .pin_bit_mask = ((1ULL<<36) | (1ULL<<39) | (1ULL<<34) | (1ULL<<35) | (1ULL<<32)),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_HIGH_LEVEL,
  };
  ESP_ERROR_CHECK(gpio_config(&input_interrupt_config));


  gpio_config_t output_config = {
    .pin_bit_mask = ((1ULL<<26) | (1ULL<<21) | (1ULL<<5)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&output_config));

  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  // setup front right wheel speed sensor
  gpio_isr_handler_add((gpio_num_t)WHEEL_HEIGHT_FR_SENSOR, FRWheelSensorCallback, (void*) (gpio_num_t)WHEEL_HEIGHT_FR_SENSOR);
  
  // setup front left wheel speed sensor
  gpio_isr_handler_add((gpio_num_t)WHEEL_HEIGHT_FL_SENSOR, FLWheelSensorCallback, (void*) (gpio_num_t)WHEEL_HEIGHT_FL_SENSOR);

  // setup RTD button
  gpio_isr_handler_add((gpio_num_t)RTD_BUTTON_PIN, ReadyToDriveButtonPressed, (void*) (gpio_num_t)RTD_BUTTON_PIN);

  // setup adc pins
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  ESP_ERROR_CHECK(adc1_config_channel_atten(PEDAL_0_PIN, ADC_ATTEN_11db));    // pedal 0 potentiometer
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db)); // wheel height sensor
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_11db)); // wheel height sensor

  ESP_ERROR_CHECK(adc2_config_channel_atten(BRAKE_0_PIN, ADC_ATTEN_11db));    // brake 0 potentiometer
  ESP_ERROR_CHECK(adc2_config_channel_atten(BRAKE_0_PIN, ADC_ATTEN_11db));    // brake 1 potentiometer
  ESP_ERROR_CHECK(adc2_config_channel_atten(PEDAL_1_PIN, ADC_ATTEN_11db));    // pedal 1 potentiometer

  setup.ioActive = true;
  // -------------------------------------------------------------------------- //


  // --------------------- initialize CAN Controller -------------------------- //
  if (CAN.begin(500E3)) {
    Serial.printf("CAN INIT [ SUCCESS ]\n");
    
    // set can pins
    CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);

    setup.canActive = true;
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
    ESP_ERROR_CHECK(esp_now_register_recv_cb(WCBDataReceived));

    // add peers
    if (esp_now_add_peer(&wcbInfo) == ESP_OK) {
      Serial.printf("ESP-NOW WCB CONNECTION [ SUCCESS ]\n");
      setup.wcbActive = true;
    }
    else {
      Serial.printf("ESP-NOW WCB CONNECTION [ FAILED ]\n");
    }

    if (esp_now_add_peer(&rcbInfo) == ESP_OK) {
      Serial.printf("ESP-NOW RCB CONNECTION [ SUCCESS ]\n");
      setup.rcbActive = true;
    }
    else {
      Serial.printf("ESP-NOW RCB CONNECTION [ FAILED ]\n");
    }
  }

  else {
    Serial.printf("ESP-NOW INIT [ FAILED ]\n");
  }
  // ------------------------------------------------------------------------ //


  // ------------------- initialize ARDAN Connection ------------------------ //
  // LoRa Interface
  SPI.begin();

  // init lora
  if (lora.begin()) {
    Serial.printf("ARDAN INIT [SUCCESSS ]\n");

    // set the sync word so the car and monitoring station can communicate
    lora.setSyncWord(0xA1);         // the channel to be transmitting on (range: 0x00 - 0xFF)

    setup.ardanActive = true;
  }
  else { 
    Serial.printf("ARDAN INIT [ FAILED ]\n");
  }
  // ------------------------------------------------------------------------- //


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

  // timer 3 - ARDAN Update
  const esp_timer_create_args_t timer3_args = {
    .callback = &ARDANCallback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "ARDAN Update Timer"
  };
  esp_timer_handle_t timer3;
  ESP_ERROR_CHECK(esp_timer_create(&timer3_args, &timer3));

  // timer 4 - ESP-NOW Update
  const esp_timer_create_args_t timer4_args = {
    .callback = &ESPNOWCallback,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "ESP-NOW Update Timer"
  };
  esp_timer_handle_t timer4;
  ESP_ERROR_CHECK(esp_timer_create(&timer4_args, &timer4));

  // start timers
  if (setup.ioActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer1, SENSOR_POLL_INTERVAL));
  if (setup.canActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer2, CAN_WRITE_INTERVAL));
  if (setup.ardanActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer3, ARDAN_UPDATE_INTERVAL));
  if (setup.wcbActive && setup.rcbActive)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer4, ESP_NOW_UPDATE_INTERVAL));

  Serial.printf("SENSOR TASK STATUS: %s\n", esp_timer_is_active(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("CAN TASK STATUS: %s\n", esp_timer_is_active(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("ARDAN TASK STATUS: %s\n", esp_timer_is_active(timer3) ? "RUNNING" : "DISABLED");
  Serial.printf("ESP-NOW TASK STATUS: %s\n", esp_timer_is_active(timer4) ? "RUNNING" : "DISABLED");
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
  xTaskCreate(ReadSensorsTask, "Poll-Senser-Data", TASK_STACK_SIZE, &ucParameterToPass, 6, &xHandle);
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
void ARDANCallback(void* args) {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateARDANTask, "ARDAN-Update", TASK_STACK_SIZE, &ucParameterToPass, 3, &xHandle);
}


/**
 * @brief callback function for creating a new WCB Update task
 * 
 * @param args arguments to be passed to the task
 */
void ESPNOWCallback(void* args) {
  // queue wcb update
  static uint8_t ucParameterToPassWCB;
  TaskHandle_t xHandleWCB = NULL;
  xTaskCreate(UpdateESPNOWTask, "ESP-NOW-Update", TASK_STACK_SIZE, &ucParameterToPassWCB, 4, &xHandleWCB);
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
  // copy data to the wcbData struct 
  memcpy((uint8_t *) &carData, incomingData, sizeof(carData));

  return;
}


/**
 * @brief callback function for when the hall effect sensor fires on front right wheel
 * 
 * @param args arguments to be passed to the task
 */
void FRWheelSensorCallback(void* args) {
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
 * @param args arguments to be passed to the task
 */
void FLWheelSensorCallback(void* args) {
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


/**
 * @brief handle the ready to drive button press event
 * 
 * @param args arguments to be passed to the task
 */
void ReadyToDriveButtonPressed(void* args) {
  if (carData.drivingData.readyToDrive) {
    // turn on buzzer to indicate TSV is live
    carData.outputs.buzzerActive = true;
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

  // get pedal positions
  float tmpPedal0 = adc1_get_raw(PEDAL_0_PIN);
  carData.inputs.pedal0 = MapValue(tmpPedal0, 575, 2810, 0, 255);   // starting min and max values must be found via testing!!! (0.59V - 2.75V)
  if (carData.inputs.pedal0 > 255) {
    carData.inputs.pedal0 = 255;
  }

  int tmpPedal1 = 0;
  esp_err_t resultPedal1 = adc2_get_raw(PEDAL_1_PIN, ADC_WIDTH_12Bit, &tmpPedal1);
  if (resultPedal1 == ESP_OK) {
    carData.inputs.pedal1 = MapValue(tmpPedal1, 250, 1400, 0, 255);   // starting min and max values must be found via testing!!! (0.29V - 1.379V)
  }
  if (carData.inputs.pedal1 > 255) {
    carData.inputs.pedal1 = 255;
  }

  // Calculate commanded torque
  GetCommandedTorque();


  // get brake positions
  int tmpBrake0;
  esp_err_t resultBrake0 = adc2_get_raw(BRAKE_0_PIN, ADC_WIDTH_12Bit, &tmpBrake0);
  carData.inputs.brake0 = MapValue(tmpBrake0, 0, 1024, 0, 255);   // starting min and max values must be found via testing!!! 

  int tmpBrake1;
  esp_err_t resultBrake1 = adc2_get_raw(BRAKE_1_PIN, ADC_WIDTH_12Bit, &tmpBrake1);
  carData.inputs.brake1 = MapValue(tmpBrake1, 0, 1024, 0, 255);   // starting min and max values must be found via testing!!!

  // brake light logic 
  int brakeAverage = (carData.inputs.brake0 + carData.inputs.brake1) / 2;
  if (brakeAverage >= BRAKE_LIGHT_THRESHOLD) {
    carData.outputs.brakeLight = true;      // turn it on 
  }

  else {
    carData.outputs.brakeLight = false;     // turn it off
  }


  // update wheel ride height values
  carData.sensors.wheelHeightFR = adc1_get_raw(WHEEL_HEIGHT_FR_SENSOR);
  carData.sensors.wheelHeightFL = adc1_get_raw(WHEEL_HEIGHT_FL_SENSOR);

  // update steering wheel position
  carData.sensors.steeringWheelAngle = adc1_get_raw(STEERING_WHEEL_POT);

  // buzzer logic
  if (carData.outputs.buzzerActive)
  {
    gpio_set_level((gpio_num_t)BUZZER_PIN, carData.outputs.buzzerActive);
    carData.outputs.buzzerCounter++;

    if (carData.outputs.buzzerCounter >= (2 * (SENSOR_POLL_INTERVAL / 10000)))    // convert to activations per second and multiply by 2
    {
      // update buzzer state and turn off the buzzer
      carData.outputs.buzzerActive = false;
      gpio_set_level((gpio_num_t)BUZZER_PIN, carData.outputs.buzzerActive);

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
  CAN.setPins(CAN_RX_PIN, CAN_TX_PIN);
  unsigned char incomingMessage[8];
  long id;

  // --- receive messages --- //
  // check for new messages in the CAN buffer
  for (int i = 0; i < NUM_CAN_READS; ++i) {
    int packetSize = CAN.parsePacket();

    if (packetSize || CAN.packetId() != -1) {
      id = CAN.packetId();

      // parse out data
      switch (id) {
          case RCB_CONTROL_ADDR:
            incomingMessage[0] = carData.drivingData.readyToDrive;
            incomingMessage[1] = carData.drivingData.imdFault;
            incomingMessage[2] = carData.drivingData.bmsFault;
          break;

          case RCB_DATA_ADDR:
            incomingMessage[0] = carData.sensors.wheelSpeedBR;
            incomingMessage[1] = carData.sensors.wheelSpeedBL;
            incomingMessage[2] = carData.sensors.wheelHeightBR;
            incomingMessage[3] = carData.sensors.wheelHeightBL;
          break;

          default:
          break;
      }
    }
  }

  // --- send message --- // 
  unsigned char outgoingMessage[8];
  bool sentStatus = false;
  int result;

  // build rinehart CONTROL message
  outgoingMessage[0] = carData.drivingData.commandedTorque & 0xFF; // commanded torque is sent across two bytes
  outgoingMessage[1] = carData.drivingData.commandedTorque >> 8;
  outgoingMessage[2] = 0;                                          // speed command NOT USING
  outgoingMessage[3] = 0;                                          // speed command NOT USING
  outgoingMessage[4] = carData.drivingData.driveDirection;         // 1: forward | 0: reverse (we run in reverse!)
  outgoingMessage[5] = carData.drivingData.enableInverter;         // 
  outgoingMessage[6] = MAX_TORQUE;                                 // this is the max torque value that we are establishing that can be sent to rinehart
  outgoingMessage[7] = 0;                                          // i think this one is min torque or it does nothing

  // send message
  CAN.beginPacket(RINE_CONTROL_ADDR);
  CAN.write((uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
  // CAN.endPacket();


  // build message for RCB - control
  outgoingMessage[0] = carData.outputs.brakeLight;
  outgoingMessage[1] = 0;
  outgoingMessage[2] = 0;
  outgoingMessage[3] = 0;
  outgoingMessage[4] = 0;
  outgoingMessage[5] = 0;
  outgoingMessage[6] = 0;
  outgoingMessage[7] = 0;

  // send message
  CAN.beginPacket(FCB_CONTROL_ADDR);
  CAN.write((uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
  // result = CAN.endPacket();

  if (result == 0) {
    sentStatus = true;
  }

  // debugging
  if (debugger.debugEnabled) {
    for (int i = 0; i < 8; ++i) {
      debugger.CAN_outgoingMessage[i] = outgoingMessage[i];
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
  // send LoRa update
  int result = lora.transmit((uint8_t *) &carData, sizeof(carData));

  // debugging
  if (debugger.debugEnabled) {
    debugger.ardanTransmitResult = result;
    debugger.ardanTaskCount++;
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
      carData.drivingData.commandedTorque = MapValue(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.50);
    break;

    case ECO:   // runs at 75% power
      carData.drivingData.commandedTorque = MapValue(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.75);
    break;

    case FAST:  // runs at 100% power
      carData.drivingData.commandedTorque = MapValue(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10));
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
  return ((x - in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min);
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
