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


#include <pin_config.h>
#include <data_types.h>


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/
// SD Card
#define SD_BUFF                         128

// definitions
#define TIRE_DIAMETER                   20.0        // diameter of the vehicle's tires in inches
#define WHEEL_RPM_CALC_THRESHOLD        100         // the number of times the hall effect sensor is tripped before calculating vehicle speed
#define PRECHARGE_FLOOR                 0.9         // minimum percentage of acceptable voltage to run car
#define MIN_BUS_VOLTAGE                 220         // a voltage that can only be reached with two active packs

// CAN
#define FCB_CONTROL_ADDR                0x0A
#define FCB_DATA_ADDR                   0x0B
#define RCB_CONTROL_ADDR                0x0C
#define RCB_DATA_ADDR                   0x0D
#define RINE_CONTROL_ADDR               0x0C0
#define RINE_MOTOR_INFO_ADDR            0x0A5
#define RINE_VOLT_INFO_ADDR             0x0A7

// ESP-NOW
#define WCB_ADDRESS                     {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x80}
#define FCB_ADDRESS                     {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81}
#define DEVICE_ADDRESS                  {0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a}

// tasks & timers
#define SENSOR_POLL_INTERVAL            10000       // 0.01 seconds in microseconds
#define PRECHARGE_INTERVAL              10000       // 0.01 seconds in microseconds
#define CAN_WRITE_INTERVAL              10000       // 0.01 seconds in microseconds
#define LOGGER_UPDATE_INTERVAL          100000      // 0.1 seconds in microseconds
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
  .CAN_debugEnabled = false,
  .IO_debugEnabled = false,
  .scheduler_debugEnable = true,

  // debug data
  .CAN_sentStatus = 0,
  .CAN_outgoingMessage = {},

  .WCB_updateResult = ESP_OK,
  .WCB_updateMessage = {},

  .IO_data = {},

  .prechargeState = PRECHARGE_OFF,

  // scheduler data
  .sensorTaskCount = 0,
  .prechargeTaskCount = 0,
  .canTaskCount = 0,
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

esp_now_peer_info_t fcbInfo = {
  .peer_addr = FCB_ADDRESS,
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


// SD Card Interface


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void SensorCallback();
void PrechargeCallback();
void CANCallback();
void ESPNOWCallback();
void LoggerCallback();
void BRWheelSensorCallback();
void BLWheelSensorCallback();

// tasks
void ReadSensorsTask(void* pvParameters);
void UpdateCANTask(void* pvParameters);
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
    bool prechargeActive = false;
  };
  setup setup;


  // -------------------------- initialize GPIO ------------------------------------- //
  analogReadResolution(12);
  // analogSetAttenuation();

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
  setup.prechargeActive = true;
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

    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    Serial.print("WIFI MAC: "); Serial.println(WiFi.macAddress());
    Serial.print("WIFI CHANNEL: "); Serial.println(WiFi.channel());

    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
      Serial.printf("ESP-NOW INIT [ SUCCESS ]\n");
    }
  }
    else {
      Serial.printf("ESP-NOW INIT [ FAILED ]\n");
    }

  // ------------------------------------------------------------------------ //


  // ---------------------- initialize SD Logger ---------------------------- //
  // // init sd card
  // if (esp_vfs_fat_sdmmc_mount(sdMountPoint, &sdHost, &sdSlotConfig, &sdMountConfig, &sdCard) == ESP_OK) {
  //   Serial.printf("SD CARD INIT [ SUCCESS ]\n");

  //   // SD connected pins should have external 10k pull-ups.
  //   // uncomment if those are not enough
  //   // ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SD_CMD_PIN, GPIO_PULLUP_ONLY));
  //   // ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SD_D0_PIN, GPIO_PULLUP_ONLY));
  //   // ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SD_D1_PIN, GPIO_PULLUP_ONLY));
  //   // ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SD_D2_PIN, GPIO_PULLUP_ONLY));
  //   // ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SD_D3_PIN, GPIO_PULLUP_ONLY));

  //   // check for sd card inserted
  //   if (gpio_get_level((gpio_num_t) SD_DETECT_PIN)) {
  //     Serial.printf("SD CARD DETECT [ CONNECTED ]\n");

  //     // get log file number from existing file for per-boot file creation
  //     FILE* trackerFile = fopen(SD_MOUNT_POINT"/tracker.txt", "r");
  //     FILE* tmpFile = fopen(SD_MOUNT_POINT"/tmp.txt", "w");
  //     // ensure the file can be opened
  //     if (trackerFile != NULL && tmpFile != NULL) {
  //       char line[SD_BUFF];
  //       fgets(line, sizeof(line), trackerFile);
  //       sdLogfileNumber = atoi(line);

  //       // update tracker number and write to temp file
  //       sdLogfileNumber++;
  //       fprintf(tmpFile, "%d", sdLogfileNumber);

  //       // close files to save changes
  //       fclose(trackerFile);
  //       fclose(tmpFile);

  //       Serial.printf("SD CARD TRACKER FILE READ [ SUCCESS ]\n");

  //       // delete original file and update tmp file
  //       remove(SD_MOUNT_POINT"/tracker.txt");
  //       if (rename(SD_MOUNT_POINT"/tmp.txt", SD_MOUNT_POINT"/tracker.txt") == 0) {
  //         Serial.printf("SD CARD TRACKER FILE UPDATE [ SUCCESS ]\n");

  //         // set filename for logfile
  //         GenerateFilename();
          
  //         setup.loggerActive = true;
  //       }
        
  //       // failed to rename file tmp to tracker
  //       else {
  //         Serial.printf("SD CARD TRACKER FILE UPDATE [ FAILED ]\n");
  //       }

  //     // on open file open failure 
  //     }
  //     else {
  //       Serial.printf("SD TRACKER FILE READ [ FAILED ]\n");
  //     }
  
  //   // failed to detect sd card via sd detect pin 
  //   }
  //   else {
  //     Serial.printf("SD CARD DETECT [ FAILED ]\n");
  //   }

  // // sd card struct init failed
  // }
  // else {
  //   Serial.printf("SD CARD INIT [ FAILED ]\n");
  // }
  

  // ------------------------------------------------------------------------ //


  // ---------------------- initialize timer interrupts ---------------------- //
  // timer 1 - Read Sensors 
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &SensorCallback, true);
  timerAlarmWrite(timer1, SENSOR_POLL_INTERVAL, true);

  // timer 2 - CAN Update
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &CANCallback, true);
  timerAlarmWrite(timer2, CAN_WRITE_INTERVAL, true);

  // timer 3 - Logger Update
  timer3 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer3, &LoggerCallback, true);
  timerAlarmWrite(timer3, LOGGER_UPDATE_INTERVAL, true);

  // timer 4 - ESP-NOW Update
  timer4 = timerBegin(3, 80, true);
  timerAttachInterrupt(timer4, &PrechargeCallback, true);
  timerAlarmWrite(timer4, PRECHARGE_INTERVAL, true);

  // start timers
  if (setup.ioActive)
    timerAlarmEnable(timer1);
  if (setup.canActive)
    timerAlarmEnable(timer2);
  if (setup.loggerActive)
    timerAlarmEnable(timer3);
  if (setup.prechargeActive)
    timerAlarmEnable(timer4);

  Serial.printf("SENSOR TASK STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("CAN TASK STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("LOGGER TASK STATUS: %s\n", timerAlarmEnabled(timer3) ? "RUNNING" : "DISABLED");
  Serial.printf("PRECHARGE TASK STATUS: %s\n", timerAlarmEnabled(timer4) ? "RUNNING" : "DISABLED");

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
  // queue sensor task
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(ReadSensorsTask, "Poll-Senser-Data", TASK_STACK_SIZE, &ucParameterToPass, 5, &xHandle);
}


/**
 * @brief callback function for creating a new precharge task
 * 
 * @param args arguments to be passed to the task
 */
void PrechargeCallback() {
  // queue precharge task
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(PrechargeTask, "Precharge-Data", TASK_STACK_SIZE, &ucParameterToPass, 6, &xHandle);
}


/**
 * @brief callback function for creating a new CAN Update task
 * 
 * @param args arguments to be passed to the task
 */
void CANCallback() {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateCANTask, "CAN-Update", TASK_STACK_SIZE, &ucParameterToPass, 5, &xHandle);
}


/**
 * @brief callback function for creating a new Logger Update task
 * 
 * @param args arguments to be passed to the task
 */
void LoggerCallback() { 
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateLoggerTask, "ARDAN-Update", TASK_STACK_SIZE, &ucParameterToPass, 4, &xHandle);
}


/**
 * @brief callback function for creating a new WCB Update task
 * 
 * @param args arguments to be passed to the task
 */
void ESPNOWCallback() {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateWCBTask, "WCB-Update", TASK_STACK_SIZE, &ucParameterToPass, 3, &xHandle);
}


/**
 * @brief a callback function for when data is received from FCB
 * 
 * @param mac             the address of the FCB
 * @param incomingData    the structure of incoming data
 * @param length          size of the incoming data
 */
void FCBDataReceived(const uint8_t* mac, const uint8_t* incomingData, int length) {
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

  return;
}


/**
 * @brief callback function for when the hall effect sensor fires on front right wheel
 * 
 * @param args 
 */
void BRWheelSensorCallback() {
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
void BLWheelSensorCallback() {
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

  // update wheel ride height values
  carData.sensors.wheelHeightFR = adc1_get_raw(WHEEL_HEIGHT_BR_SENSOR);
  carData.sensors.wheelHeightFL = adc1_get_raw(WHEEL_HEIGHT_BL_SENSOR);

  // read radiator sensors
  carData.sensors.pumpTempIn = adc1_get_raw(RAD_TEMP_IN_PIN);
  carData.sensors.pumpTempOut = adc1_get_raw(RAD_TEMP_OUT_PIN);

  // update fans
  if (carData.outputs.fansActive) {
    gpio_set_level((gpio_num_t)FAN_ENABLE_PIN, 1);    // turn on fans
  }
  else {
    gpio_set_level((gpio_num_t)FAN_ENABLE_PIN, 0);    // turn off fans
  }

  // update pump
  if (carData.outputs.pumpActive) {
    gpio_set_level((gpio_num_t)PUMP_ENABLE_PIN, 1);   // turn on pump
  }
  else {
    gpio_set_level((gpio_num_t)PUMP_ENABLE_PIN, 0);   // turn off pump
  }

  // read faults
  carData.drivingData.imdFault = gpio_get_level((gpio_num_t)IMD_FAULT_PIN);
  carData.drivingData.bmsFault = gpio_get_level((gpio_num_t)BMS_FAULT_PIN);

  // update brake light state
  if (carData.outputs.brakeLight) {
    gpio_set_level((gpio_num_t)BRAKE_LIGHT_PIN, 1);   // turn on the brake light
  }
  else {
    gpio_set_level((gpio_num_t)BRAKE_LIGHT_PIN, 0);   // turn off the brake light
  }

  // turn wifi back on to re-enable esp-now connection to wheel board
  esp_wifi_start();

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

      // send a message to rinehart
      outgoingMessage.identifier = 0xAA;
      outgoingMessage.flags = CAN_MSG_FLAG_NONE;
      outgoingMessage.data_length_code = 8;

      // build message
      // message is sent to rinehart to turn everything off
      outgoingMessage.data[0] = 1;          // parameter address. LSB
      outgoingMessage.data[1] = 0;          // parameter address. MSB
      outgoingMessage.data[2] = 1;          // Read / Write. 1 is write
      outgoingMessage.data[3] = 0;          // N/A
      outgoingMessage.data[4] = 0;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      outgoingMessage.data[5] = 0x55;       // 55 means relay control
      outgoingMessage.data[6] = 0;          // N/A
      outgoingMessage.data[7] = 0;          // N/A


      // queue message for transmission
      result = can_transmit(&outgoingMessage, pdMS_TO_TICKS(100));

      // ensure message was successfully sent
      if (result == ESP_OK) {
        carData.drivingData.prechargeState = PRECHARGE_ON;
      }
      else {
        carData.drivingData.prechargeState = PRECHARGE_ERROR;
      }
    break;

    // do precharge
    case PRECHARGE_ON:
      // ensure voltages are above correct values
      if ((carData.batteryStatus.rinehartVoltage > (carData.batteryStatus.busVoltage * PRECHARGE_FLOOR)) && (carData.batteryStatus.busVoltage > MIN_BUS_VOLTAGE)) {
        carData.drivingData.prechargeState = PRECHARGE_DONE;
      }

      // re-send message to ensure relay is on
      else {
        // send a message to rinehart
        outgoingMessage.identifier = 0xAA;
        outgoingMessage.flags = CAN_MSG_FLAG_NONE;
        outgoingMessage.data_length_code = 8;

        // build message
        // message is sent to rinehart to turn on precharge relay
        // precharge relay is on relay 1 from Rinehart
        outgoingMessage.data[0] = 1;            // parameter address. LSB
        outgoingMessage.data[1] = 0;            // parameter address. MSB
        outgoingMessage.data[2] = 1;            // Read / Write. 1 is write
        outgoingMessage.data[3] = 0;            // N/A
        outgoingMessage.data[4] = 1;            // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
        outgoingMessage.data[5] = 0x55;         // 55 means relay control
        outgoingMessage.data[6] = 0;            // N/A
        outgoingMessage.data[7] = 0;            // N/A

        // queue message for transmission
        result = can_transmit(&outgoingMessage, pdMS_TO_TICKS(100));

        // ensure message was successfully sent
        if (result == ESP_OK) {
          carData.drivingData.prechargeState = PRECHARGE_ON;
        }
        else {
          carData.drivingData.prechargeState = PRECHARGE_ERROR;
        }
      }
    break;


    // precharge complete!
    case PRECHARGE_DONE:
      // message is sent to rinehart to turn everything on
      // Keep precharge relay on and turn on main contactor
      outgoingMessage.data[0] = 1;            // parameter address. LSB
      outgoingMessage.data[1] = 0;            // parameter address. MSB
      outgoingMessage.data[2] = 1;            // Read / Write. 1 is write
      outgoingMessage.data[3] = 0;            // N/A
      outgoingMessage.data[4] = 3;            // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      outgoingMessage.data[5] = 0x55;         // 55 is relay control
      outgoingMessage.data[6] = 0;            // N/A
      outgoingMessage.data[7] = 0;            // N/A

      // queue message for transmission
      result = can_transmit(&outgoingMessage, pdMS_TO_TICKS(100));

      // ensure message was successfully sent
      if (result != ESP_OK) {
        carData.drivingData.prechargeState = PRECHARGE_ERROR;
      } 

      // if rinehart voltage drops below battery, something's wrong, 
      if (carData.batteryStatus.rinehartVoltage < (carData.batteryStatus.busVoltage * PRECHARGE_FLOOR)) {
        carData.drivingData.prechargeState = PRECHARGE_ERROR;
      }

    break;


    // error state
    case PRECHARGE_ERROR:
      // send a message to rinehart
      outgoingMessage.identifier = 0xAA;
      outgoingMessage.flags = CAN_MSG_FLAG_NONE;
      outgoingMessage.data_length_code = 8;
      
      // build message
      // message is sent to rinehart to turn everything off
      outgoingMessage.data[0] = 1;            // parameter address. LSB
      outgoingMessage.data[1] = 0;            // parameter address. MSB
      outgoingMessage.data[2] = 1;            // Read / Write. 1 is write
      outgoingMessage.data[3] = 0;            // N/A
      outgoingMessage.data[4] = 0;            // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      outgoingMessage.data[5] = 0x55;         // 55 means relay control
      outgoingMessage.data[6] = 0;            // N/A
      outgoingMessage.data[7] = 0;            // N/A

      // queue message for transmission
      result = can_transmit(&outgoingMessage, pdMS_TO_TICKS(100));

      // ensure car cannot drive
      carData.drivingData.readyToDrive = false;
      carData.drivingData.commandedTorque = 0;
      carData.drivingData.enableInverter = false;

      // ensure we do not leave precharge error
      carData.drivingData.prechargeState = PRECHARGE_ERROR;
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
 * @brief reads and writes to the CAN bus
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateCANTask(void* pvParameters)
{
  // inits
  can_message_t incomingMessage;
  int result;

  // --- receive messages --- //
  // check for new messages in the CAN buffer
  if (can_receive(&incomingMessage, pdMS_TO_TICKS(100))) {

    if (incomingMessage.flags & CAN_MSG_FLAG_NONE) {
      // filter for only the IDs we are interested in
      switch (incomingMessage.identifier)
      {
        // Rinehart: voltage
        case RINE_VOLT_INFO_ADDR:
        if (!(incomingMessage.flags & CAN_MSG_FLAG_RTR)) {
          // rinehart voltage is spread across the first 2 bytes
          int rine1 = incomingMessage.data[0];
          int rine2 = incomingMessage.data[1];

          // combine the first two bytes and assign that to the rinehart voltage
          carData.batteryStatus.rinehartVoltage = (rine2 << 8) | rine1;
        }
        break;

        // BMS: voltage and maybe other things
        case 0x101:   // TODO: update this
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
  bool sentStatus = false;

  // build message for FCB 
  outgoingMessage.identifier = RCB_CONTROL_ADDR;
  outgoingMessage.flags = CAN_MSG_FLAG_NONE;
  outgoingMessage.data_length_code = 8;

  outgoingMessage.data[0] = carData.drivingData.readyToDrive;
  outgoingMessage.data[1] = carData.drivingData.imdFault;
  outgoingMessage.data[2] = carData.drivingData.bmsFault;
  outgoingMessage.data[3] = 0x00;
  outgoingMessage.data[4] = 0x00;
  outgoingMessage.data[5] = 0x00;
  outgoingMessage.data[6] = 0x00;
  outgoingMessage.data[7] = 0x00;

  // queue message for transmission
  result = can_transmit(&outgoingMessage, pdMS_TO_TICKS(100));

  // build message for FCB 
  outgoingMessage.identifier = RCB_DATA_ADDR;
  outgoingMessage.flags = CAN_MSG_FLAG_NONE;
  outgoingMessage.data_length_code = 8;

  outgoingMessage.data[0] = carData.sensors.wheelSpeedBR;
  outgoingMessage.data[1] = carData.sensors.wheelSpeedBL;
  outgoingMessage.data[2] = carData.sensors.wheelHeightBR;
  outgoingMessage.data[3] = carData.sensors.wheelHeightBL;
  outgoingMessage.data[4] = 0x00;
  outgoingMessage.data[5] = 0x00;
  outgoingMessage.data[6] = 0x00;
  outgoingMessage.data[7] = 0x00;

  // queue message for transmission
  esp_err_t result = can_transmit(&outgoingMessage, pdMS_TO_TICKS(10));

  // debugging
  if (debugger.debugEnabled) {
    debugger.CAN_sentStatus = sentStatus;
    for (int i = 0; i < 8; ++i) {
      debugger.CAN_outgoingMessage[i] = outgoingMessage.data[i];
    }

    // Serial.printf("result status: 0x%X\n", result);
    debugger.canTaskCount++;
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
  const uint8_t wcbAddress[6] = WCB_ADDRESS;
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
//   // inits
  char tmpStr[SD_BUFF];
  bool logWritten = false;
  float timeStamp = (float)esp_timer_get_time() * 1000000;    // convert uptime from microseconds to seconds

  // open file
  FILE* logFile = fopen(sdLogFilename, "w");

  // ensure file is open before writing and write those logs
  if (logFile != NULL) {
    // write start of block seperator
    fprintf(logFile, "\n------------------------------\n");

    // write timestamp
    fprintf(logFile, "\t\t[ %.2f seconds ]:\n\n", timeStamp);

    // write data
    fprintf(logFile, "DRIVE STATE: RTD: %d | INVER-EN: %d | MODE: %d\n", carData.drivingData.readyToDrive, carData.drivingData.enableInverter, (int)carData.drivingData.driveMode);

    fprintf(logFile, "FAULTS: IMD: %d | BMS: %d\n", carData.drivingData.imdFault, carData.drivingData.bmsFault);

    fprintf(logFile, "DRIVE STATS: COMM-TORQ: %d | SPEED: %f | DIR: %d\n", carData.drivingData.commandedTorque, carData.drivingData.currentSpeed, carData.drivingData.driveDirection);
    
    fprintf(logFile, "PEDALS: P1: %d | P2: %d | B1: %d | B2: %d\n", carData.inputs.pedal0, carData.inputs.pedal1, carData.inputs.brake0, carData.inputs.brake1);
    
    fprintf(logFile, "WHEEL SPEED: FR: %f | FL: %f | BR: %f | BL: %f\n", carData.sensors.wheelSpeedFR, carData.sensors.wheelSpeedFL, carData.sensors.wheelSpeedBR, carData.sensors.wheelSpeedBL);
    
    fprintf(logFile, "WHEEL HEIGHT: FR: %f | FL: %f | BR: %f | BL: %f\n", carData.sensors.wheelHeightFR, carData.sensors.wheelHeightFL, carData.sensors.wheelHeightBR, carData.sensors.wheelHeightBL);
    
    fprintf(logFile, "STEERING ANGLE: %d\n", carData.sensors.steeringWheelAngle);

    fprintf(logFile, "PACKS: CHARGE: %f | BUS-V: %f | RINE-V: %f\n", carData.batteryStatus.batteryChargeState, carData.batteryStatus.busVoltage, carData.batteryStatus.rinehartVoltage);
    
    fprintf(logFile, "TEMPS: PACK-1: %f | PACK-2: %f | VICORE: %f | PUMP-I: %f | PUMP-O: %f\n", carData.batteryStatus.pack1Temp, carData.batteryStatus.pack2Temp, carData.sensors.vicoreTemp, carData.sensors.pumpTempIn, carData.sensors.pumpTempOut);

    fprintf(logFile, "OUTPUTS: BUZZER: %d | BRAKE: %d | FAN-EN: %d | PUMP-EN: %d\n", carData.outputs.buzzerActive, carData.outputs.brakeLight, carData.outputs.fansActive, carData.outputs.pumpActive);

    // write end of block seperator
    fprintf(logFile, "\n------------------------------\n");
  }


  else {
    Serial.printf("Failed to open log file!\n");
  }

  // close log file
  fclose(logFile);


  // debugging
  if (debugger.debugEnabled) {
    debugger.loggerTaskCount++;
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
 * @brief generate a new log filename based on tracker.txt
 * 
 */
void GenerateFilename() {
  // inits
  char num[SD_BUFF];

  strcat(sdLogFilename, SD_MOUNT_POINT"/poop-aids-");
  strcat(sdLogFilename, itoa(sdLogfileNumber, num, SD_BUFF));
  strcat(sdLogFilename, ".txt");
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

  // FCB
  if (debugger.FCB_debugEnabled) {
    PrintWCBDebug();
  }

  // I/O
  if (debugger.IO_debugEnabled) {
    PrintIODebug();
  }

  // Scheduler
  if (debugger.scheduler_debugEnable) {
    Serial.printf("sensor: %d | can: %d | precharge: %d | logger: %d\n", debugger.sensorTaskCount, debugger.canTaskCount, debugger.prechargeTaskCount, debugger.loggerTaskCount);
  }
}