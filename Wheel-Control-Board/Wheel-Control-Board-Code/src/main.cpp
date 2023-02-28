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
#include "TFT_eSPI.h"
#include "TFT_eWidget.h"
#include "PNGdec.h"
#include "bessie.h"
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
#define MAX_IMAGE_WIDTH                 240         //

// tasks & timers
#define SENSOR_POLL_INTERVAL            100000      // 0.1 seconds in microseconds
#define DISPLAY_UPDATE_INTERVAL         250000      // 0.25 seconds in microseconds
#define FCB_UPDATE_INTERVAL             200000      // 0.2 seconds in microseconds
#define TASK_STACK_SIZE                 3500        // in bytes

// debug
#define ENABLE_DEBUG                    true        // master debug message control
#define MAIN_LOOP_DELAY                 1000        // in milliseconds


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
  .FCB_debugEnabled = true,
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
  }
};


// ESP-Now Peers
esp_now_peer_info fcbInfo = {
  .peer_addr = {0xC4, 0xDE, 0xE2, 0xC0, 0x75, 0x81},    // TODO: make this use the address defined in pinConfig.h
  .channel = 0,
  .ifidx = WIFI_IF_STA,
  .encrypt = false,
};


// Display 
TFT_eSPI tft = TFT_eSPI();

// init GUI elements
MeterWidget MainVolts = MeterWidget(&tft);
MeterWidget MainSpeed = MeterWidget(&tft);
MeterWidget MainBattery = MeterWidget(&tft);
MeterWidget MainDriveMode = MeterWidget(&tft);

MeterWidget ElectricalBusVolts = MeterWidget(&tft);
MeterWidget ElectricalRinehartVolts = MeterWidget(&tft);
MeterWidget ElectricalAmps = MeterWidget(&tft);
MeterWidget ElectricalBattery = MeterWidget(&tft);

// init images
TFT_eSprite imgBessie = TFT_eSprite(&tft);  // Create Sprite object "img" with pointer to "tft" object
PNG png;  // image decoder
int bessieXPos = 0;
int bessieYPos = 0;

// create mode type and tracker
typedef enum DisplayMode {
  BOOT = 0,
  MAIN = 1,
  ELECTRICAL = 2,
  MECHANICAL = 3,
} DisplayMode;
DisplayMode currentDisplayMode = BOOT;
DisplayMode previousDisplayMode = currentDisplayMode;

// create mode type and tracker
typedef enum BootMode {
  INIT_DISPLAY = 1,
  INIT_SENSORS = 2,
  INIT_ESPNOW = 3,
} BootMode;
BootMode currentBootMode = INIT_DISPLAY;

bool refreshDisplay = false;

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

// helpers
long MapValue(long x, long in_min, long in_max, long out_min, long out_max);

// display
void DisplayBootScreen(BootMode);
void DisplayMainScreen();
void DisplayElectricalScreen();
void DisplayMechanicalScreen();
void InitDisplayElements();
void pngDraw(PNGDRAW*);


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
    bool displayActive = false;
    bool fcbActive = false;
  };
  setup setup;


  // -------------------------- initialize Display  ---------------------------- //

  // inits and general setup
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // init analog meters
  InitDisplayElements();

  // show boot screen
  DisplayBootScreen();
  setup.displayActive = true;
  // --------------------------------------------------------------------------- //


  // -------------------------- initialize GPIO ------------------------------------- //

  // inputs
  gpio_set_direction((gpio_num_t)LCD_MODE_SELECT_BUTTON, GPIO_MODE_INPUT);
  

  // setup adc 1
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  ESP_ERROR_CHECK(adc1_config_channel_atten((adc1_channel_t)COAST_REGEN_KNOB, ADC_ATTEN_0db));
  ESP_ERROR_CHECK(adc1_config_channel_atten((adc1_channel_t)BRAKE_REGEN_KNOB, ADC_ATTEN_0db));

  // outputs //


  DisplayBootScreen();
  setup.ioActive = true;
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

  // get peer informtion about FCB
  memcpy(fcbInfo.peer_addr, fcbAddress, sizeof(fcbAddress));
  fcbInfo.channel = 0;
  fcbInfo.encrypt = false;

  // add FCB as a peer
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
 * @brief callback function for creating a new FCB Update task
 * 
 * @param args arguments to be passed to the task
 */
void FCBCallback(void* args) {
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate(UpdateFCBTask, "FCB-Update", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
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
  // inits
  CarData tmp;

  // copy data to the tmp CarData struct 
  memcpy((uint8_t *) &tmp, incomingData, sizeof(carData));

  // copy relevant information
  // TODO: do that ^

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
void ReadSensorsTask(void* pvParameters) {
  // turn off wifi for ADC channel 2 to function
  esp_wifi_stop();

  // read regen knobs
  float coastRegenTmp = adc1_get_raw((adc1_channel_t)COAST_REGEN_KNOB);
  carData.inputs.coastRegen = MapValue(coastRegenTmp, 0, 1024, 0, 255);    // get values through testing
  float brakeRegenTmp = adc1_get_raw((adc1_channel_t)BRAKE_REGEN_KNOB);
  carData.inputs.brakeRegen = MapValue(brakeRegenTmp, 0, 1024, 0, 255);    // get values through testing

  // debugging
  if (debugger.debugEnabled) {
    debugger.IO_data = carData;
    debugger.sensorTaskCount++;
  }

  // turn wifi back on to re-enable esp-now connection to front board
  esp_wifi_start();

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief updates the display 
 * 
 * @param pvParameters parameters passed to task
 */
void UpdateDisplayTask(void* pvParameters) {
  while (1) {
    if (currentDisplayMode != previousDisplayMode) {
      refreshDisplay = true;
      previousDisplayMode = currentDisplayMode;
    }

    switch (currentDisplayMode) {
      case BOOT:
          DisplayBootScreen();
      break;

      case MAIN:
        DisplayMainScreen();
      break;


      case ELECTRICAL:
        DisplayElectricalScreen();
      break;


      case MECHANICAL:
        DisplayMechanicalScreen();
      break;
      
      default:
        currentDisplayMode = MAIN;
        break;
    }

    // debugging
    if (debugger.debugEnabled) {
      debugger.displayTaskCount++;
    }
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
  // send message
  esp_err_t result = esp_now_send(fcbAddress, (uint8_t *) &carData, sizeof(carData));

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
                                  DISPLAY FUNCTIONS
================================================================================================
*/


/**
 * @brief the stuff on the screen during boot
 * 
 */
void DisplayBootScreen() {
  // draw bessie
  tft.startWrite();
  int rc = png.decode(NULL, 0);
  tft.endWrite();

  // boot sequence
  if (currentBootMode == INIT_DISPLAY) {
    tft.drawString("DISPLAY INITIALIZED", 0, 0);
  }

  if (currentBootMode == INIT_SENSORS) {
    tft.drawString("SENSORS INITIALIZED", 0, 0);
  }

  if (currentBootMode == INIT_ESPNOW) {
    tft.drawString("ESP-NOW INITIALIZED", 0, 0);
  }
}

/**
 * @brief main driving display
 * 
 */
void DisplayMainScreen() {
  if (refreshDisplay) {
    // rinehart voltage
    MainVolts.setZones(0, 70, 70, 80, 80, 90, 90, 100);
    MainVolts.analogMeter(0, 0, 320, "V", "0", "200", "260", "290", "320");

    // speed
    MainSpeed.analogMeter(0, 128, 75, "mph", "0", "", "50", "", "75");

    // battery precentage
    MainBattery.setZones(0, 25, 25, 50, 50, 75, 75, 100);
    MainBattery.analogMeter(128, 0, 100, "%", "0", "25", "50", "75", "100");

    // drive mode
    MainDriveMode.analogMeter(128, 128, 3, "Mode", "SLOW", "", "ECO", "", "FAST");

    refreshDisplay = false;
  }

  // update needles
  MainVolts.updateNeedle(carData.batteryStatus.rinehartVoltage, 0);
  MainSpeed.updateNeedle(carData.drivingData.currentSpeed, 0);
  MainBattery.updateNeedle(carData.batteryStatus.batteryChargeState, 0);

  int tmpMode = (int)carData.drivingData.driveMode;
  MainDriveMode.updateNeedle(tmpMode, 0);
}


/**
 * @brief 
 * 
 */
void DisplayElectricalScreen() {
  if (refreshDisplay) {
    ElectricalBusVolts.setZones(0, 100, 25, 75, 0, 0, 40, 60);
    ElectricalBusVolts.analogMeter(0, 128, 10.0, "V", "0", "2.5", "5", "7.5", "10");

    ElectricalRinehartVolts.setZones(0, 100, 25, 75, 0, 0, 40, 60);
    ElectricalRinehartVolts.analogMeter(0, 128, 10.0, "V", "0", "2.5", "5", "7.5", "10");

    ElectricalAmps.setZones(0, 100, 25, 75, 0, 0, 40, 60);
    ElectricalAmps.analogMeter(0, 128, 10.0, "A", "0", "2.5", "5", "7.5", "10");

    ElectricalBattery.setZones(0, 100, 25, 75, 0, 0, 40, 60);
    ElectricalBattery.analogMeter(0, 128, 10.0, "%", "0", "2.5", "5", "7.5", "10");
    
    refreshDisplay = false;
  }

  // update needles

}


/**
 * @brief 
 * 
 */
void DisplayMechanicalScreen() {

}


/**
 * @brief 
 * 
 */
void InitDisplayElements() {
  // --- main page --- //


  // --- electrical --- //


  // init bessie
  if (png.openFLASH((uint8_t *)bessie, sizeof(bessie), pngDraw) == PNG_SUCCESS) {
    Serial.println("Successfully opened png file");
    Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
  }
}


/**
 * @brief helper drawing function for pngs
 * 
 * @param pDraw 
 */
void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(bessieXPos, bessieYPos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
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
  Serial.printf("FCB rtd status: %d\n", carData.drivingData.readyToDrive);
  Serial.printf("message rec count: %d\n", debugger.fcbTaskCount);

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