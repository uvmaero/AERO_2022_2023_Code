/**
 * @file debugger.h
 * @author Dominic Gasperini
 * @brief 
 * @version 1.0
 * @date 2023-02-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "data_types.h"


// Debug information
typedef struct Debugger
{
  // debug toggle
  bool debugEnabled;
  bool CAN_debugEnabled;
  bool FCB_debugEnabled;
  bool IO_debugEnabled;
  bool scheduler_debugEnable;

  // debug data
  byte CAN_sentStatus;
  byte CAN_outgoingMessage[8];

  esp_err_t WCB_updateResult = ESP_OK;
  CarData WCB_updateMessage = {};

  CarData IO_data;

  

  // scheduler data
  int sensorTaskCount;
  int prechargeTaskCount;
  int canTaskCount;
  int loggerTaskCount;
  int wcbTaskCount;
} Debugger;


// functions
void PrintDebug();
void PrintCANDebug();
void PrintFCBDebug();
void PrintIODebug();