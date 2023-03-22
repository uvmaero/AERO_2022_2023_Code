/**
 * @file debugger.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <Arduino.h>
#include "data_types.h"


// Debug information
typedef struct Debugger
{
  // debug toggle
  bool debugEnabled;
  bool CAN_debugEnabled;
  bool WCB_debugEnabled;
  bool IO_debugEnabled;
  bool scheduler_debugEnable;

  // debug data
  int CAN_sentStatus;
  int CAN_outgoingMessage[8];

  esp_err_t RCB_updateResult = ESP_OK;
  CarData RCB_updateMessage = {};

  esp_err_t WCB_updateResult;
  CarData WCB_updateMessage;

  CarData IO_data;

  int ardanTransmitResult;

  // scheduler data
  int sensorTaskCount;
  int canTaskCount;
  int ardanTaskCount;
  int espnowTaskCount;
} Debugger;


// functions
void PrintDebug();
void PrintCANDebug();
void PrintWCBDebug();
void PrintIODebug();