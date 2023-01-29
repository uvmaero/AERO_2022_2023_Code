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
#include "dataTypes.h"


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
  byte CAN_sentStatus;
  byte CAN_outgoingMessage[8];
  esp_err_t WCB_updateResult;
  CarData WCB_updateMessage;
  CarData IO_data;

  // scheduler data
  int sensorTaskCount;
  int canTaskCount;
  int ardanTaskCount;
  int wcbTaskCount;
} Debugger;


// functions
void PrintDebug();
void PrintCANDebug();
void PrintWCBDebug();
void PrintIODebug();