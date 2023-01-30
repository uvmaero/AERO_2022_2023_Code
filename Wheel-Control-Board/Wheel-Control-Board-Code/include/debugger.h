/**
 * @file debugger.h
 * @author Dominic Gasperini
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
  // debug toggles
  bool debugEnabled;
  bool display_debugEnabled;
  bool FCB_debugEnabled;
  bool IO_debugEnabled;
  bool scheduler_debugEnable;

  // debug data
  esp_err_t FCB_updateResult;
  CarData FCB_updateMessage;
  CarData IO_data;

  // scheduler data
  int sensorTaskCount;
  int displayTaskCount;
  int fcbTaskCount;
} Debugger;


// functions
void PrintDebug();
void PrintDisplayDebug();
void PrintFCBDebug();
void PrintIODebug();