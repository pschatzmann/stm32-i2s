/**
 * @file log.cpp
 * @author Phil Schatzmann
 * @brief Output to Serial for logging
 * @version 0.1
 * @date 2022-09-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Arduino.h"

extern "C" void STM32_LOG(const char *fmt,...) {
  char log_buffer[200];
  strcpy(log_buffer,"STM32: ");  
  va_list arg;
  va_start(arg, fmt);
  int len = vsnprintf(log_buffer + 7,200, fmt, arg);
  va_end(arg);
  Serial.println(log_buffer);
}
