/**
 * Common Configuration Settings 
 */
#pragma once

#ifdef STM32F4xx
// Define Boards:
// -- 0 = No STM board
// -- 1 = Black Pill
// -- 2 = STM32F411 Disco
#  define STM32_BOARD    2

// Audio I2S Pins
//#  define PIN_SDA        PB9
//#  define PIN_SCL        PB6
//#  define I2C_CLOCK_FREQ -1

//STM32F411 Disco is  PD4
//#  define CS43L22_RESET  PD4

//#else
//#  define STM32_BOARD 0
//#  define CS43L22_RESET -1
#endif


// Set to 1 to read after write to compare the values: Default is 0
#define VERIFY_WRITTENDATA 1

#if STM32_BOARD==1
#  include "drivers/stm32F411-generic/pins.h"
#elif STM32_BOARD==2
#  include "drivers/stm32F411-disco/pins.h"
#endif