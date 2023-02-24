#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32-config.h"
#ifdef STM32F4xx
#  include "stm32f4xx_hal.h"
#endif

/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1

#define BITMASK_SET(x, mask) ((x) |= (mask))
#define BITMASK_CLEAR(x, mask) ((x) &= (~(mask)))
#define BITMASK_FLIP(x, mask) ((x) ^= (mask))
#define BITMASK_CHECK_ALL(x, mask) (!(~(x) & (mask)))
#define BITMASK_CHECK_ANY(x, mask) ((x) & (mask))

#ifdef ARDUINO
#  include "Arduino.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

int CODEC_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
void STM32_LOG(const char *fmt,...);
void CODEC_AUDIO_POWER(bool active);

/* AUDIO IO functions */
void AUDIO_IO_Init(void);
void AUDIO_IO_DeInit(void);
void AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
int AUDIO_IO_Read(uint8_t Addr, uint8_t Reg);


#ifdef __cplusplus
}
#endif
