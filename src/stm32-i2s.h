/**
 * @file stm32-i2s.h
 * @author phil Schatzmann
 * @brief Main entry point header for this library
 * @version 0.1
 * @date 2022-09-22
 * 
 * @copyright Copyright (c) 2022
* @addtogroup I2S
  * @{
 * 
 */

#include "stm32f4xx_hal.h"


#define I2S_BUFFER_SIZE 512
#ifdef STM32F407xx 
#  define USE_PLLI2SM false
#else
#  define USE_PLLI2SM true
#endif


#ifdef __cplusplus
extern "C" {
#else
typedef _Bool bool;
#endif

extern I2S_HandleTypeDef hi2s3;

// supported parameters
typedef struct I2SSettingsSTM32 {
  uint32_t mode;
  uint32_t standard;
  uint32_t fullduplexmode;
  uint32_t sample_rate;
  I2S_HandleTypeDef *i2s;
} I2SSettingsSTM32;


void SysTick_Handler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);

/// Start to transmit I2S data
bool startI2STransmit(I2SSettingsSTM32 *settings,void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
/// Start to receive I2S data
bool startI2SReceive(I2SSettingsSTM32 *settings,void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
/// Start to receive and transmit I2S data
bool startI2STransmitReceive(I2SSettingsSTM32 *settings, void (*readToTransmit)(uint8_t *buffer, uint16_t byteCount), void (*writeFromReceive)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
/// Stop I2S processing
void stopI2S(I2SSettingsSTM32 *settings);



#ifdef __cplusplus
}
#endif
