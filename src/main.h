/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup I2S
  * @{
  */ 

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef ARDUINO
#undef Error_Handler
#endif

#define I2S_BUFFER_SIZE 512

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#ifndef __cplusplus
typedef int boolean;
#endif
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

extern I2S_HandleTypeDef hi2s3;

// supported parameters
typedef struct I2SSettingsSTM32 {
  uint32_t mode;
  uint32_t standard;
  uint32_t fullduplexmode;
  uint32_t sample_rate;
  I2S_HandleTypeDef *i2s;
} I2SSettingsSTM32;

/// Start to transmit I2S data
boolean startI2STransmit(I2SSettingsSTM32 *settings,void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
/// Start to receive I2S data
boolean startI2SReceive(I2SSettingsSTM32 *settings,void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
/// Start to receive and transmit I2S data
boolean startI2STransmitReceive(I2SSettingsSTM32 *settings, void (*readToTransmit)(uint8_t *buffer, uint16_t byteCount), void (*writeFromReceive)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
/// Stop I2S processing
void stopI2S();


extern void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount);
extern void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */


/**
  * @}
  */ 


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
