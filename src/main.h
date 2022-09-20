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
#define I2S_BUFFER_SIZE 512

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void startI2STransmit(I2S_HandleTypeDef *i2s,void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
void startI2SReceive(I2S_HandleTypeDef *i2s,void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
void startI2STransmitReceive(I2S_HandleTypeDef *i2s, void (*readToTransmit)(uint8_t *buffer, uint16_t byteCount), void (*writeFromReceive)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size);
void stopI2S();

extern I2S_HandleTypeDef hi2s3;
extern uint32_t i2s_default_mode;
extern uint32_t i2s_default_standard;
extern uint32_t i2s_default_fullduplexmode;
extern uint32_t i2s_default_samplerate;

extern void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount);
extern void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
