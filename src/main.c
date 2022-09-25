/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
boolean is_error;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_i2s3_ext_rx;
DMA_HandleTypeDef hdma_spi3_tx;

/* USER CODE BEGIN PV */

void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount)=NULL;
void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount)=NULL;
void STM32_LOG(const char *fmt, ...);
void Error_Handler(void);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
static void MX_I2S3_Init_Ext(I2SSettingsSTM32 *settings);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// WARDNING: The SystemClock_Config leads to conflicts, so we rename it to make sure that it is not used!
//#define SystemClock_Config NA_SystemClock_Config

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/// Copy of MX_I2S3_Init which allows some default parameters
static void MX_I2S3_Init_Ext(I2SSettingsSTM32 *settings)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = settings->mode;
  hi2s3.Init.Standard = settings->standard;
  hi2s3.Init.FullDuplexMode = settings->fullduplexmode;
  hi2s3.Init.AudioFreq = settings->sample_rate; //I2S_AUDIOFREQ_44K; //I2S_AUDIOFREQ_8K;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
}

// Undo rename of SystemClock_Config method
#undef SystemClock_Config

/// Starts the i2s processing
boolean i2s_begin(I2SSettingsSTM32 *settings)
{
  // default values
  if (settings->mode ==0){
     settings->mode = I2S_MODE_MASTER_TX;
  }
  if (settings->standard==0){
     settings->standard = I2S_STANDARD_PHILIPS;
  }
  if (settings->fullduplexmode==0){
     settings->fullduplexmode = I2S_FULLDUPLEXMODE_ENABLE;
  }
  if (settings->sample_rate==0){
     settings->sample_rate = I2S_AUDIOFREQ_44K;
  }
  if (settings->i2s==NULL){
     settings->i2s = &hi2s3;
  }
  is_error = false;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();// Not needed -> called by Arduino
  /* Configure the system clock */
  //SystemClock_Config(); // Not needed -> called by Arduino
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init_Ext(settings);
  return !is_error;
}

boolean startI2STransmit(I2SSettingsSTM32 *settings, void (*readToTransmit)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size) {
	readToTransmitCB = readToTransmit;
  void *dma_buffer_tx = calloc(1, buffer_size);
  boolean result = true;
  if (!i2s_begin(settings)){
    return false;
  }
	// start circular dma
	if (HAL_I2S_Transmit_DMA(settings->i2s, (uint16_t*) dma_buffer_tx, buffer_size)!=HAL_OK){
		//LOGE("HAL_I2S_Transmit_DMA");
    Error_Handler();
    result = false;
	}
  return result;
}

boolean startI2SReceive(I2SSettingsSTM32 *settings, void (*writeFromReceive)(uint8_t *buffer, uint16_t byteCount),uint16_t buffer_size) {
  boolean result = true;
  writeFromReceiveCB = writeFromReceive;
  void *dma_buffer_rx = calloc(1, buffer_size);
  if (!i2s_begin(settings)){
    return false;
  }
	// start circular dma
	if (HAL_I2S_Receive_DMA(settings->i2s, (uint16_t*) dma_buffer_rx, buffer_size)!=HAL_OK){
		//LOGE("HAL_I2S_Transmit_DMA");
    Error_Handler();
    result = false;
	}
  return result;
}

boolean startI2STransmitReceive(I2SSettingsSTM32 *settings, void (*readToTransmit)(uint8_t *buffer, uint16_t byteCount), void (*writeFromReceive)(uint8_t *buffer, uint16_t byteCount), uint16_t buffer_size) {
  boolean result = true;
	readToTransmitCB = readToTransmit;
  void *dma_buffer_tx = calloc(1, buffer_size);
  writeFromReceiveCB = writeFromReceive;
  void *dma_buffer_rx = calloc(1, buffer_size);
  if (!i2s_begin(settings)){
    return false;
  }
  if (HAL_I2SEx_TransmitReceive_DMA(settings->i2s, (uint16_t*) dma_buffer_tx, (uint16_t*) dma_buffer_rx, buffer_size)!=HAL_OK){
		//LOGE("HAL_I2S_Transmit_DMA");
    Error_Handler();
    result = false;
	}
  return result;
}

void stopI2S(I2S_HandleTypeDef *i2s) {
  HAL_I2S_DMAStop(i2s);
  HAL_I2S_DeInit(i2s);
  HAL_I2S_MspDeInit(i2s);
  if (i2s->pTxBuffPtr!=NULL){
    free(i2s->pTxBuffPtr);
  }
  if (i2s->pRxBuffPtr!=NULL){
    free(i2s->pRxBuffPtr);
  }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
	// second half finished, filling it up again while first  half is playing
  uint8_t* dma_buffer_tx = (uint8_t*)hi2s->pTxBuffPtr;
  uint8_t* dma_buffer_rx = (uint8_t*)hi2s->pRxBuffPtr;
  uint16_t buffer_size_tx = hi2s->TxXferSize;
  uint16_t buffer_size_rx = hi2s->RxXferSize;
	if (readToTransmitCB!=NULL) readToTransmitCB(&(dma_buffer_tx[buffer_size_tx >> 1]), buffer_size_tx >> 1);
  if (writeFromReceiveCB!=NULL) writeFromReceiveCB(&(dma_buffer_rx[buffer_size_rx >> 1]), buffer_size_rx >> 1);
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	// second half finished, filling it up again while first  half is playing
  uint8_t* dma_buffer_tx = (uint8_t*)hi2s->pTxBuffPtr;
  uint8_t* dma_buffer_rx = (uint8_t*)hi2s->pRxBuffPtr;
  uint16_t buffer_size_tx = hi2s->TxXferSize;
  uint16_t buffer_size_rx = hi2s->RxXferSize;
	if (readToTransmitCB!=NULL) readToTransmitCB(&(dma_buffer_tx[buffer_size_tx >> 1]), buffer_size_tx >> 1);
  if (writeFromReceiveCB!=NULL) writeFromReceiveCB(&(dma_buffer_rx[buffer_size_rx >> 1]), buffer_size_rx >> 1);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  HAL_I2SEx_TxRxCpltCallback(hi2s);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  HAL_I2SEx_TxRxHalfCpltCallback(hi2s);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  HAL_I2SEx_TxRxCpltCallback(hi2s);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  HAL_I2SEx_TxRxHalfCpltCallback(hi2s);
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
  Error_Handler();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  is_error = true;
  STM32_LOG("%s","stm32-i2s: Error");
  // assert(0);
  // __disable_irq();
  // while (1)
  // {
  // }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  STM32_LOG("stm32-i2s: Wrong parameters value: file %s on line %d", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
