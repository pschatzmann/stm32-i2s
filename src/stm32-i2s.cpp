
#include "stm32-i2s.h"
#include "Arduino.h"
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool is_error;
/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_i2s3_ext_rx;
DMA_HandleTypeDef hdma_i2s3_ext_tx;

void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount) = NULL;
void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount) = NULL;
void STM32_LOG(const char *fmt, ...);
void Report_Error();

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2S3_Init_Ext(I2SSettingsSTM32 *settings);

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
    Report_Error();
  }
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
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
 * @brief GPIO Initialization Function for I2S_CKIN
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
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

  /**I2S3 GPIO Configuration
  PA4     ------> I2S3_WS
  PB10     ------> I2S3_MCK
  PB3     ------> I2S3_CK
  PB4     ------> I2S3_ext_SD
  PB5     ------> I2S3_SD
  */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_I2S3ext;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/// Copy of MX_I2S3_Init which allows some default parameters
static void MX_I2S3_Init_Ext(I2SSettingsSTM32 *settings) {

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = settings->mode;
  hi2s3.Init.Standard = settings->standard;
  hi2s3.Init.FullDuplexMode = settings->fullduplexmode;
  hi2s3.Init.AudioFreq = settings->sample_rate;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
    Report_Error();
  }
}

/// Starts the i2s processing
bool i2s_begin(I2SSettingsSTM32 *settings) {
  // default values
  if (settings->mode == 0) {
    settings->mode = I2S_MODE_MASTER_TX;
  }
  if (settings->standard == 0) {
    settings->standard = I2S_STANDARD_PHILIPS;
  }
  if (settings->fullduplexmode == 0) {
    settings->fullduplexmode = I2S_FULLDUPLEXMODE_ENABLE;
  }
  if (settings->sample_rate == 0) {
    settings->sample_rate = I2S_AUDIOFREQ_44K;
  }
  if (settings->i2s == NULL) {
    settings->i2s = &hi2s3;
  }
  is_error = false;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  // HAL_Init();// Not needed -> called by Arduino
  /* Configure the system clock */
  // SystemClock_Config(); // Not needed -> called by Arduino
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init_Ext(settings);
  return !is_error;
}

void stopI2S(I2SSettingsSTM32 *settings) {
  I2S_HandleTypeDef *i2s = settings->i2s;
  HAL_I2S_DMAStop(i2s);
  HAL_I2S_DeInit(i2s);
  HAL_I2S_MspDeInit(i2s);
  if (i2s->pTxBuffPtr != NULL) {
    free(i2s->pTxBuffPtr);
  }
  if (i2s->pRxBuffPtr != NULL) {
    free(i2s->pRxBuffPtr);
  }
}

bool startI2STransmit(I2SSettingsSTM32 *settings,
                      void (*readToTransmit)(uint8_t *buffer,
                                             uint16_t byteCount),
                      uint16_t buffer_size) {
  readToTransmitCB = readToTransmit;
  void *dma_buffer_tx = calloc(1, buffer_size);
  bool result = true;
  if (!i2s_begin(settings)) {
    return false;
  }
  // start circular dma
  if (HAL_I2S_Transmit_DMA(settings->i2s, (uint16_t *)dma_buffer_tx,
                           buffer_size) != HAL_OK) {
    // LOGE("HAL_I2S_Transmit_DMA");
    Report_Error();
    result = false;
  }
  return result;
}

bool startI2SReceive(I2SSettingsSTM32 *settings,
                     void (*writeFromReceive)(uint8_t *buffer,
                                              uint16_t byteCount),
                     uint16_t buffer_size) {
  bool result = true;
  writeFromReceiveCB = writeFromReceive;
  void *dma_buffer_rx = calloc(1, buffer_size);
  if (!i2s_begin(settings)) {
    return false;
  }
  // start circular dma
  if (HAL_I2S_Receive_DMA(settings->i2s, (uint16_t *)dma_buffer_rx,
                          buffer_size) != HAL_OK) {
    // LOGE("HAL_I2S_Transmit_DMA");
    Report_Error();
    result = false;
  }
  return result;
}

bool startI2STransmitReceive(I2SSettingsSTM32 *settings,
                             void (*readToTransmit)(uint8_t *buffer,
                                                    uint16_t byteCount),
                             void (*writeFromReceive)(uint8_t *buffer,
                                                      uint16_t byteCount),
                             uint16_t buffer_size) {
  bool result = true;
  readToTransmitCB = readToTransmit;
  void *dma_buffer_tx = calloc(1, buffer_size);
  writeFromReceiveCB = writeFromReceive;
  void *dma_buffer_rx = calloc(1, buffer_size);
  if (!i2s_begin(settings)) {
    return false;
  }
  if (HAL_I2SEx_TransmitReceive_DMA(settings->i2s, (uint16_t *)dma_buffer_tx,
                                    (uint16_t *)dma_buffer_rx,
                                    buffer_size) != HAL_OK) {
    // LOGE("HAL_I2S_Transmit_DMA");
    Report_Error();
    result = false;
  }
  return result;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
  // second half finished, filling it up again while first  half is playing
  uint8_t *dma_buffer_tx = (uint8_t *)hi2s->pTxBuffPtr;
  uint8_t *dma_buffer_rx = (uint8_t *)hi2s->pRxBuffPtr;
  uint16_t buffer_size_tx = hi2s->TxXferSize * 2; // XferSize is in words
  uint16_t buffer_size_rx = hi2s->RxXferSize * 2;
  if (readToTransmitCB != NULL)
    readToTransmitCB(&(dma_buffer_tx[buffer_size_tx / 2]), buffer_size_tx / 2);
  if (writeFromReceiveCB != NULL)
    writeFromReceiveCB(&(dma_buffer_rx[buffer_size_rx / 2]),
                       buffer_size_rx / 2);
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  // second half finished, filling it up again while first  half is playing
  uint8_t *dma_buffer_tx = (uint8_t *)hi2s->pTxBuffPtr;
  uint8_t *dma_buffer_rx = (uint8_t *)hi2s->pRxBuffPtr;
  uint16_t buffer_size_tx = hi2s->TxXferSize * 2; // XferSize is in words
  uint16_t buffer_size_rx = hi2s->RxXferSize * 2;
  if (readToTransmitCB != NULL)
    readToTransmitCB(&(dma_buffer_tx[0]), buffer_size_tx / 2);
  if (writeFromReceiveCB != NULL)
    writeFromReceiveCB(&(dma_buffer_rx[0]), buffer_size_rx / 2);
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

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) { Report_Error(); }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Report_Error() {
  is_error = true;
  STM32_LOG("%s", "stm32-i2s: Error");
}

/**
 * @brief I2S MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2s: I2S handle pointer
 * @retval None
 */
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (hi2s->Instance == SPI3) {
    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      Report_Error();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    /* I2S3 DMA Init */
    /* I2S3_EXT_RX Init */
    hdma_i2s3_ext_rx.Instance = DMA1_Stream0;
    hdma_i2s3_ext_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_i2s3_ext_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2s3_ext_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s3_ext_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s3_ext_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s3_ext_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s3_ext_rx.Init.Mode = DMA_CIRCULAR;
    hdma_i2s3_ext_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2s3_ext_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2s3_ext_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_i2s3_ext_rx.Init.MemBurst = DMA_MBURST_INC4;
    hdma_i2s3_ext_rx.Init.PeriphBurst = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&hdma_i2s3_ext_rx) != HAL_OK) {
      Report_Error();
    }

    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2s3_ext_rx);

    /* SPI3_TX Init */
    hdma_i2s3_ext_tx.Instance = DMA1_Stream5;
    hdma_i2s3_ext_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_i2s3_ext_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2s3_ext_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s3_ext_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s3_ext_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s3_ext_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s3_ext_tx.Init.Mode = DMA_CIRCULAR;
    hdma_i2s3_ext_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2s3_ext_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2s3_ext_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_i2s3_ext_tx.Init.MemBurst = DMA_PBURST_INC4;
    hdma_i2s3_ext_tx.Init.PeriphBurst = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&hdma_i2s3_ext_tx) != HAL_OK) {
      Report_Error();
    }

    __HAL_LINKDMA(hi2s, hdmatx, hdma_i2s3_ext_tx);
  }
}

/**
 * @brief I2S MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2s: I2S handle pointer
 * @retval None
 */
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s) {
  if (hi2s->Instance == SPI3) {
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**I2S3 GPIO Configuration
    PA4     ------> I2S3_WS
    PB10     ------> I2S3_MCK
    PB3     ------> I2S3_CK
    PB4     ------> I2S3_ext_SD
    PB5     ------> I2S3_SD
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* I2S3 DMA DeInit */
    HAL_DMA_DeInit(hi2s->hdmarx);
    HAL_DMA_DeInit(hi2s->hdmatx);
  }
}

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2s3_ext_rx); }

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2s3_ext_tx); }

/**
 * @brief Write log output to Serial
 */
void STM32_LOG(const char *fmt, ...) {
  char log_buffer[200];
  strcpy(log_buffer, "STM32: ");
  va_list arg;
  va_start(arg, fmt);
  int len = vsnprintf(log_buffer + 7, 200, fmt, arg);
  va_end(arg);
  Serial.println(log_buffer);
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  STM32_LOG("stm32-i2s: Wrong parameters value: file %s on line %d", file,
            line);
}
#endif /* USE_FULL_ASSERT */
