#pragma once
#include "stm32f4xx_hal.h"

// Relevant Callbacks
extern "C" void DMA1_Stream0_IRQHandler(void);
extern "C" void DMA1_Stream0_IRQHandler(void);
extern "C" void DMA1_Stream5_IRQHandler(void);
extern "C" void HAL_MspInit(void);
extern "C" void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_DMA_IRQHandler_i2s3();
extern "C" void HAL_DMA_IRQHandler_spi3();

// Forward Declaration
class I2SWithCallbacks;
class I2SWithCallbacks *SelfI2S = nullptr;
extern I2S_HandleTypeDef hi2s3;

/**
 * @brief I2S Parameters
 *
 */
struct I2SSettingsSTM32 {
  uint32_t mode;
  uint32_t standard;
  uint32_t sample_rate;
  uint16_t buffer_size = 1024;

  void (*writeCB)(uint8_t *buffer, uint16_t byteCount) = nullptr;
  void (*readCB)(uint8_t *buffer, uint16_t byteCount) = nullptr;

  I2S_HandleTypeDef *i2s;
};

/**
 * @brief I2S class
 *
 */
class I2SWithCallbacks {
  friend void DMA1_Stream0_IRQHandler(void);
  friend void DMA1_Stream0_IRQHandler(void);
  friend void DMA1_Stream5_IRQHandler(void);
  friend void HAL_MspInit(void);
  friend void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_DMA_IRQHandler_i2s3();
  friend void HAL_DMA_IRQHandler_spi3();

public:
  I2SWithCallbacks() { SelfI2S = this; }

  /// Starts the i2s processing
  bool begin(I2SSettingsSTM32 settings) {
    this->settings = settings;
    // default values
    if (settings.mode == 0) {
      settings.mode = I2S_MODE_MASTER_TX;
    }
    if (settings.standard == 0) {
      settings.standard = I2S_STANDARD_PHILIPS;
    }
    if (settings.writeCB && settings.readCB) {
      fullduplexmode = I2S_FULLDUPLEXMODE_ENABLE;
    } else {
      fullduplexmode = I2S_FULLDUPLEXMODE_DISABLE;
    }
    if (settings.sample_rate == 0) {
      settings.sample_rate = I2S_AUDIOFREQ_44K;
    }
    if (settings.i2s == nullptr) {
      settings.i2s = &hi2s3;
    }
    is_error = false;

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2S3_Init_Ext(settings);

    if (is_error) {
      return false;
    }

    // Start DMA
    bool result = true;
    if (settings.writeCB && settings.readCB) {
      result = transmitReceive();
    } else if (settings.writeCB) {
      result = transmit();
    } else if (settings.readCB) {
      result = receive();
    }
    return result;
  }

  void end() {
    HAL_I2S_DMAStop(settings.i2s);
    HAL_I2S_DeInit(settings.i2s);
    HAL_I2S_MspDeInit(settings.i2s);
    if (settings.i2s->pTxBuffPtr != nullptr) {
      free(settings.i2s->pTxBuffPtr);
    }
    if (settings.i2s->pRxBuffPtr != nullptr) {
      free(settings.i2s->pRxBuffPtr);
    }
  }

protected:
  I2SSettingsSTM32 settings;
  DMA_HandleTypeDef hdma_i2s3_ext_rx;
  DMA_HandleTypeDef hdma_spi3_tx;
  bool is_error = false;
  uint32_t fullduplexmode;

  void ErrorHandler(void) {
    is_error = true;
    // STM32_LOG("%s", "stm32-i2s: Error");
  }

  bool transmit() {
    void *dma_buffer_tx = calloc(1, settings.buffer_size);
    bool result = true;
    // start circular dma
    if (HAL_I2S_Transmit_DMA(settings.i2s, (uint16_t *)dma_buffer_tx,
                             settings.buffer_size) != HAL_OK) {
      // LOGE("HAL_I2S_Transmit_DMA");
      ErrorHandler();
      result = false;
    }
    return result;
  }

  bool receive() {
    bool result = true;
    void *dma_buffer_rx = calloc(1, settings.buffer_size);
    // start circular dma
    if (HAL_I2S_Receive_DMA(settings.i2s, (uint16_t *)dma_buffer_rx,
                            settings.buffer_size) != HAL_OK) {
      // LOGE("HAL_I2S_Transmit_DMA");
      ErrorHandler();
      result = false;
    }
    return result;
  }

  bool transmitReceive() {
    bool result = true;
    void *dma_buffer_tx = calloc(1, settings.buffer_size);
    void *dma_buffer_rx = calloc(1, settings.buffer_size);
    if (HAL_I2SEx_TransmitReceive_DMA(settings.i2s, (uint16_t *)dma_buffer_tx,
                                      (uint16_t *)dma_buffer_rx,
                                      settings.buffer_size) != HAL_OK) {
      ErrorHandler();
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
    if (settings.readCB != nullptr)
      settings.readCB(&(dma_buffer_tx[buffer_size_tx / 2]), buffer_size_tx / 2);
    if (settings.writeCB != nullptr)
      settings.writeCB(&(dma_buffer_rx[buffer_size_rx / 2]),
                       buffer_size_rx / 2);
  }

  void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    // second half finished, filling it up again while first  half is playing
    uint8_t *dma_buffer_tx = (uint8_t *)hi2s->pTxBuffPtr;
    uint8_t *dma_buffer_rx = (uint8_t *)hi2s->pRxBuffPtr;
    uint16_t buffer_size_tx = hi2s->TxXferSize * 2; // XferSize is in words
    uint16_t buffer_size_rx = hi2s->RxXferSize * 2;
    if (settings.readCB != nullptr)
      settings.readCB(&(dma_buffer_tx[0]), buffer_size_tx / 2);
    if (settings.writeCB != nullptr)
      settings.writeCB(&(dma_buffer_rx[0]), buffer_size_rx / 2);
  }

  /**
   * @brief I2S3 Initialization Function
   * @param None
   * @retval None
   */
  void MX_I2S3_Init(void) {
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
      ErrorHandler();
    }
  }

  /**
   * Enable DMA controller clock
   */
  void MX_DMA_Init(void) {

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
  void MX_GPIO_Init(void) {
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

  /// Copy of MX_I2S3_Init which allows some default parameters
  void MX_I2S3_Init_Ext(I2SSettingsSTM32 &settings) {
    hi2s3.Instance = SPI3;
    hi2s3.Init.Mode = settings.mode;
    hi2s3.Init.Standard = settings.standard;
    hi2s3.Init.FullDuplexMode = fullduplexmode;
    hi2s3.Init.AudioFreq =
        settings.sample_rate; // I2S_AUDIOFREQ_44K; //I2S_AUDIOFREQ_8K;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
      ErrorHandler();
    }
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
        ErrorHandler();
      }

      /* Peripheral clock enable */
      __HAL_RCC_SPI3_CLK_ENABLE();

      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
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
        ErrorHandler();
      }

      __HAL_LINKDMA(hi2s, hdmarx, hdma_i2s3_ext_rx);

      /* SPI3_TX Init */
      hdma_spi3_tx.Instance = DMA1_Stream5;
      hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
      hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
      hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
      hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hdma_spi3_tx.Init.Mode = DMA_CIRCULAR;
      hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
      hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
      hdma_spi3_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
      hdma_spi3_tx.Init.MemBurst = DMA_PBURST_INC4;
      hdma_spi3_tx.Init.PeriphBurst = DMA_PBURST_INC4;
      if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) {
        ErrorHandler();
      }

      __HAL_LINKDMA(hi2s, hdmatx, hdma_spi3_tx);
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

      HAL_GPIO_DeInit(GPIOB,
                      GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

      /* I2S3 DMA DeInit */
      HAL_DMA_DeInit(hi2s->hdmarx);
      HAL_DMA_DeInit(hi2s->hdmatx);
    }
  }

  void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriority(FPU_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FPU_IRQn);
  }

  void HAL_DMA_IRQHandler_i2s3() { HAL_DMA_IRQHandler(&hdma_i2s3_ext_rx); }
  void HAL_DMA_IRQHandler_spi3() { HAL_DMA_IRQHandler(&hdma_spi3_tx); }
};

// Callback Functions

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2SEx_TxRxCpltCallback(hi2s);
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2SEx_TxRxHalfCpltCallback(hi2s);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2SEx_TxRxCpltCallback(hi2s);
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2SEx_TxRxHalfCpltCallback(hi2s);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2SEx_TxRxCpltCallback(hi2s);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2SEx_TxRxHalfCpltCallback(hi2s);
}

void HAL_MspInit(void) {
  if (SelfI2S)
    SelfI2S->HAL_MspInit();
}

void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2S_MspInit(hi2s);
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s) {
  if (SelfI2S)
    SelfI2S->HAL_I2S_MspDeInit(hi2s);
}

void SysTick_Handler(void) { HAL_IncTick(); }

void DMA1_Stream0_IRQHandler(void) {
  if (SelfI2S)
    SelfI2S->HAL_DMA_IRQHandler_i2s3();
}

void DMA1_Stream5_IRQHandler(void) {
  if (SelfI2S)
    SelfI2S->HAL_DMA_IRQHandler_spi3();
}
