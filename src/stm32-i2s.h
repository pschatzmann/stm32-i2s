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
#pragma once

#define I2S_BUFFER_SIZE 512
#define STM32_I2S_WITH_OBJECT
#define USE_FULL_ASSERT

#include "Arduino.h"
#include "stm32-pins.h"
#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" void DMA1_Stream0_IRQHandler(void);
extern "C" void DMA1_Stream5_IRQHandler(void);
extern "C" void Report_Error();
extern "C" void STM32_LOG(const char *fmt, ...);
static bool is_error;

using byte = uint8_t;

enum I2SPinFunction { mclk, bck, ws, data_out, data_in };

struct I2SPin {
  I2SPin(I2SPinFunction f, PinName n, int alt) {}
  I2SPinFunction function;
  PinName pin;
  int altFunction;
};

/**
 * @brief Processor specific settings
 */
struct HardwareConfig {
  IRQn_Type irq1;
  IRQn_Type irq2;

  uint32_t plln = 0;
  uint32_t pllm = 0;
  uint32_t pllr = 0;

  DMA_Stream_TypeDef *rx_instance = nullptr;
  uint32_t rx_channel;
  uint32_t rx_direction;

  DMA_Stream_TypeDef *tx_instance = nullptr;
  uint32_t tx_channel;
  uint32_t tx_direction;

  I2SPin pins[5] = STM_I2S_PINS;

  HardwareConfig() {
    irq1 = DMA1_Stream0_IRQn;
    irq2 = DMA1_Stream5_IRQn;

    rx_instance = DMA1_Stream0;
    rx_channel = DMA_CHANNEL_3;
    rx_direction = DMA_PERIPH_TO_MEMORY;

    tx_instance = DMA1_Stream5;
    tx_channel = DMA_CHANNEL_0;
    tx_direction = DMA_MEMORY_TO_PERIPH;

#ifdef BLACK_PILL
    plln = 192;
    pllm = 16;
    pllr = 2;
#endif
#ifdef STM32F411DISCO
    plln = 200;
    pllm = 5;
    pllr = 2;
#endif
  }
};

/**
 * @brief Currently supported parameters
 */
struct I2SSettingsSTM32 {
  uint32_t mode = I2S_MODE_MASTER_TX;
  uint32_t standard = I2S_STANDARD_PHILIPS;
  uint32_t fullduplexmode = I2S_FULLDUPLEXMODE_ENABLE;
  uint32_t sample_rate = I2S_AUDIOFREQ_44K;
  HardwareConfig hardware_config;
};

/**
 * I2S API for STM32
 */
class Stm32I2sClass {
public:
  /// Start to transmit I2S data
  bool startI2STransmit(I2SSettingsSTM32 settings,
                        void (*readToTransmit)(uint8_t *buffer,
                                               uint16_t byteCount),
                        uint16_t buffer_size) {
    this->settings = settings;
    this->hw = settings.hardware_config;
    readToTransmitCB = readToTransmit;
    if (dma_buffer_tx == nullptr)
      dma_buffer_tx = new byte[buffer_size];
    bool result = true;
    if (!i2s_begin()) {
      return false;
    }
    // start circular dma
    if (HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dma_buffer_tx, buffer_size) !=
        HAL_OK) {
      // LOGE("HAL_I2S_Transmit_DMA");
      Report_Error();
      result = false;
    }
    return result;
  }

  /// Start to receive I2S data
  bool startI2SReceive(I2SSettingsSTM32 settings,
                       void (*writeFromReceive)(uint8_t *buffer,
                                                uint16_t byteCount),
                       uint16_t buffer_size) {
    this->settings = settings;
    this->hw = settings.hardware_config;
    bool result = true;
    writeFromReceiveCB = writeFromReceive;
    if (dma_buffer_rx == nullptr)
      dma_buffer_rx = new byte[buffer_size];
    if (!i2s_begin()) {
      return false;
    }
    // start circular dma
    if (HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)dma_buffer_rx, buffer_size) !=
        HAL_OK) {
      // LOGE("HAL_I2S_Transmit_DMA");
      Report_Error();
      result = false;
    }
    return result;
  }

  /// Start to receive and transmit I2S data
  bool startI2STransmitReceive(I2SSettingsSTM32 settings,
                               void (*readToTransmit)(uint8_t *buffer,
                                                      uint16_t byteCount),
                               void (*writeFromReceive)(uint8_t *buffer,
                                                        uint16_t byteCount),
                               uint16_t buffer_size) {
    this->settings = settings;
    this->hw = settings.hardware_config;
    bool result = true;
    readToTransmitCB = readToTransmit;
    writeFromReceiveCB = writeFromReceive;

    if (dma_buffer_tx == nullptr)
      dma_buffer_tx = new byte[buffer_size];

    if (dma_buffer_rx == nullptr)
      dma_buffer_rx = new byte[buffer_size];

    if (!i2s_begin()) {
      return false;
    }
    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)dma_buffer_tx,
                                      (uint16_t *)dma_buffer_rx,
                                      buffer_size) != HAL_OK) {
      // LOGE("HAL_I2S_Transmit_DMA");
      Report_Error();
      result = false;
    }
    return result;
  }

  void stopI2S() {
    HAL_I2S_DMAStop(&hi2s3);
    HAL_I2S_DeInit(&hi2s3);
    HAL_I2S_MspDeInit(&hi2s3);
    if (dma_buffer_tx != NULL) {
      delete[](dma_buffer_tx);
    }
    if (dma_buffer_rx != NULL) {
      delete[](dma_buffer_rx);
    }
  }

  void txRxCpltCallback(I2S_HandleTypeDef *hi2s) {
    // second half finished, filling it up again while first  half is playing
    uint8_t *dma_buffer_tx = (uint8_t *)hi2s->pTxBuffPtr;
    uint8_t *dma_buffer_rx = (uint8_t *)hi2s->pRxBuffPtr;
    uint16_t buffer_size_tx = hi2s->TxXferSize * 2; // XferSize is in words
    uint16_t buffer_size_rx = hi2s->RxXferSize * 2;
    if (readToTransmitCB != NULL)
      readToTransmitCB(&(dma_buffer_tx[buffer_size_tx / 2]),
                       buffer_size_tx / 2);
    if (writeFromReceiveCB != NULL)
      writeFromReceiveCB(&(dma_buffer_rx[buffer_size_rx / 2]),
                         buffer_size_rx / 2);
  }

  void txRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
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

  inline void dmaIrqRx() { HAL_DMA_IRQHandler(&hdma_i2s3_ext_rx); }

  inline void dmaIrqTx() { HAL_DMA_IRQHandler(&hdma_i2s3_ext_tx); }

  inline void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) { i2s_MspInit(hi2s); }

  inline void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s) {
    i2s_MspDeInit(hi2s);
  }

protected:
  I2SSettingsSTM32 settings;
  I2S_HandleTypeDef hi2s3;
  byte *dma_buffer_tx = nullptr;
  byte *dma_buffer_rx = nullptr;
  void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount);
  void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount);
  DMA_HandleTypeDef hdma_i2s3_ext_rx;
  DMA_HandleTypeDef hdma_i2s3_ext_tx;
  HardwareConfig hw;

  // example call: pinModeAF(PD12, GPIO_AF2_TIM4);
  void pinModeAltFunction(PinName pn, uint32_t Alternate) {
    // int pn = digitalPinToPinName(ulPin);

    if (STM_PIN(pn) < 8) {
      LL_GPIO_SetAFPin_0_7(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn),
                           Alternate);
    } else {
      LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn),
                            Alternate);
    }

    LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn),
                       LL_GPIO_MODE_ALTERNATE); // STM_MODE_AF_PP
  }

  /// Starts the i2s processing
  bool i2s_begin() {
    is_error = false;
    if (settings.sample_rate == 0) {
      STM32_LOG("sample_rate must not be 0");
      return false;
    }
    is_error = false;
    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick.
     */
    // HAL_Init();// Not needed -> called by Arduino
    /* Configure the system clock */
    // SystemClock_Config(); // Not needed -> called by Arduino
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2S3_Init();
    return !is_error;
  }

  /**
   * @brief GPIO Initialization Function for I2S_CKIN
   * @param None
   * @retval None
   */
  virtual void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Define pins
    for (I2SPin &pin : hw.pins) {
      pinModeAltFunction(pin.pin, pin.altFunction);
    }
  }

  /**
   * Enable DMA controller clock
   */
  virtual void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(hw.irq1, 0, 0);
    HAL_NVIC_EnableIRQ(hw.irq1);
    /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(hw.irq2, 0, 0);
    HAL_NVIC_EnableIRQ(hw.irq2);
  }

  /**
   * @brief I2S3 Initialization Function
   * @param None
   * @retval None
   */
  virtual void MX_I2S3_Init(void) {
    hi2s3.Instance = SPI3;
    hi2s3.Init.Mode = settings.mode;
    hi2s3.Init.Standard = settings.standard;
    hi2s3.Init.FullDuplexMode = settings.fullduplexmode;
    hi2s3.Init.AudioFreq = settings.sample_rate;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
      Report_Error();
    }
  }

  /**
   * @brief I2S MSP Initialization
   * This function configures the hardware resources used in this example
   * @param hi2s: I2S handle pointer
   * @retval None
   */
  virtual void i2s_MspInit(I2S_HandleTypeDef *hi2s) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (hi2s->Instance == SPI3) {
      /**
       * Initializes the peripherals clock
       */
      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
      PeriphClkInitStruct.PLLI2S.PLLI2SN = hw.plln; // 192;
#ifdef HAS_PLLI2SM
      PeriphClkInitStruct.PLLI2S.PLLI2SM = hw.pllm; // 16;
#endif
      PeriphClkInitStruct.PLLI2S.PLLI2SR = hw.pllr; // 2;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Report_Error();
      }

      /* Peripheral clock enable */
      __HAL_RCC_SPI3_CLK_ENABLE();

      DMA_Stream_TypeDef *rx_instance;
      uint32_t rx_channel;
      uint32_t rx_direction;

      /* I2S3 DMA Init */
      if (dma_buffer_rx) {
        setupDMA(hdma_i2s3_ext_rx, hw.rx_instance, hw.rx_channel,
                 hw.rx_direction);
        __HAL_LINKDMA(hi2s, hdmarx, hdma_i2s3_ext_rx);
      }

      if (dma_buffer_tx) {
        setupDMA(hdma_i2s3_ext_tx, hw.tx_instance, hw.tx_channel,
                 hw.tx_direction);
        __HAL_LINKDMA(hi2s, hdmatx, hdma_i2s3_ext_tx);
      }
    }
  }

  void setupDMA(DMA_HandleTypeDef &dma, DMA_Stream_TypeDef *instance,
                uint32_t channel, uint32_t direction) {
    dma.Instance = instance;
    dma.Init.Channel = channel;
    dma.Init.Direction = direction;
    dma.Init.PeriphInc = DMA_PINC_DISABLE;
    dma.Init.MemInc = DMA_MINC_ENABLE;
    dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma.Init.Mode = DMA_CIRCULAR;
    dma.Init.Priority = DMA_PRIORITY_HIGH;
    dma.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    dma.Init.MemBurst = DMA_PBURST_INC4;
    dma.Init.PeriphBurst = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&dma) != HAL_OK) {
      Report_Error();
    }
  }

  /**
   * @brief I2S MSP De-Initialization
   * This function freeze the hardware resources used in this example
   * @param hi2s: I2S handle pointer
   * @retval None
   */
  virtual void i2s_MspDeInit(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI3) {
      /* Peripheral clock disable */
      __HAL_RCC_SPI3_CLK_DISABLE();

      /**I2S3 GPIO Configuration
      PA2     -------> I2S_CKIN
      PA4     ------> I2S3_WS
      PB10     ------> I2S3_MCK
      PB3     ------> I2S3_CK
      PB4     ------> I2S3_ext_SD
      PB5     ------> I2S3_SD
      */
      // HAL_GPIO_DeInit(GPIOA, hw.a_pins);
      // HAL_GPIO_DeInit(GPIOB, hw.b_pins);
      // HAL_GPIO_DeInit(GPIOD, hw.d_pins);

      /* I2S3 DMA DeInit */
      HAL_DMA_DeInit(hi2s->hdmarx);
      HAL_DMA_DeInit(hi2s->hdmatx);
    }
  }
};

static Stm32I2sClass I2S;
