/**
 * @file stm32-i2s.h
 * @author phil Schatzmann
 * @brief Main entry point header for this library
 * @version 0.1
 * @date 2022-09-22
 *
 * @copyright Copyright (c) 2022
 */
#pragma once

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Arduino.h"
#include "PinConfigured.h"
#include "stm32-config-i2s.h"
#include "stm32f4xx_hal.h"

extern uint32_t g_anOutputPinConfigured[MAX_NB_PORT];

namespace stm32_i2s {

using byte = uint8_t;

extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
extern "C" void DMA1_Stream0_IRQHandler(void);
extern "C" void DMA1_Stream5_IRQHandler(void);
extern "C" void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
extern "C" void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
extern "C" void Report_Error();
extern "C" void STM32_LOG(const char *fmt, ...);
extern bool stm32_i2s_is_error;

/// @brief i2s pin function used as documentation
enum I2SPinFunction { mclk, bck, ws, data_out, data_in };

/**
 * @brief Define individual Pin. This is used to set up processor specific
 * arrays with all I2S pins and to setup and end the pin definition.
 * @author Phil Schatzmann
 * @copyright GPLv3
 */

struct I2SPin {
  I2SPin(I2SPinFunction f, PinName n, int alt) {
    function = f;
    pin = n;
    altFunction = alt;
  }
  I2SPinFunction function;
  PinName pin;
  int altFunction;

  void begin() {
    end();
    // define the pin function
    pin_function(pin, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, altFunction));
  }

  /// Undo the current pin function
  void end() {
    PinName p = pin;
    if (p != NC) {
      // If the pin that support PWM or DAC output, we need to turn it off
#if (defined(HAL_DAC_MODULE_ENABLED) && !defined(HAL_DAC_MODULE_ONLY)) || \
    (defined(HAL_TIM_MODULE_ENABLED) && !defined(HAL_TIM_MODULE_ONLY))
      if (is_pin_configured(p, g_anOutputPinConfigured)) {
#if defined(HAL_DAC_MODULE_ENABLED) && !defined(HAL_DAC_MODULE_ONLY)
        if (pin_in_pinmap(p, PinMap_DAC)) {
          dac_stop(p);
        } else
#endif  // HAL_DAC_MODULE_ENABLED && !HAL_DAC_MODULE_ONLY
#if defined(HAL_TIM_MODULE_ENABLED) && !defined(HAL_TIM_MODULE_ONLY)
            if (pin_in_pinmap(p, PinMap_TIM)) {
          pwm_stop(p);
        }
#endif  // HAL_TIM_MODULE_ENABLED && !HAL_TIM_MODULE_ONLY
        { reset_pin_configured(p, g_anOutputPinConfigured); }
      }
#endif
    }
  }
};

/**
 * @brief Processor specific settings that are needed to set up I2S
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
struct HardwareConfig {
  IRQn_Type irq1 = DMA1_Stream0_IRQn;
  IRQn_Type irq2 = DMA1_Stream5_IRQn;

#ifdef PLLM
  uint32_t pllm = PLLM;
#endif
  uint32_t plln = PLLN;
  uint32_t pllr = PLLR;

  DMA_Stream_TypeDef *rx_instance = DMA1_Stream0;
  uint32_t rx_channel = DMA_CHANNEL_3;
  uint32_t rx_direction = DMA_PERIPH_TO_MEMORY;

  DMA_Stream_TypeDef *tx_instance = DMA1_Stream5;
  uint32_t tx_channel = DMA_CHANNEL_0;
  uint32_t tx_direction = DMA_MEMORY_TO_PERIPH;

  I2SPin pins[5] = STM_I2S_PINS;

  int buffer_size = 512;

  HardwareConfig() {
    // overwrite processor specific default settings if necessary
  }
};

/**
 * @brief Currently supported parameters
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
struct I2SSettingsSTM32 {
  uint32_t mode = I2S_MODE_MASTER_TX;
  uint32_t standard = I2S_STANDARD_PHILIPS;
  uint32_t fullduplexmode = I2S_FULLDUPLEXMODE_ENABLE;
  uint32_t sample_rate = I2S_AUDIOFREQ_44K;
  uint32_t data_format = I2S_DATAFORMAT_16B;
  HardwareConfig hardware_config;
  // optioinal reference that will be provided by the callbacks
  void *ref = nullptr;
};

/**
 * @brief I2S API for STM32
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class Stm32I2sClass {
  friend void DMA1_Stream0_IRQHandler(void);
  friend void DMA1_Stream5_IRQHandler(void);
  friend void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
  friend void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);

 public:
  /// start I2S w/o DMA: use write and readBytes
  bool begin(I2SSettingsSTM32 settings, bool transmit, bool receive) {
    this->use_dma = false;
    this->settings = settings;
    this->hw = settings.hardware_config;
    int buffer_size = hw.buffer_size;
    bool result = true;

    if (!i2s_begin()) {
      return false;
    }

    if (!transmit && !receive) {
      return false;
    }

    if (use_dma) {
      if (transmit && !receive) {
        if (HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dma_buffer_tx,
                                 buffer_size) != HAL_OK) {
          STM32_LOG("error HAL_I2S_Transmit_DMA");
          Report_Error();
          result = false;
        }
      }

      if (receive && !transmit) {
        if (HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)dma_buffer_rx,
                                buffer_size) != HAL_OK) {
          STM32_LOG("error: HAL_I2S_Receive_DMA");
          Report_Error();
          result = false;
        }
      }

      if (receive && transmit) {
        if (HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)dma_buffer_tx,
                                          (uint16_t *)dma_buffer_rx,
                                          buffer_size) != HAL_OK) {
          STM32_LOG("error HAL_I2SEx_TransmitReceive_DMA");
          Report_Error();
          result = false;
        }
      }
    }

    return result;
  }

  /// Start to transmit I2S data
  bool beginWriteDMA(I2SSettingsSTM32 settings,
                     void (*readToTransmit)(uint8_t *buffer, uint16_t byteCount,
                                            void *ref) = nullptr) {
    this->use_dma = true;
    this->settings = settings;
    this->hw = settings.hardware_config;
    int buffer_size = hw.buffer_size;
    readToTransmitCB = readToTransmit;
    if (dma_buffer_tx == nullptr) dma_buffer_tx = new byte[buffer_size];
    bool result = true;
    if (!i2s_begin()) {
      return false;
    }
    // start circular dma
    if (HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dma_buffer_tx, buffer_size) !=
        HAL_OK) {
      STM32_LOG("error HAL_I2S_Transmit_DMA");
      Report_Error();
      result = false;
    }
    return result;
  }

  /// Start to receive I2S data
  bool beginReadDMA(I2SSettingsSTM32 settings,
                    void (*writeFromReceive)(uint8_t *buffer,
                                             uint16_t byteCount,
                                             void *ref) = nullptr) {
    this->use_dma = true;
    this->settings = settings;
    this->hw = settings.hardware_config;
    int buffer_size = hw.buffer_size;
    bool result = true;
    writeFromReceiveCB = writeFromReceive;
    if (dma_buffer_rx == nullptr) dma_buffer_rx = new byte[buffer_size];
    if (!i2s_begin()) {
      return false;
    }
    // start circular dma
    if (HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)dma_buffer_rx, buffer_size) !=
        HAL_OK) {
      STM32_LOG("error: HAL_I2S_Transmit_DMA");
      Report_Error();
      result = false;
    }
    return result;
  }

  /// Start to receive and transmit I2S data
  bool beginReadWriteDMA(I2SSettingsSTM32 settings,
                         void (*readToTransmit)(uint8_t *buffer,
                                                uint16_t byteCount,
                                                void *) = nullptr,
                         void (*writeFromReceive)(uint8_t *buffer,
                                                  uint16_t byteCount,
                                                  void *) = nullptr) {
    this->use_dma = true;
    this->settings = settings;
    this->hw = settings.hardware_config;
    int buffer_size = hw.buffer_size;
    bool result = true;
    readToTransmitCB = readToTransmit;
    writeFromReceiveCB = writeFromReceive;

    if (dma_buffer_tx == nullptr) dma_buffer_tx = new byte[buffer_size];

    if (dma_buffer_rx == nullptr) dma_buffer_rx = new byte[buffer_size];

    if (!i2s_begin()) {
      return false;
    }
    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (uint16_t *)dma_buffer_tx,
                                      (uint16_t *)dma_buffer_rx,
                                      buffer_size) != HAL_OK) {
      STM32_LOG("error HAL_I2SEx_TransmitReceive_DMA");
      Report_Error();
      result = false;
    }
    return result;
  }

  void end() {
    if (use_dma) HAL_I2S_DMAStop(&hi2s3);
    HAL_I2S_DeInit(&hi2s3);
    HAL_I2S_MspDeInit(&hi2s3);
    if (dma_buffer_tx != NULL) {
      delete[] (dma_buffer_tx);
    }
    if (dma_buffer_rx != NULL) {
      delete[] (dma_buffer_rx);
    }
  }

  /// @brief Write method which needs to be called when ansync mode is disabled
  /// @param data
  /// @param bytes
  /// @return
  size_t write(const uint8_t *data, size_t bytes) {
    HAL_StatusTypeDef rc = HAL_OK;
    if (!this->use_dma) {
      int samples = bytes / getBytes();
      HAL_StatusTypeDef rc =
          HAL_I2S_Transmit(&hi2s3, (uint16_t *)data, samples, HAL_MAX_DELAY);
    }
    return rc == HAL_OK ? bytes : 0;
  }

  /// @brief Read method which needs to be called when ansync mode is disabled
  /// @param data
  /// @param bytes
  /// @return
  size_t readBytes(uint8_t *data, size_t bytes) {
    HAL_StatusTypeDef rc = HAL_OK;
    if (!this->use_dma) {
      int samples = bytes / getBytes();
      HAL_StatusTypeDef rc =
          HAL_I2S_Receive(&hi2s3, (uint16_t *)data, samples, HAL_MAX_DELAY);
    }
    return rc == HAL_OK ? bytes : 0;
  }

 protected:
  I2SSettingsSTM32 settings;
  I2S_HandleTypeDef hi2s3;
  byte *dma_buffer_tx = nullptr;
  byte *dma_buffer_rx = nullptr;
  void (*readToTransmitCB)(uint8_t *buffer, uint16_t byteCount, void *ref);
  void (*writeFromReceiveCB)(uint8_t *buffer, uint16_t byteCount, void *ref);
  DMA_HandleTypeDef hdma_i2s3_ext_rx;
  DMA_HandleTypeDef hdma_i2s3_ext_tx;
  HardwareConfig hw;
  bool use_dma = false;

  int getBytes() {
    if (settings.data_format == I2S_DATAFORMAT_16B) return 2;
    if (settings.data_format == I2S_DATAFORMAT_24B) return 4;
    if (settings.data_format == I2S_DATAFORMAT_32B) return 4;
    STM32_LOG("unsuppoted data_format");
    return 2;
  }

  /// @brief Callback for double buffer
  /// @param hi2s
  void cb_TxRxComplete(I2S_HandleTypeDef *hi2s) {
    // second half finished, filling it up again while first  half is playing
    uint8_t *dma_buffer_tx = (uint8_t *)hi2s->pTxBuffPtr;
    uint8_t *dma_buffer_rx = (uint8_t *)hi2s->pRxBuffPtr;
    uint16_t buffer_size_tx = hi2s->TxXferSize * 2;  // XferSize is in words
    uint16_t buffer_size_rx = hi2s->RxXferSize * 2;
    if (readToTransmitCB != NULL)
      readToTransmitCB(&(dma_buffer_tx[buffer_size_tx / 2]), buffer_size_tx / 2,
                       settings.ref);
    if (writeFromReceiveCB != NULL)
      writeFromReceiveCB(&(dma_buffer_rx[buffer_size_rx / 2]),
                         buffer_size_rx / 2, settings.ref);
  }

  /// @brief Callback for double buffer
  /// @param hi2s
  void cb_TxRxHalfComplete(I2S_HandleTypeDef *hi2s) {
    // second half finished, filling it up again while first  half is playing
    uint8_t *dma_buffer_tx = (uint8_t *)hi2s->pTxBuffPtr;
    uint8_t *dma_buffer_rx = (uint8_t *)hi2s->pRxBuffPtr;
    uint16_t buffer_size_tx = hi2s->TxXferSize * 2;  // XferSize is in words
    uint16_t buffer_size_rx = hi2s->RxXferSize * 2;
    if (readToTransmitCB != NULL)
      readToTransmitCB(&(dma_buffer_tx[0]), buffer_size_tx / 2, settings.ref);
    if (writeFromReceiveCB != NULL)
      writeFromReceiveCB(&(dma_buffer_rx[0]), buffer_size_rx / 2, settings.ref);
  }

  /// @brief Callback for DMA interrupt request
  inline void cb_dmaIrqRx() { HAL_DMA_IRQHandler(&hdma_i2s3_ext_rx); }

  /// @brief Callback for DMA interrupt request
  inline void cb_dmaIrqTx() { HAL_DMA_IRQHandler(&hdma_i2s3_ext_tx); }

  /// @brief Callback I2S intitialization
  inline void cb_HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) {
    cb_i2s_MspInit(hi2s);
  }

  /// @brief Callback I2S de-intitialization
  inline void cb_HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s) {
    cb_i2s_MspDeInit(hi2s);
  }

  /// Starts the i2s processing
  bool i2s_begin() {
    stm32_i2s_is_error = false;
    if (settings.sample_rate == 0) {
      STM32_LOG("sample_rate must not be 0");
      return false;
    }
    STM32_LOG("use_dma: %s", use_dma ? "true" : "false");
    stm32_i2s_is_error = false;
    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick.
     */
    // HAL_Init();// Not needed -> called by Arduino
    /* Configure the system clock */
    // SystemClock_Config(); // Not needed -> called by Arduino
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    if (use_dma) MX_DMA_Init();
    MX_I2S3_Init();
    return !stm32_i2s_is_error;
  }

  /**
   * @brief GPIO Initialization Function for I2S pins
   *
   * @param None
   * @retval None
   */
  virtual void MX_GPIO_Init(void) {
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Define pins
    for (I2SPin &pin : hw.pins) {
      pin.begin();
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
    hi2s3.Instance = SPI_INSTANCE_FOR_I2S;
    hi2s3.Init.Mode = settings.mode;
    hi2s3.Init.Standard = settings.standard;
    hi2s3.Init.FullDuplexMode = settings.fullduplexmode;
    hi2s3.Init.AudioFreq = settings.sample_rate;
    hi2s3.Init.DataFormat = settings.data_format;
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
  virtual void cb_i2s_MspInit(I2S_HandleTypeDef *hi2s) {
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    /**
     * Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
#ifdef PLLM
    PeriphClkInitStruct.PLLI2S.PLLI2SM = hw.pllm;  // 16;
#endif
#ifdef PLLN
    PeriphClkInitStruct.PLLI2S.PLLI2SN = hw.plln;  // 192;
#endif
#ifdef PLLR
    PeriphClkInitStruct.PLLI2S.PLLI2SR = hw.pllr;  // 2;
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
      Report_Error();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    if (use_dma) {

      /* I2S3 DMA Init */
      if (dma_buffer_rx != nullptr) {
        setupDMA(hdma_i2s3_ext_rx, hw.rx_instance, hw.rx_channel,
                 hw.rx_direction);
        __HAL_LINKDMA(hi2s, hdmarx, hdma_i2s3_ext_rx);
      }

      if (dma_buffer_tx != nullptr) {
        setupDMA(hdma_i2s3_ext_tx, hw.tx_instance, hw.tx_channel,
                 hw.tx_direction);
        __HAL_LINKDMA(hi2s, hdmatx, hdma_i2s3_ext_tx);
      }
    }
  }

  /**
   * @brief DMA Initialization
   * This function configures and initializes the DMA
   */
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
  virtual void cb_i2s_MspDeInit(I2S_HandleTypeDef *hi2s) {
    if (hi2s->Instance == SPI3) {
      /* Peripheral clock disable */
      __HAL_RCC_SPI3_CLK_DISABLE();

      /* I2S3 DMA DeInit */
      HAL_DMA_DeInit(hi2s->hdmarx);
      HAL_DMA_DeInit(hi2s->hdmatx);
    }
  }
};

/// @brief Global I2S Object
extern Stm32I2sClass I2S;

}  // namespace stm32_i2s
