#pragma once

#define I2S_BUFFER_SIZE 512
#define STM32_I2S_WITH_OBJECT
#define USE_FULL_ASSERT

#ifndef I2S_FULLDUPLEXMODE_DISABLE
#  define I2S_FULLDUPLEXMODE_DISABLE   (0x00000000U)
#endif
#ifndef I2S_FULLDUPLEXMODE_ENABLE
#  define 	I2S_FULLDUPLEXMODE_ENABLE   (0x00000001U)
#endif

#ifdef ARDUINO_BLACKPILL_F411CE
  #define SPI_INSTANCE_FOR_I2S SPI3
  #define STM_I2S_PINS \
  {\
    {mclk, PB_10, GPIO_AF6_SPI3},\
    {bck, PB_3, GPIO_AF6_SPI3},\
    {ws, PA_4, GPIO_AF6_SPI3}, \
    {data_out, PB_4, GPIO_AF7_I2S3ext},\
    {data_in, PB_5, GPIO_AF6_SPI3},\
  }
// 8 MHz / M * N / R  => I2S Freq
  #define PLLM  16
  #define PLLN 192
  #define PLLR   2
  #define IS_F4

#endif

#ifdef ARDUINO_GENERIC_F411VETX
  #define SPI_INSTANCE_FOR_I2S SPI3
  #define STM_I2S_PINS \
    { \
      {mclk, PC_7, GPIO_AF6_SPI3},\
      {bck, PC_10, GPIO_AF6_SPI3},\
      {ws, PA_4, GPIO_AF6_SPI3},\
      {data_out, PC_12, GPIO_AF6_SPI3},\
      {data_in, PC_3, GPIO_AF6_SPI3}\
    };
// 8 MHz / M * N / R  => I2S Freq
  #define PLLM   16
  #define PLLN   100
  #define PLLR    2
  #define IS_F4

#endif

#ifdef STM32H750xx
  #define SPI_INSTANCE_FOR_I2S SPI3
  #define STM_I2S_PINS \
    { \
      {mclk, PC_7, GPIO_AF6_SPI3},\
      {bck, PC_10, GPIO_AF6_SPI3},\
      {ws, PA_4, GPIO_AF6_SPI3},\
      {data_out, PB_2, GPIO_AF6_SPI3},\
      {data_in, PC_11, GPIO_AF6_SPI3}\
    };
  #define IS_H7
#endif

#ifdef STM32H743xx
  #define SPI_INSTANCE_FOR_I2S SPI3
  #define STM_I2S_PINS \
    { \
      {mclk, PC_7, GPIO_AF6_SPI3},\
      {bck, PC_10, GPIO_AF6_SPI3},\
      {ws, PA_4, GPIO_AF6_SPI3},\
      {data_out, PB_5, GPIO_AF6_SPI3},\
      {data_in, PB_4, GPIO_AF6_SPI3}\
    };
  #define IS_H7
#endif

#ifdef STM32F723xx
  #define SPI_INSTANCE_FOR_I2S SPI3
  #define STM_I2S_PINS \
    { \
      {mclk, PC_7, GPIO_AF6_SPI3},\
      {bck, PC_10, GPIO_AF6_SPI3},\
      {ws, PA_4, GPIO_AF6_SPI3},\
      {data_out, PC_12, GPIO_AF6_SPI3},\
      {data_in, PC_11, GPIO_AF6_SPI3}\
    };
  #define IS_F7
  #define SPI_CLOCK_SOURCE LL_RCC_SPI123_CLKSOURCE_PLL1Q
#endif
