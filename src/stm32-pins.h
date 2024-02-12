#pragma once

#ifdef ARDUINO_BLACKPILL_F411CE
#define BLACK_PILL \
  {\
    {mclk, PB_10, GPIO_AF6_SPI3},\
    {bck, PB_3, GPIO_AF6_SPI3},\
    {ws, PA_4, GPIO_AF6_SPI3}, \
    {data_out, PB_4, GPIO_AF7_I2S3ext},\
    {data_in, PB_5, GPIO_AF6_SPI3},\
  }
  #define STM_I2S_PINS BLACK_PILL
  #define HAS_PLLI2SM
#endif

#ifdef TM32F411xE
  #define STM32F411DISCO \
    { \
      {mclk, PC_7, GPIO_AF6_SPI3},\
      {bck, PC_10, GPIO_AF6_SPI3},\
      {ws, PA_4, GPIO_AF6_SPI3},\
      {data_out, PC_3, GPIO_AF6_SPI3},\
      {data_in, PC_12, GPIO_AF6_SPI3}\
    };
  #define STM_I2S_PINS STM32F411DISCO
  #define HAS_PLLI2SM
#endif

