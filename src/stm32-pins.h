#pragma once

#ifdef ARDUINO_BLACKPILL_F411CE
#define BLACK_PILL                                                             \
  {                                                                            \
    {mclk, PB_10, GPIO_AF6_SPI3}, {bck, PB_3, GPIO_AF6_SPI3},                  \
        {ws, PA_4, GPIO_AF6_SPI3}, {out, PB_4, GPIO_AF7_I2S3ext}, {            \
      in, PB_5, GPIO_AF6_SPI3                                                  \
    }                                                                          \
  }
  #define STM_I2S_PINS BLACK_PILL
  #define HAS_PLLI2SM
#endif

#ifdef TM32F411xE
  #define STM32F411DISCO                                                         \
    {{mclk, PC_7, GPIO_AF6_SPI3},                                                \
    {bck, PC_10, GPIO_AF6_SPI3},                                                \
    {ws, PA_4, GPIO_AF6_SPI3},                                                  \
    {out, PC_3, GPIO_AF6_SPI3},                                                 \
    {in, PC_12, GPIO_AF6_SPI3}};
  #define STM_I2S_PINS STM32F411DISCO
  #define HAS_PLLI2SM
#endif

#ifndef STM_I2S_PINS
  #define BLACK_PILL                                                             \
    {                                                                            \
      {mclk, PB_10, GPIO_AF6_SPI3}, {bck, PB_3, GPIO_AF6_SPI3},                  \
          {ws, PA_4, GPIO_AF6_SPI3}, {out, PB_4, GPIO_AF7_I2S3ext}, {            \
        in, PB_5, GPIO_AF6_SPI3                                                  \
      }                                                                          \
    }
  #define STM_I2S_PINS BLACK_PILL

#endif
