
#include "stm32-i2s.h"

namespace stm32_i2s {

Stm32I2sClass I2S;
bool stm32_i2s_is_error = false;

extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  I2S.cb_TxRxComplete(hi2s);
}

extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  I2S.cb_TxRxHalfComplete(hi2s);
}

extern "C" void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  I2S.cb_TxRxComplete(hi2s);
}

extern "C" void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  I2S.cb_TxRxHalfComplete(hi2s);
}

extern "C" void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) { Report_Error(); }

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
extern "C" void DMA1_Stream0_IRQHandler(void) { I2S.cb_dmaIrqRx(); }

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
extern "C" void DMA1_Stream5_IRQHandler(void) { I2S.cb_dmaIrqTx(); }

/**
 * @brief I2S MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2s: I2S handle pointer
 * @retval None
 */
extern "C" void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s) { I2S.cb_HAL_I2S_MspInit(hi2s); }

/**
 * @brief I2S MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2s: I2S handle pointer
 * @retval None
 */
extern "C" void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s) { I2S.cb_HAL_I2S_MspDeInit(hi2s); }
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Report_Error() {
  stm32_i2s_is_error = true;
  STM32_LOG("%s", "stm32-i2s: Error");
}

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
  Serial.flush();
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
extern "C" void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  STM32_LOG("stm32-i2s: Wrong parameters value: file %s on line %d", file,
            line);
}
#endif /* USE_FULL_ASSERT */

}
