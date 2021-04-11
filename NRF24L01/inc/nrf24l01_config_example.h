#pragma once

#define NRF_STM32_HAL
#define NRF_FREERTOS

#ifdef NRF_STM32_HAL
// define to use _IT calls, undefine to use blocking calls
#define NRF_STM32_HAL_SPI_USE_IRQ
#endif // STM32_HAL


#ifdef NRF_FREERTOS
#define NRF_RTOS_TX_QUEUE_LENGTH 5
#define NRF_RTOS_IRQ_TASK_PRIORITY 40
#endif // NRF_FREERTOS
