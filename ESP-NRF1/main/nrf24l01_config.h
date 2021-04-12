#pragma once

// #define NRF_STM32_HAL
#define NRF_ESP_IDF
#define NRF_FREERTOS

#ifdef NRF_STM32_HAL
// define to use _IT calls, undefine to use blocking calls
#define NRF_STM32_HAL_SPI_USE_IRQ
#endif // STM32_HAL

#ifdef NRF_ESP_IDF
// define to use _IT calls, undefine to use blocking calls
#define NRF_ESP_IDF_SPI_USE_IRQ
#endif // NRF_ESP_IDF

#ifdef NRF_FREERTOS
// task stack size. may need to increase if e.g. printf is used in rx callback
// default configMINIMAL_STACK_SIZE
#define NRF_RTOS_IRQ_TASK_STACK_SIZE 2048 // configMINIMAL_STACK_SIZE
#define NRF_RTOS_TX_TASK_STACK_SIZE 2048 // configMINIMAL_STACK_SIZE
#define NRF_RTOS_RX_TASK_STACK_SIZE 4096 //configMINIMAL_STACK_SIZE

// priority of tasks
// NOTE: idle task priority is different in e.g. cmsis-rtos v1/v2
#define NRF_RTOS_IRQ_TASK_PRIORITY 5
#define NRF_RTOS_TX_TASK_PRIORITY (NRF_RTOS_IRQ_TASK_PRIORITY - 1)
#define NRF_RTOS_RX_TASK_PRIORITY (NRF_RTOS_IRQ_TASK_PRIORITY - 1)
// size of tx queue
#define NRF_RTOS_TX_QUEUE_LENGTH 5
#define NRF_RTOS_TX_QUEUE_SEND_TIMEOUT portMAX_DELAY
#define NRF_RTOS_TX_QUEUE_RECEIVE_TIMEOUT portMAX_DELAY
#define NRF_RTOS_TX_STATUS_TIMEOUT_MS 100
#define NRF_RTOS_RX_EVENT_RECEIVE_TIMEOUT portMAX_DELAY
#endif // NRF_FREERTOS
