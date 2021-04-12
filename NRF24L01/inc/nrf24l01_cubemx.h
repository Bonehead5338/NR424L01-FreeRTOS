#pragma once

#include "main.h"
#include "nrf24l01_config.h"
#include "nrf24l01.h"

nrf24l01_io_calls nrf_stm32_hal_io;

typedef struct {
    SPI_HandleTypeDef* spi;
    uint32_t           spi_timeout;

    GPIO_TypeDef* csn_port;
    uint16_t      csn_pin;

    GPIO_TypeDef* ce_port;
    uint16_t      ce_pin;

    GPIO_TypeDef* irq_port;
    uint16_t      irq_pin;
} nrf24l01_stm32_hal_config;

void nrf_stm32_hal_init(nrf24l01* nrf);
