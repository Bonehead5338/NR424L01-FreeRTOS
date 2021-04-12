#pragma once

#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nrf24l01_config.h"
#include "nrf24l01.h"

nrf24l01_io_calls nrf_esp_idf_io;

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t csn_pin;
    gpio_num_t ce_pin;
    gpio_num_t irq_pin;

} nrf24l01_esp_idf_config;

void nrf_esp_idf_init(nrf24l01* nrf);
