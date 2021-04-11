#pragma once

#include "nrf24l01_config.h"
#include "nrf24l01_ll.h"

void IsrHandlerTask(void * vnrf);
void NrfTxTask(void* vnrf);
void NrfRxTask(void* vnrf);

NRF_RESULT nrf_send_packet_freertos(nrf24l01* nrf, uint8_t* data, uint8_t len);
NRF_RESULT nrf_send_packet_freertos_w_retry(nrf24l01* nrf, uint8_t* data, uint8_t len, uint8_t retries);
void nrf_freertos_isr(nrf24l01* nrf);
