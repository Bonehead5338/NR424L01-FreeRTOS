#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nrf24l01.h"
#include "nrf24l01_cubemx.h"

nrf24l01 nrf1;

extern SPI_HandleTypeDef hspi1;

void nrf_rx_msg_cb(nrf24l01* dev, NRF_PIPE_NO pipe, uint8_t len);

void StartDefaultTask(void* params)
{
    nrf24l01_stm32_hal_config nrf1_platform_conf = {
        .spi = &hspi1,
        .spi_timeout = 10,
        .ce_port = NRF_CE_GPIO_Port,
        .ce_pin = NRF_CE_Pin,
        .csn_port = NRF_CSN_GPIO_Port,
        .csn_pin = NRF_CSN_Pin,
        .irq_port = NRF_IRQ_GPIO_Port,
        .irq_pin = NRF_IRQ_Pin
    };

    nrf24l01_init init1 = {
        .platform_conf = &nrf1_platform_conf,
        .platform_init = nrf_stm32_hal_init,
        .io = nrf_stm32_hal_io,
        .rx_msg_cb = nrf_rx_msg_cb,
    };

    nrf24l01_esb_init esb1 = {
        .data_rate = NRF_DATA_RATE_1MBPS,
        .tx_power = NRF_TX_PWR_0dBm,
        .rf_channel = 19,
        .tx_address = 0x0A0B0C0D02,
        .rx_address = 0x0A0B0C0D01,
    };

    nrf_init_enhanced_shockbust_default(&nrf1, &init1, &esb1);

    uint64_t tx_data = 1;

    for (;;)
    {
        nrf_send_packet(&nrf1, (uint8_t*)&tx_data, sizeof(tx_data));
        tx_data++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15)
    {
        nrf_freertos_isr(&nrf1);
    }
}

void nrf_rx_msg_cb(nrf24l01* dev, NRF_PIPE_NO pipe, uint8_t len)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
