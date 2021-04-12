/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"

#include "nrf24l01_config.h"
#include "nrf24l01.h"
#include "nrf24l01_esp_idf.h"
#include "nrf24l01_ll.h"

#define PIN_NUM_NRF_MISO 19
#define PIN_NUM_NRF_MOSI 23
#define PIN_NUM_NRF_CLK  18
#define PIN_NUM_NRF_CSN  5
#define PIN_NUM_NRF_CE   16
#define PIN_NUM_NRF_IRQ  17

#define NRF_HOST VSPI_HOST
#define NRF_DMA_CHAN 1

nrf24l01 nrf1;
spi_device_handle_t nrf_spi_dev;

void nrf_rx_msg_cb(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t len);
void nrf_tx_complete_cb(nrf24l01* nrf, NRF_TX_RESULT result, uint8_t len);
void IRAM_ATTR nrf_isr(void* args);

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

void app_main(void)
{   
    /* SPI */
    
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_NRF_MISO,
        .mosi_io_num = PIN_NUM_NRF_MOSI,
        .sclk_io_num = PIN_NUM_NRF_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz =33
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10*1000*1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 5,
    };

    spi_bus_initialize(NRF_HOST, &buscfg, NRF_DMA_CHAN);
    spi_bus_add_device(NRF_HOST, &devcfg, &nrf_spi_dev);

    /* IRQ Pin and Interrupt */

    gpio_set_direction(PIN_NUM_NRF_IRQ, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_NUM_NRF_IRQ, GPIO_INTR_NEGEDGE);
    gpio_pullup_dis(PIN_NUM_NRF_IRQ);
    gpio_pulldown_dis(PIN_NUM_NRF_IRQ);

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_isr_handler_add(PIN_NUM_NRF_IRQ, nrf_isr, &nrf1 );

    /* CE Pin */

    gpio_pad_select_gpio(PIN_NUM_NRF_CE);
    gpio_set_direction(PIN_NUM_NRF_CE, GPIO_MODE_OUTPUT);
    
    gpio_pad_select_gpio(PIN_NUM_NRF_CSN);
    gpio_set_direction(PIN_NUM_NRF_CSN, GPIO_MODE_OUTPUT);

    /* NRF config */

    nrf24l01_esp_idf_config nrf1_platform_conf = {
        .spi = nrf_spi_dev,
        .csn_pin = PIN_NUM_NRF_CSN,
        .ce_pin = PIN_NUM_NRF_CE,
        .irq_pin = PIN_NUM_NRF_IRQ,
    };

    nrf24l01_init init1 = {
        .platform_conf = &nrf1_platform_conf,
        .platform_init = NULL,
        .io = nrf_esp_idf_io,
        .rx_msg_cb = nrf_rx_msg_cb,
        .tx_complete_cb = nrf_tx_complete_cb,
    };

    nrf24l01_esb_init esb1 = {
        .data_rate = NRF_DATA_RATE_1MBPS,
        .tx_power = NRF_TX_PWR_0dBm,
        .rf_channel = 65,
        .tx_address = 0x0A0B0C0D02,
        .rx_address = 0x0A0B0C0D01,
    };

    nrf_init_enhanced_shockbust_default(&nrf1, &init1, &esb1);

    int tx_data = 1;

    while(1) {
        printf("Sending message\n");
        nrf_send_packet(&nrf1, (uint8_t*)&tx_data, sizeof(tx_data));
        tx_data++;
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
}

void nrf_rx_msg_cb(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t len)
{
    printf("Received Message on pipe %d, len %d \n", pipe, len);
}

void nrf_tx_complete_cb(nrf24l01* nrf, NRF_TX_RESULT result, uint8_t len)
{
    switch (result)
    {
    case NRF_TX_DS:
        printf("Successfully Sent Message, len %d\n", len);
        return;
    case NRF_TX_MAX_RT:
        printf("Transmit Message Failed (Maximum Retries Exceeded)\n");
        return;
    case NRF_TX_TIMEOUT:
        printf("Transmit Message Failed (No TX Completion Event)\n");
    default:
        return;
    }
}

void IRAM_ATTR nrf_isr(void* args) {
    nrf24l01* nrf = (nrf24l01*) args;
    nrf_freertos_isr(nrf);
}
