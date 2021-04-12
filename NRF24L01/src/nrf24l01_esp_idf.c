#include "nrf24l01_esp_idf.h"
#include "string.h"
//#include "nrf24l01.h"

static nrf24l01_esp_idf_config* nrf_get_esp_idf_conf(nrf24l01* nrf)
{
    return (nrf24l01_esp_idf_config*) nrf->platform_conf;
}

void nrf_esp_idf_ce_set(nrf24l01* nrf)
{
    nrf24l01_esp_idf_config* esp = nrf_get_esp_idf_conf(nrf);
    gpio_set_level(esp->ce_pin, 1);
}

void nrf_esp_idf_ce_reset(nrf24l01* nrf)
{
    nrf24l01_esp_idf_config* esp = nrf_get_esp_idf_conf(nrf);
    gpio_set_level(esp->ce_pin, 0);
}

void nrf_esp_idf_csn_set(nrf24l01* nrf)
{
    nrf24l01_esp_idf_config* esp = nrf_get_esp_idf_conf(nrf);
    gpio_set_level(esp->csn_pin, 1);
}

void nrf_esp_idf_csn_reset(nrf24l01* nrf)
{
    nrf24l01_esp_idf_config* esp = nrf_get_esp_idf_conf(nrf);
    gpio_set_level(esp->csn_pin, 0);
}

NRF_RESULT nrf_esp_idf_spi_txrx(nrf24l01* nrf, uint8_t* tx, uint8_t* rx, uint8_t len)
{   
    esp_err_t spi_err;
    static spi_transaction_t t;

    nrf24l01_esp_idf_config* esp = nrf_get_esp_idf_conf(nrf);

    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

#ifdef NRF_FREERTOS
    xSemaphoreTake(nrf->freertos.spi_mutex, portMAX_DELAY);
#endif // NRF_FREERTOS

#ifdef NRF_ESP_IDF_SPI_USE_IRQ
    spi_err = spi_device_transmit(esp->spi, &t);
#else
    spi_err = spi_device_polling_transmit(esp->spi, &t);
#endif
    
#ifdef NRF_FREERTOS
    xSemaphoreGive(nrf->freertos.spi_mutex);
#endif // NRF_FREERTOS

   return spi_err == ESP_OK ? NRF_OK : NRF_ERR;
    
}

void nrf_esp_idf_init(nrf24l01* nrf)
{  
    // nrf24l01_esp_idf_config* esp = nrf_get_esp_idf_conf(nrf);
    
    // // SPI CS pin inactive state
    // nrf_esp_idf_csn_set(nrf);
        
    // //Send dummy byte to initialize SPI peripheral pin states
    // uint8_t dummy;
    // HAL_SPI_Transmit(esp->spi, &dummy, 1, esp->spi_timeout);
}

nrf24l01_io_calls nrf_esp_idf_io = { 
    .ce_set = nrf_esp_idf_ce_set,
    .ce_reset = nrf_esp_idf_ce_reset,
    .csn_set = nrf_esp_idf_csn_set,
    .csn_reset = nrf_esp_idf_csn_reset,
    .spi_txrx = nrf_esp_idf_spi_txrx,
};
