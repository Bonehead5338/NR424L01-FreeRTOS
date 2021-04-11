#include "nrf24l01_cubemx.h"
//#include "nrf24l01.h"

static nrf24l01_stm32_hal_config* nrf_get_stm32_hal_conf(nrf24l01* nrf)
{
    return (nrf24l01_stm32_hal_config*) nrf->platform_conf;
}

void nrf_stm32_hal_ce_set(nrf24l01* nrf)
{
    nrf24l01_stm32_hal_config* hal = nrf_get_stm32_hal_conf(nrf);
    HAL_GPIO_WritePin(hal->ce_port, hal->ce_pin, GPIO_PIN_SET);
}

void nrf_stm32_hal_ce_reset(nrf24l01* nrf)
{
    nrf24l01_stm32_hal_config* hal = nrf_get_stm32_hal_conf(nrf);
    HAL_GPIO_WritePin(hal->ce_port, hal->ce_pin, GPIO_PIN_RESET);
}

void nrf_stm32_hal_csn_set(nrf24l01* nrf)
{
    nrf24l01_stm32_hal_config* hal = nrf_get_stm32_hal_conf(nrf);
    HAL_GPIO_WritePin(hal->csn_port, hal->csn_pin, GPIO_PIN_SET);
}

void nrf_stm32_hal_csn_reset(nrf24l01* nrf)
{
    nrf24l01_stm32_hal_config* hal = nrf_get_stm32_hal_conf(nrf);
    HAL_GPIO_WritePin(hal->csn_port, hal->csn_pin, GPIO_PIN_RESET);
}

NRF_RESULT nrf_stm32_hal_spi_txrx(nrf24l01* nrf, uint8_t* tx, uint8_t* rx, uint8_t len)
{   
    static HAL_StatusTypeDef spi_err;
    nrf24l01_stm32_hal_config* hal = nrf_get_stm32_hal_conf(nrf);
    
#ifdef NRF_FREERTOS
    xSemaphoreTake(nrf->freertos.spi_mutex, portMAX_DELAY);
#endif // NRF_FREERTOS
   
#ifdef NRF_STM32_HAL_SPI_USE_IRQ
    spi_err = HAL_SPI_TransmitReceive_IT(hal->spi, tx, rx, len);
#else
    spi_err = HAL_SPI_TransmitReceive(hal->spi, tx, rx, len, hal->spi_timeout);
#endif // NRF_STM32_HAL_SPI_USE_IRQ
    
#ifdef NRF_FREERTOS
    xSemaphoreGive(nrf->freertos.spi_mutex);
#endif // NRF_FREERTOS
    
    return spi_err == HAL_OK ? NRF_OK : NRF_ERR;
}

void nrf_stm32_hal_init(nrf24l01* nrf)
{  
    nrf24l01_stm32_hal_config* hal = nrf_get_stm32_hal_conf(nrf);
    
    // SPI CS pin inactive state
    nrf_stm32_hal_csn_set(nrf);
        
    //Send dummy byte to initialize SPI peripheral pin states
    uint8_t dummy;
    HAL_SPI_Transmit(hal->spi, &dummy, 1, hal->spi_timeout);
}

nrf24l01_io_calls nrf_stm32_hal_io = { 
    .ce_set = nrf_stm32_hal_ce_set,
    .ce_reset = nrf_stm32_hal_ce_reset,
    .csn_set = nrf_stm32_hal_csn_set,
    .csn_reset = nrf_stm32_hal_csn_reset,
    .spi_txrx = nrf_stm32_hal_spi_txrx,
};
