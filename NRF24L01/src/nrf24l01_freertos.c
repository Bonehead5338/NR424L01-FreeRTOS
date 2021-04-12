#include "nrf24l01_freertos.h"
#include <string.h>

#ifdef NRF_ESP_IDF
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#else
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "task.h"
#endif // NRF_ESP_IDF

static const BaseType_t NRF_EVENT_IRQ_RX_DR = 0x01;
static const BaseType_t NRF_EVENT_IRQ_TX_DS = 0x02;
static const BaseType_t NRF_EVENT_IRQ_MAX_RT = 0x04;

NRF_RESULT nrf_send_packet_freertos(nrf24l01* nrf, uint8_t* data, uint8_t len)
{
	if (len > NRF_MAX_PAYLOAD_WIDTH) return NRF_ERR_MSG_LENGTH;
    
    uint8_t* msg_data = pvPortMalloc(len);
    memcpy(msg_data, data, len);
    nrf24l01_freertos_tx_packet pck = { 
        .data = msg_data,
        .length = len,
    };
    
    if (xQueueSend(nrf->freertos.tx_queue, &pck, NRF_RTOS_TX_QUEUE_SEND_TIMEOUT) != pdTRUE)
    {
        return NRF_ERR;
    }
    
    return NRF_OK;
}

#ifdef NRF_ESP_IDF
void IRAM_ATTR nrf_freertos_isr(nrf24l01* nrf)
#else
void nrf_freertos_isr(nrf24l01* nrf)
#endif
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(nrf->freertos.isr_handler_task, &xHigherPriorityTaskWoken);
#ifdef NRF_ESP_IDF
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
#else
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}

void IsrHandlerTask(void * vnrf)
{
    nrf24l01* nrf = (nrf24l01*) vnrf;
        
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
        nrf_nop(nrf);  // to get status

        // RX FIFO Interrupt   
        if(nrf->status.rx_dr) {
            xEventGroupSetBits(nrf->freertos.events, NRF_EVENT_IRQ_RX_DR);
        }

        // TX Data Sent Interrupt
        if(nrf->status.tx_ds) {
            if (nrf->tx_busy) {
                xEventGroupSetBits(nrf->freertos.events, NRF_EVENT_IRQ_TX_DS);
            } else {
                nrf_clear_irq_tx_ds(nrf);
            }
        }
    
        // MaxRetransmits reached
        if(nrf->status.max_rt) {
            if (nrf->tx_busy) {
                xEventGroupSetBits(nrf->freertos.events, NRF_EVENT_IRQ_MAX_RT);
            } else {
                nrf_clear_irq_max_rt(nrf);
            }
        }
    }
}

void NrfTxTask(void* vnrf)
{
    nrf24l01* nrf = (nrf24l01*) vnrf;
    nrf24l01_freertos_tx_packet packet;
    EventBits_t event_bits;
    BaseType_t tx_irq_mask = NRF_EVENT_IRQ_TX_DS | NRF_EVENT_IRQ_MAX_RT;
    
    for (;;)
    {        
        xQueueReceive(nrf->freertos.tx_queue, &packet, NRF_RTOS_TX_QUEUE_RECEIVE_TIMEOUT);
        nrf_send_packet_async(nrf, packet.data, packet.length);
        nrf->tx_busy = true;
        event_bits = xEventGroupWaitBits(nrf->freertos.events, tx_irq_mask, pdTRUE, pdFALSE, pdMS_TO_TICKS(NRF_RTOS_TX_STATUS_TIMEOUT_MS));
        
        if (!(event_bits & tx_irq_mask)) {
            nrf->tx_result = NRF_TX_TIMEOUT;
        }
        if (event_bits & NRF_EVENT_IRQ_MAX_RT) // tx status timeout || max retries
        {
            nrf->tx_result = NRF_TX_MAX_RT;
            nrf_flush_tx_fifo(nrf);
            nrf_clear_irq_max_rt(nrf);
        }
        if (event_bits & NRF_EVENT_IRQ_TX_DS) {
            nrf->tx_result = NRF_TX_DS;
            nrf_flush_tx_fifo(nrf);
            nrf_clear_irq_tx_ds(nrf);
        }

        if (nrf->tx_complete_cb) {
            nrf->tx_complete_cb(nrf, nrf->tx_result, packet.length);
        }

        nrf->tx_busy = false;   
        nrf_rx_mode(nrf);
        vPortFree(packet.data);
    }
}

void NrfRxTask(void* vnrf)
{
    nrf24l01* nrf = (nrf24l01*) vnrf;
    EventBits_t event_bits;
        
    for (;;)
    {        
        event_bits = xEventGroupWaitBits(nrf->freertos.events, NRF_EVENT_IRQ_RX_DR, pdTRUE, pdFALSE, portMAX_DELAY);
        if (event_bits & NRF_EVENT_IRQ_RX_DR) {
            nrf_reset_ce(nrf);
            nrf_rx_read_all(nrf);
            nrf_set_ce(nrf);
            nrf_clear_irq_rx_dr(nrf);
        }
    }
}
