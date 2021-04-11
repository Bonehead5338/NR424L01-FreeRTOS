#include "nrf24l01_freertos.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

static const BaseType_t NRF_EVENT_IRQ_RX_DR = 0x01;
static const BaseType_t NRF_EVENT_IRQ_TX_DS = 0x02;
static const BaseType_t NRF_EVENT_IRQ_MAX_RT = 0x04;

NRF_RESULT nrf_send_packet_freertos(nrf24l01* nrf, uint8_t* data, uint8_t len)
{
    return nrf_send_packet_freertos_w_retry(nrf, data, len, 0);
}

NRF_RESULT nrf_send_packet_freertos_w_retry(nrf24l01* nrf, uint8_t* data, uint8_t len, uint8_t retries)
{
	if (len > NRF_MAX_PAYLOAD_WIDTH) return NRF_ERR_MSG_LENGTH;
    
    uint8_t* msg_data = pvPortMalloc(len);
    memcpy(msg_data, data, len);
    nrf24l01_tx_packet pck = { 
        .data = msg_data,
        .length = len,
        .retries = retries,
    };
    
    if (xQueueSend(nrf->freertos.tx_queue, &pck, portMAX_DELAY) != pdTRUE)
    {
        return NRF_ERR;
    }
    
    return NRF_OK;
}

void nrf_freertos_isr(nrf24l01* nrf)
{
    static BaseType_t xHigherPriorityTaskWoken;
    vTaskNotifyGiveFromISR(nrf->freertos.isr_handler_task, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
            xEventGroupSetBits(nrf->freertos.events, NRF_EVENT_IRQ_TX_DS);
        }
    
        // MaxRetransmits reached
        if(nrf->status.max_rt) {
            xEventGroupSetBits(nrf->freertos.events, NRF_EVENT_IRQ_MAX_RT);
        }
    }
}

void NrfTxTask(void* vnrf)
{
    nrf24l01* nrf = (nrf24l01*) vnrf;
    nrf24l01_tx_packet packet;
    EventBits_t event_bits;
    BaseType_t tx_irq_mask = NRF_EVENT_IRQ_TX_DS | NRF_EVENT_IRQ_MAX_RT;
    
    for (;;)
    {        
        xQueueReceive(nrf->freertos.tx_queue, &packet, portMAX_DELAY);
        nrf_send_packet_async(nrf, packet.data, packet.length);
        event_bits = xEventGroupWaitBits(nrf->freertos.events, tx_irq_mask, pdTRUE, pdFALSE, portMAX_DELAY);
        if (!(event_bits & tx_irq_mask))
        {
            // TODO: handle timeout
        }
        if (event_bits & NRF_EVENT_IRQ_MAX_RT) {
            // TODO: error handling
            nrf_flush_tx_fifo(nrf);
            nrf_set_power_up(nrf, 0);
            nrf_set_power_up(nrf, 1);
            nrf_clear_irq_max_rt(nrf);
            if (packet.retries > 0) {
                packet.retries -= 1;
                xQueueSendToFront(nrf->freertos.tx_queue, &packet, portMAX_DELAY);
                continue;
            }
        }
        if (event_bits & NRF_EVENT_IRQ_TX_DS) {
            nrf_clear_irq_tx_ds(nrf);
        }
        
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
        if (!(event_bits & NRF_EVENT_IRQ_RX_DR))
        {
            // TODO: handle timeout
        }
        
        nrf_reset_ce(nrf);
        nrf_rx_read_all(nrf);
        nrf_set_ce(nrf);
        nrf_clear_irq_rx_dr(nrf);
    }
}
