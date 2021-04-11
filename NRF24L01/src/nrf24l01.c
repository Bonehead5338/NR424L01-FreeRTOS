#include <string.h>

#include "nrf24l01_ll.h"

#ifdef NRF_FREERTOS
#include "nrf24l01_freertos.h"
#endif // NRF_FREERTOS

NRF_RESULT nrf_tx_irq_handler(nrf24l01* nrf, NRF_TX_RESULT tx_result);
NRF_RESULT nrf_rx_irq_handler(nrf24l01* nrf);
NRF_RESULT nrf_clear_tx_irq(nrf24l01* nrf, NRF_TX_RESULT tx_result);
static NRF_RESULT nrf_tx_pre_flight(nrf24l01* nrf, uint8_t len, bool* ok);

NRF_RESULT nrf_rx_read_payload(nrf24l01* nrf);

static const uint8_t NRF_RF_CHANNEL_COUNT = 125;
static const uint64_t NRF_RX_ADDR_DEFAULT[NRF_RX_PIPE_COUNT] = { 
    0xE7E7E7E7,
    0xC2C2C2C2,
    0xC3,
    0xC4,
    0xC5,
    0xC6,
};

// default/example ISR, weak
void nrf_irq_isr(nrf24l01* nrf)
{
    nrf->irq_active = true;
}

void nrf_init(nrf24l01* nrf, nrf24l01_init* init)
{
    nrf->platform_conf = init->platform_conf;
    nrf->io = init->io;
    nrf->rx_msg_cb = init->rx_msg_cb;
    nrf->rx_msg_len = -1;

    if (init->platform_init)
    {
        init->platform_init(nrf);
    }
    
#ifdef NRF_FREERTOS
    nrf->freertos.spi_mutex = xSemaphoreCreateMutex();
    nrf->freertos.tx_queue = xQueueCreate(NRF_RTOS_TX_QUEUE_LENGTH, sizeof(nrf24l01_tx_packet));
    nrf->freertos.events = xEventGroupCreate();
    xTaskCreate(IsrHandlerTask, "IsrHandler", configMINIMAL_STACK_SIZE, (void *) nrf, NRF_RTOS_IRQ_TASK_PRIORITY, &nrf->freertos.isr_handler_task);
    xTaskCreate(NrfTxTask, "TxTask", configMINIMAL_STACK_SIZE, (void *) nrf, NRF_RTOS_IRQ_TASK_PRIORITY -1, &nrf->freertos.tx_task);
    xTaskCreate(NrfRxTask, "RxTask", configMINIMAL_STACK_SIZE, (void *) nrf, NRF_RTOS_IRQ_TASK_PRIORITY -1, &nrf->freertos.rx_task);
#endif // NRF_FREERTOS
}

NRF_RESULT nrf_init_enhanced_shockbust_default(nrf24l01* nrf, nrf24l01_init* init, nrf24l01_esb_init* esb_config)
{
    // rx_p0 same address as tx for auto-ack
    nrf24l01_rx_pipe rx_p0 = { 
        .enabled = true,
        .auto_ack = true,
        .dynamic_payload = true,
        .address = esb_config->tx_address & NRF_MAX_ADDR_WIDTH_MASK
    };
    
    nrf24l01_rx_pipe rx_p1 = { 
        .enabled = true,
        .auto_ack = true,
        .dynamic_payload = true,
        .address = esb_config->rx_address & NRF_MAX_ADDR_WIDTH_MASK
    };
    
    nrf24l01_irq irq = { 
        .max_retries = true,
        .tx_data_sent = true,
        .rx_data_ready = true,
    };
        
    nrf24l01_config config = { 
        .rf_channel = 65,
        .data_rate = esb_config->data_rate,
        .tx_power = esb_config->tx_power,
        .addr_width = NRF_ADDR_WIDTH_5,
        .retransmit_count = 15,
        .retransmit_delay = 15,
        .crc_en = true,
        .crc_width = NRF_CRC_WIDTH_1B,
        .tx_address = esb_config->tx_address & NRF_MAX_ADDR_WIDTH_MASK,
        .rx_pipes = { rx_p0, rx_p1 },
        .irq = irq,
        .custom_irq_cb = false,
    };
    
    return nrf_init_config(nrf, init, &config);
}

NRF_RESULT nrf_init_config(nrf24l01* nrf, nrf24l01_init* init, nrf24l01_config* config) {
    nrf_init(nrf, init);

    nrf_reset_ce(nrf);
    
    NRF_ERR_CHECK(nrf_set_power_up(nrf, true));

    bool pwr_up = false;
    while (!pwr_up) { // wait for powerup
      NRF_ERR_CHECK(nrf_get_power_up(nrf, &pwr_up));
      // TODO: timeout and error?
    }
    
    // override default irq handlers
    if (config->custom_irq_cb)
    {
        nrf->irq_cb.tx = config->irq_cb.tx;
        nrf->irq_cb.rx = config->irq_cb.rx;
    }
    else
    {
        nrf->irq_cb.tx = nrf_tx_irq_handler;
        nrf->irq_cb.rx = nrf_rx_irq_handler;
    }
        
    // general settings
    NRF_ERR_CHECK(nrf_set_rf_channel(nrf, config->rf_channel));
    NRF_ERR_CHECK(nrf_set_data_rate(nrf, config->data_rate));
    NRF_ERR_CHECK(nrf_set_retransmission_count(nrf, config->retransmit_count));
    NRF_ERR_CHECK(nrf_set_retransmission_delay(nrf, config->retransmit_delay));
    NRF_ERR_CHECK(nrf_set_crc_en(nrf, config->crc_en));
    NRF_ERR_CHECK(nrf_set_crc_width(nrf, config->crc_width));    
    NRF_ERR_CHECK(nrf_set_address_width(nrf, config->addr_width));
    
    // tx pipe config
    NRF_ERR_CHECK(nrf_set_tx_address(nrf, config->tx_address));
    
    // rx pipe configs
    uint8_t pipe_en_mask = 0, pipe_dpl_mask = 0, pipe_aa_mask = 0;
    for (NRF_PIPE_NO p = NRF_PIPE_P0; p < NRF_RX_PIPE_COUNT; p++)
    {
        nrf24l01_rx_pipe pipe = config->rx_pipes[p];
        
        pipe_en_mask |= pipe.enabled << p;
        pipe_dpl_mask |= pipe.dynamic_payload << p;
        pipe_aa_mask |= pipe.auto_ack << p;
        
        // address
        uint64_t address = pipe.address;
        if (!pipe.enabled && pipe.address == 0)
            address = NRF_RX_ADDR_DEFAULT[p];
        NRF_ERR_CHECK(nrf_set_rx_address(nrf, p, address)) ;
               
        // payload width
        NRF_ERR_CHECK(nrf_set_rx_payload_width(nrf, p, pipe.payload_width)) ;
    }
    
    // enable or disable dynamic payloads
    NRF_ERR_CHECK(nrf_enable_dyn_pl_feature(nrf, pipe_dpl_mask != 0));   
    NRF_ERR_CHECK(nrf_enable_dyn_pl(nrf, pipe_dpl_mask));
    
    // enable or disable auto-ack
    NRF_ERR_CHECK(nrf_enable_rx_auto_ack(nrf, pipe_aa_mask));
    
    // enable rx pipes
    NRF_ERR_CHECK(nrf_enable_rx_pipes(nrf, pipe_en_mask));
           
    // clear and enable interrupts
    NRF_ERR_CHECK(nrf_clear_interrupts(nrf));
    NRF_ERR_CHECK(nrf_enable_rx_data_ready_irq(nrf, true));
    NRF_ERR_CHECK(nrf_enable_tx_data_sent_irq(nrf, true));
    NRF_ERR_CHECK(nrf_enable_max_retransmit_irq(nrf, true));
    
    // flush FIFOs
    NRF_ERR_CHECK(nrf_flush_tx_fifo(nrf));
    NRF_ERR_CHECK(nrf_flush_rx_fifo(nrf));

    // start in Rx mode
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_RX));
    nrf_set_ce(nrf);

    return NRF_OK;
}

NRF_RESULT nrf_tx_mode(nrf24l01* nrf)
{
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_TX));
    nrf_set_ce(nrf);
    return NRF_OK;
}

NRF_RESULT nrf_rx_mode(nrf24l01* nrf)
{
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_RX));
    nrf_set_ce(nrf);
    return NRF_OK;
}

NRF_RESULT nrf_rx_read_payload(nrf24l01* nrf)
{
    NRF_RESULT error;
    uint8_t len = 0;
    // get width
    NRF_ERR_CHECK(nrf_ll_cmd_r_rx_pl_wid(nrf, &len));
    if (len > NRF_MAX_PAYLOAD_WIDTH) {
        NRF_ERR_CHECK(nrf_flush_rx_fifo(nrf));
        return NRF_ERR;
    }
    
    // get pipe # from returned status
    NRF_PIPE_NO pipe = (NRF_PIPE_NO) nrf->status.rx_p_no;
    
    // retrieve payload
    error = nrf_ll_cmd_r_rx_payload(nrf, nrf->rx_msg, len);
    if (error != NRF_OK) {
        nrf->rx_msg_len = -1;
        return error;
    }
       
    nrf->rx_msg_len = len;
            
    // message callback
    if (nrf->rx_msg_cb) {
        nrf->rx_msg_cb(nrf, pipe, len);
    }
    
    return NRF_OK;
}

NRF_RESULT nrf_rx_read_all(nrf24l01* nrf)
{
    nrf24l01_reg_fifo_status fifo_status;
    NRF_RESULT error = nrf_ll_get_fifo_status(nrf, &fifo_status);

    while(!(fifo_status.rx_empty || error)) {
       NRF_ERR_CHECK(nrf_rx_read_payload(nrf));
       error = nrf_ll_get_fifo_status(nrf, &fifo_status);
    }
    
    return error;
}

NRF_RESULT nrf_rx_irq_handler(nrf24l01* nrf) {
    NRF_RESULT err;
    nrf_reset_ce(nrf);
    err = nrf_rx_read_all(nrf);
    nrf_set_ce(nrf);
    nrf->rx_busy = 0;
    return err;
}

NRF_RESULT nrf_tx_irq_handler(nrf24l01* nrf, NRF_TX_RESULT tx_result) {
    
    nrf->tx_busy   = 0;
    switch (tx_result)
    {
    case NRF_TX_DS:
        nrf->tx_result = NRF_OK;
        break;
    case NRF_TX_MAX_RT:
        nrf->tx_result = NRF_ERR_TX_MAX_RT;
        NRF_ERR_CHECK(nrf_flush_tx_fifo(nrf));
        NRF_ERR_CHECK(nrf_set_power_up(nrf, 0));      // power down
        NRF_ERR_CHECK(nrf_set_power_up(nrf, 1));     // power up
    default:
        break;
    }
           
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_RX));
    nrf_set_ce(nrf);
    
    return NRF_OK;
}

NRF_RESULT nrf_irq_handler(nrf24l01* nrf)
{
    // NOP to get status
    nrf_nop(nrf);
    
    // RX FIFO Interrupt   
    if (nrf ->status.rx_dr) {
        if (nrf->irq_cb.rx) {
            nrf->irq_cb.rx(nrf);
        }
        else {
            nrf_rx_irq_handler(nrf);   
        }
        nrf_clear_irq_rx_dr(nrf);
    }
    
    // TX Data Sent Interrupt
    if(nrf->status.tx_ds) {
        nrf->tx_busy   = 0;
        if (nrf->irq_cb.tx) {
            nrf->irq_cb.tx(nrf, NRF_TX_DS);
        }
        else {
            nrf_tx_irq_handler(nrf, NRF_TX_DS);
        }
        nrf_clear_tx_irq(nrf, NRF_TX_DS);
    }
    
    // MaxRetransmits reached
    if(nrf->status.max_rt) {
        nrf->tx_busy   = 0;
        if (nrf->irq_cb.tx) {
            nrf->irq_cb.tx(nrf, NRF_TX_MAX_RT);
        }
        else{
            nrf_tx_irq_handler(nrf, NRF_TX_MAX_RT);
        }
        nrf_clear_tx_irq(nrf, NRF_TX_DS);
    }
    
    nrf->irq_active = false;
    
    return NRF_OK;
}

NRF_RESULT nrf_write_tx_payload(nrf24l01* nrf, const uint8_t* data, uint8_t len) {
    if (len > NRF_MAX_PAYLOAD_WIDTH) return NRF_ERR_MSG_LENGTH;
    return nrf_ll_cmd_w_tx_payload(nrf, data, len);
}

NRF_RESULT nrf_write_tx_payload_noack(nrf24l01* nrf, const uint8_t* data, uint8_t len) {
    if (len > NRF_MAX_PAYLOAD_WIDTH) return NRF_ERR_MSG_LENGTH;
    return nrf_ll_cmd_w_tx_payload_noack(nrf, data, len);
}

NRF_RESULT nrf_flush_tx_fifo(nrf24l01* dev) {
    return nrf_ll_cmd_flush_tx(dev);
}

NRF_RESULT nrf_flush_rx_fifo(nrf24l01* dev) {
    return nrf_ll_cmd_flush_rx(dev);
}

NRF_RESULT nrf_nop(nrf24l01* dev) {
    return nrf_ll_cmd_nop(dev);
}

NRF_RESULT nrf_get_status(nrf24l01* nrf, nrf24l01_status* status)
{
    return nrf_ll_get_status(nrf, status);
}

NRF_RESULT nrf_set_data_rate(nrf24l01* nrf, NRF_DATA_RATE rate) {
//    uint8_t reg = 0;
    bool rf_dr_low = rate & 0x01, rf_dr_high = rate & 0x02;
    nrf24l01_reg_rf_setup rf_setup;
    NRF_ERR_CHECK(nrf_ll_get_rf_setup(nrf, &rf_setup));
    if (rf_setup.rf_dr_low != rf_dr_low || rf_setup.rf_dr_high != rf_dr_high) 
    {
        rf_setup.rf_dr_low = rf_dr_low;
        rf_setup.rf_dr_high = rf_dr_high;
        return nrf_ll_set_rf_setup(nrf, &rf_setup);
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_tx_power(nrf24l01* nrf, NRF_TX_PWR rf_pwr) {
    if (rf_pwr != (rf_pwr & 0x03)) {
        return NRF_ERR_INVALID_ARGUMENT;
    }
    nrf24l01_reg_rf_setup rf_setup;
    NRF_ERR_CHECK(nrf_ll_get_rf_setup(nrf, &rf_setup));
    if (rf_setup.rf_pwr != rf_pwr) 
    {
        rf_setup.rf_pwr = rf_pwr;
        return nrf_ll_set_rf_setup(nrf, &rf_setup);
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_ccw(nrf24l01* nrf, bool activate) {
    nrf24l01_reg_rf_setup rf_setup;
    NRF_ERR_CHECK(nrf_ll_get_rf_setup(nrf, &rf_setup));
    if (rf_setup.cont_wave != activate) 
    {
        rf_setup.cont_wave = activate;
        return nrf_ll_set_rf_setup(nrf, &rf_setup);
    }
    return NRF_OK;
}

NRF_RESULT nrf_clear_interrupts(nrf24l01* nrf) {
    nrf24l01_reg_status status = {
        .max_rt = true,
        .tx_ds = true,
        .rx_dr = true,
    };
    return nrf_ll_set_status(nrf, &status);
}

NRF_RESULT nrf_clear_tx_irq(nrf24l01* nrf, NRF_TX_RESULT tx_result)
{
    switch (tx_result)
    {
    case NRF_TX_DS:
        return nrf_clear_irq_tx_ds(nrf);
    case NRF_TX_MAX_RT:
        return nrf_clear_irq_max_rt(nrf);
    default:
        return NRF_ERR;
    }
}

NRF_RESULT nrf_clear_irq_max_rt(nrf24l01* nrf) {
    nrf24l01_reg_status status = {
        .max_rt = true,
    };
    return nrf_ll_set_status(nrf, &status);
}

NRF_RESULT nrf_clear_irq_tx_ds(nrf24l01* nrf) {
    nrf24l01_reg_status status = {
        .tx_ds = true,
    };
    return nrf_ll_set_status(nrf, &status);
}

NRF_RESULT nrf_clear_irq_rx_dr(nrf24l01* nrf) {
    nrf24l01_reg_status status = {
        .rx_dr = true,
    };
    return nrf_ll_set_status(nrf, &status);
}

NRF_RESULT nrf_set_rf_channel(nrf24l01* nrf, uint8_t ch) {
    if (ch >= NRF_RF_CHANNEL_COUNT)
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf24l01_reg_rf_ch rf_ch;
    NRF_ERR_CHECK(nrf_ll_get_rf_ch(nrf, &rf_ch));
    if (rf_ch.rf_ch != ch)
    {
        rf_ch.rf_ch = ch;
        return nrf_ll_set_rf_ch(nrf, &rf_ch);
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_retransmission_count(nrf24l01* nrf, uint8_t count) {
    if (count > NRF_RF_MAX_ARC_VALUE)
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf24l01_reg_setup_retr setup_retr;
    NRF_ERR_CHECK(nrf_ll_get_setup_retr(nrf, &setup_retr));
    if (setup_retr.arc != count)
    {
        setup_retr.arc = count;
        NRF_ERR_CHECK(nrf_ll_set_setup_retr(nrf, &setup_retr));
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_retransmission_count(nrf24l01* nrf, uint8_t* count) {
    nrf24l01_reg_setup_retr setup_retr;
    NRF_ERR_CHECK(nrf_ll_get_setup_retr(nrf, &setup_retr));
    *count = setup_retr.arc;
    return NRF_OK;
}

NRF_RESULT nrf_set_retransmission_delay(nrf24l01* nrf, uint8_t x250ms) {
    if (x250ms > NRF_RF_MAX_ARD_VALUE)
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf24l01_reg_setup_retr setup_retr;
    NRF_ERR_CHECK(nrf_ll_get_setup_retr(nrf, &setup_retr));
    if (setup_retr.ard != x250ms)
    {
        setup_retr.ard = x250ms;
        NRF_ERR_CHECK(nrf_ll_set_setup_retr(nrf, &setup_retr));
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_retransmission_delay(nrf24l01* nrf, uint8_t* x250ms) {
    nrf24l01_reg_setup_retr setup_retr;
    NRF_ERR_CHECK(nrf_ll_get_setup_retr(nrf, &setup_retr));
    *x250ms = setup_retr.ard;
    return NRF_OK;
}

NRF_RESULT nrf_get_address_width(nrf24l01* nrf, NRF_ADDR_WIDTH* width) {    
    nrf24l01_reg_setup_aw setup;
    NRF_ERR_CHECK(nrf_ll_get_setup_aw(nrf, &setup));
    *width = setup.aw;
    return NRF_OK;
}

NRF_RESULT nrf_set_address_width(nrf24l01* nrf, NRF_ADDR_WIDTH width) {
    if (width != (width & NRF_SETUP_AW_MASK))
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf24l01_reg_setup_aw setup = {
        .aw = width,
    };
    return nrf_ll_set_setup_aw(nrf, &setup);
}

NRF_RESULT nrf_enable_rx_pipes(nrf24l01* nrf, uint8_t pipe_mask) {
    if (pipe_mask != (pipe_mask & NRF_PIPE_MASK))
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf_reg_en_rxaddr en_rx_addr = *(nrf_reg_en_rxaddr*)&pipe_mask;
    return nrf_ll_set_en_rx_addr(nrf, &en_rx_addr);
}

NRF_RESULT nrf_enable_rx_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe) {
    nrf_reg_en_rxaddr en_rx_addr;
    NRF_ERR_CHECK(nrf_ll_get_en_rx_addr(nrf, &en_rx_addr));
    uint8_t pipe_bit_mask = 1 << pipe;
    if (!(en_rx_addr & pipe_bit_mask)) {
        en_rx_addr |= pipe_bit_mask;
        return nrf_ll_set_en_rx_addr(nrf, &en_rx_addr);
    }
    return NRF_ERR;
}

NRF_RESULT nrf_disable_rx_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe) {
    nrf_reg_en_rxaddr en_rx_addr;
    NRF_ERR_CHECK(nrf_ll_get_en_rx_addr(nrf, &en_rx_addr));
    uint8_t pipe_bit_mask = 1 << pipe;
    if ((en_rx_addr | ~pipe_bit_mask) & pipe_bit_mask) {
        en_rx_addr &= ~pipe_bit_mask;
        return nrf_ll_set_en_rx_addr(nrf, &en_rx_addr);
    }
    return NRF_ERR;
}

NRF_RESULT nrf_enable_rx_auto_ack(nrf24l01* nrf, uint8_t pipe_mask) {
    if (pipe_mask != (pipe_mask & NRF_PIPE_MASK))
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf_reg_en_aa en_aa = *(nrf_reg_en_aa*)&pipe_mask;
    return nrf_ll_set_en_aa(nrf, &en_aa);
}

NRF_RESULT nrf_enable_rx_auto_ack_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe) {
    nrf_reg_en_aa en_rx_aa;
    NRF_ERR_CHECK(nrf_ll_get_en_aa(nrf, &en_rx_aa));
    uint8_t pipe_bit_mask = 1 << pipe;
    if (!(en_rx_aa & pipe_bit_mask)) {
        en_rx_aa |= pipe_bit_mask;
        return nrf_ll_set_en_aa(nrf, &en_rx_aa);
    }
    return NRF_ERR;
}

NRF_RESULT nrf_disable_rx_auto_ack_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe) {
    nrf_reg_en_aa en_rx_aa;
    NRF_ERR_CHECK(nrf_ll_get_en_aa(nrf, &en_rx_aa));
    uint8_t pipe_bit_mask = 1 << pipe;
    if ((en_rx_aa | ~pipe_bit_mask) & pipe_bit_mask) {
        en_rx_aa &= ~pipe_bit_mask;
        return nrf_ll_set_en_aa(nrf, &en_rx_aa);
    }
    return NRF_ERR;
}

NRF_RESULT nrf_enable_dyn_pl_feature(nrf24l01* nrf, bool enable) {
    nrf24l01_reg_feature feature;
    NRF_ERR_CHECK(nrf_ll_get_feature(nrf, &feature));
    if (feature.en_dpl != enable)
    {
        feature.en_dpl = enable;
        NRF_ERR_CHECK(nrf_ll_set_feature(nrf, &feature)) ;
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_dyn_pl_feature(nrf24l01* nrf, bool* enabled) {
    nrf24l01_reg_feature feature;
    NRF_ERR_CHECK(nrf_ll_get_feature(nrf, &feature));
    *enabled = feature.en_dpl;
    return NRF_OK;
}

NRF_RESULT nrf_enable_ack_pl_feature(nrf24l01* nrf, bool enable) {
    nrf24l01_reg_feature feature;
    NRF_ERR_CHECK(nrf_ll_get_feature(nrf, &feature));
    if (feature.en_ack_pay != enable)
    {
        feature.en_ack_pay = enable;
        NRF_ERR_CHECK(nrf_ll_set_feature(nrf, &feature));
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_ack_pl_feature(nrf24l01* nrf, bool* enabled) {
    nrf24l01_reg_feature feature;
    NRF_ERR_CHECK(nrf_ll_get_feature(nrf, &feature));
    *enabled = feature.en_ack_pay;
    return NRF_OK;
}

NRF_RESULT nrf_enable_dyn_ack_feature(nrf24l01* nrf, bool enable) {
    nrf24l01_reg_feature feature;
    NRF_ERR_CHECK(nrf_ll_get_feature(nrf, &feature));
    if (feature.en_dyn_ack != enable)
    {
        feature.en_dyn_ack = enable;
        NRF_ERR_CHECK(nrf_ll_set_feature(nrf, &feature));
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_dyn_ack_feature(nrf24l01* nrf, bool* enabled) {
    nrf24l01_reg_feature feature;
    NRF_ERR_CHECK(nrf_ll_get_feature(nrf, &feature));
    *enabled = feature.en_dyn_ack;
    return NRF_OK;
}

NRF_RESULT nrf_enable_dyn_pl(nrf24l01* nrf, uint8_t pipe_mask) {
    if (pipe_mask != (pipe_mask & NRF_PIPE_MASK))
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf24l01_reg_dynpd dynpd = pipe_mask;
    return nrf_ll_set_dynpd(nrf, &dynpd);
}

NRF_RESULT nrf_enable_dyn_pl_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe)
{
    nrf24l01_reg_dynpd dynpd;
    NRF_ERR_CHECK(nrf_ll_get_dynpd(nrf, &dynpd));
    uint8_t pipe_bit_mask = 1 << pipe;
    if (!(dynpd & pipe_bit_mask)) {
        dynpd |= pipe_bit_mask;
        return nrf_ll_set_dynpd(nrf, &dynpd);
    }
    return NRF_ERR;
}

NRF_RESULT nrf_disable_dyn_pl_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe) {
    nrf24l01_reg_dynpd dynpd;
    NRF_ERR_CHECK(nrf_ll_get_dynpd(nrf, &dynpd));
    uint8_t pipe_bit_mask = 1 << pipe;
    if ((dynpd | ~pipe_bit_mask) & pipe_bit_mask) {
        dynpd &= ~pipe_bit_mask;
        return nrf_ll_set_dynpd(nrf, &dynpd);
    }
    return NRF_ERR;
}

NRF_RESULT nrf_set_crc_en(nrf24l01* nrf, bool enable) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    if (config.en_crc != enable)
    {
        config.en_crc = enable;
        return nrf_ll_set_config(nrf, &config);
    }
    return NRF_OK;
}

NRF_RESULT nrf_set_crc_width(nrf24l01* nrf, NRF_CRC_WIDTH width) {
    if (width != NRF_CRC_WIDTH_1B && width != NRF_CRC_WIDTH_2B)
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    if (config.crc0 != width)
    {
        config.crc0 = width;
        return nrf_ll_set_config(nrf, &config);
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_crc_width(nrf24l01* nrf, NRF_CRC_WIDTH* width) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    *width = config.crc0;
    return NRF_OK;
}

NRF_RESULT nrf_set_power_up(nrf24l01* nrf, bool power_up) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    if (config.pwr_up != power_up)
    {
        config.pwr_up = power_up;
        return nrf_ll_set_config(nrf, &config);
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_power_up(nrf24l01* nrf, bool* power_up) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    *power_up = config.pwr_up;
    return NRF_OK;
}

NRF_RESULT nrf_set_rx_tx(nrf24l01* nrf, NRF_TXRX_STATE tx_rx) {
    if (tx_rx != NRF_STATE_RX && tx_rx != NRF_STATE_TX)
        return NRF_ERR_INVALID_ARGUMENT;
    
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    if (config.prim_rx != tx_rx)
    {
        config.prim_rx = tx_rx;
        return nrf_ll_set_config(nrf, &config);
    }
    return NRF_OK;
}

NRF_RESULT nrf_get_rx_tx(nrf24l01* nrf, NRF_TXRX_STATE* tx_rx) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    *tx_rx = config.prim_rx;
    return NRF_OK;
}

NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* nrf, bool activate) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    // interrupt configs are inverted, 0 = activate
    bool bit_val = !activate;
    if (config.mask_rx_dr != bit_val)
    {
        config.mask_rx_dr = bit_val;
        NRF_ERR_CHECK(nrf_ll_set_config(nrf, &config));
    }
    return NRF_OK;
}

NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* nrf, bool activate) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    bool bit_val = !activate; // interrupt configs are inverted, 0 = activate
    if (config.mask_tx_ds != bit_val)
    {
        config.mask_tx_ds = bit_val;
        NRF_ERR_CHECK(nrf_ll_set_config(nrf, &config));
    }
    return NRF_OK;
}

NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* nrf, bool activate) {
    nrf24l01_reg_config config;
    NRF_ERR_CHECK(nrf_ll_get_config(nrf, &config));
    bool bit_val = !activate; // interrupt configs are inverted, 0 = activate
    if (config.mask_max_rt != bit_val)
    {
        config.mask_max_rt = bit_val;
        NRF_ERR_CHECK(nrf_ll_set_config(nrf, &config));
    }
    return NRF_OK;
}

NRF_RESULT effective_address(nrf24l01* nrf, uint64_t* address, uint64_t* result)
{
    nrf24l01_reg_setup_aw aw;
    NRF_ERR_CHECK(nrf_ll_get_setup_aw(nrf, &aw));
    uint64_t mask = 0;
    for (int i = 0; i < aw.aw + 2; i++)
    {
        mask |= (uint64_t)0xFF << (8*i);
    }
    *result = *address & mask;
    return NRF_OK;
}

NRF_RESULT nrf_get_tx_address(nrf24l01* nrf, uint64_t* result)
{
    return nrf_ll_get_tx_addr(nrf, result);
}

NRF_RESULT nrf_effective_tx_address(nrf24l01* nrf, uint64_t* result)
{
    uint64_t addr;
    NRF_ERR_CHECK(nrf_ll_get_tx_addr(nrf, &addr));
    return effective_address(nrf, &addr, result);
}

NRF_RESULT nrf_set_tx_address(nrf24l01* nrf, uint64_t address) {
    if (address != (address & NRF_MAX_ADDR_WIDTH_MASK))
        return NRF_ERR_INVALID_ARGUMENT;
    
    return nrf_ll_set_tx_addr(nrf, &address);
}

NRF_RESULT nrf_get_rx_address(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t* result)
{
    if (pipe >= NRF_RX_PIPE_COUNT)
        return NRF_ERR_INVALID_ARGUMENT;
    
    uint8_t address_bytes[NRF_MAX_ADDRESS_WIDTH], len;
    NRF_ERR_CHECK(nrf_ll_get_rx_addr(nrf, pipe, address_bytes, &len));
    *result = 0;
    for (int i = 0; i < len; i++)
    {
        *result |= (uint64_t) address_bytes[i] << (i * 8);
    }
    
    if (pipe > NRF_PIPE_P1)
    {
        // recursively call self to get P1 address
        uint64_t p1_addr;
        NRF_ERR_CHECK(nrf_get_rx_address(nrf, NRF_PIPE_P1, &p1_addr));
        *result = (*result & 0xFF) | (p1_addr & ~(0xFF));
    }
    
    return NRF_OK;
}

NRF_RESULT nrf_effective_rx_address(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t* result)
{
    uint64_t addr;
    NRF_ERR_CHECK(nrf_get_rx_address(nrf, pipe, &addr));
    return effective_address(nrf, &addr, result);
}

NRF_RESULT nrf_set_rx_address(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t address) {
    if (pipe < NRF_PIPE_P0 && pipe > NRF_RX_PIPE_COUNT) {
        return NRF_ERR_INVALID_PIPE;
    }
    
    if (pipe == NRF_PIPE_P0 || pipe == NRF_PIPE_P1) {
        // check long address
        if (address != (address & NRF_MAX_ADDR_WIDTH_MASK))
        {
            return NRF_ERR_INVALID_LONG_ADDRESS;
        }     
    }
    else
    {
        // check short address
        if(address != (address & 0xFF)) {
            {
                return NRF_ERR_INVALID_SHORT_ADDRESS;   
            }
        }
    }
        
    return nrf_ll_set_rx_addr(nrf, pipe, &address);
}

NRF_RESULT nrf_set_rx_payload_width(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t width) {       
    if(width > NRF_MAX_PAYLOAD_WIDTH) return NRF_ERR_INVALID_ARGUMENT;
    
    NRF_ERR_CHECK(nrf_ll_set_rx_pw(nrf, pipe, width)) ;
    return NRF_OK;
}

/* Transmit/Receive functions */

NRF_RESULT nrf_rx_message_available(nrf24l01* nrf, bool* available)
{
    nrf24l01_reg_fifo_status fifo_status;
    NRF_ERR_CHECK(nrf_ll_get_fifo_status(nrf, &fifo_status));
    *available = !fifo_status.rx_empty;
    return NRF_OK;
}

static NRF_RESULT nrf_tx_pre_flight(nrf24l01* nrf, uint8_t len, bool* ok)
{
    if (len > NRF_MAX_PAYLOAD_WIDTH) return NRF_ERR_MSG_LENGTH;
    
    nrf24l01_reg_fifo_status fifo_status;
    NRF_ERR_CHECK(nrf_ll_get_fifo_status(nrf, &fifo_status));
    if (fifo_status.tx_full) return NRF_ERR_TX_FIFO_FULL;
    
    *ok = true;
    return NRF_OK;
}

NRF_RESULT nrf_send_packet(nrf24l01* nrf, uint8_t* data, uint8_t len)
{   
#ifdef NRF_FREERTOS
    return nrf_send_packet_freertos(nrf, data, len);
#else
    bool pre_flight;
    NRF_ERR_CHECK(nrf_tx_pre_flight(nrf, len, &pre_flight));
    
    nrf->tx_busy = 1;
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_TX));
    NRF_ERR_CHECK(nrf_write_tx_payload(nrf, data, len));
    nrf_set_ce(nrf);
    while (nrf->tx_busy == 1) {} // wait for end of transmission
    return nrf->tx_result;
    return NRF_OK;
#endif
}

NRF_RESULT nrf_send_packet_async(nrf24l01* nrf, uint8_t* data, uint8_t len)
{   
    bool pre_flight;
    NRF_ERR_CHECK(nrf_tx_pre_flight(nrf, len, &pre_flight));
    
    nrf->tx_busy = 1;
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_TX));
    NRF_ERR_CHECK(nrf_write_tx_payload(nrf, data, len));
    nrf_set_ce(nrf);
    return NRF_OK;
}

NRF_RESULT nrf_send_packet_noack_async(nrf24l01* nrf, uint8_t* data, uint8_t len)
{
    bool pre_flight;
    NRF_ERR_CHECK(nrf_tx_pre_flight(nrf, len, &pre_flight));
    
    nrf->tx_busy = 1;
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_TX));
    NRF_ERR_CHECK(nrf_write_tx_payload_noack(nrf, data, len));
    nrf_set_ce(nrf);
    return NRF_OK;
}

NRF_RESULT nrf_send_packet_noack(nrf24l01* nrf, uint8_t* data, uint8_t len)
{
    NRF_ERR_CHECK(nrf_send_packet_async(nrf, data, len));
    while (nrf->tx_busy == 1) {} // wait for end of transmission
    return nrf->tx_result;
}


NRF_RESULT nrf_push_packet(nrf24l01* nrf, uint8_t* data, uint8_t len)
{
    bool pre_flight;
    NRF_ERR_CHECK(nrf_tx_pre_flight(nrf, len, &pre_flight));
    
    if (nrf->tx_busy == 1) {
        nrf_flush_tx_fifo(nrf);
    }
    else {
        nrf->tx_busy = 1;
    }
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_set_rx_tx(nrf, NRF_STATE_TX));
    NRF_ERR_CHECK(nrf_write_tx_payload(nrf, data, len));
    nrf_set_ce(nrf);
    return NRF_OK;
}

NRF_RESULT nrf_receive_packet(nrf24l01* nrf) {
    bool msg_available;
    NRF_ERR_CHECK(nrf_rx_message_available(nrf, &msg_available));
    if (!msg_available)
    {
        return NRF_ERR_RX_FIFO_EMPTY;
    }
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_rx_read_payload(nrf));
    nrf_set_ce(nrf);

    return NRF_OK;
}

NRF_RESULT nrf_receive_all(nrf24l01* nrf) {
    bool msg_available;
    NRF_ERR_CHECK(nrf_rx_message_available(nrf, &msg_available));
    if (!msg_available)
    {
        return NRF_ERR_RX_FIFO_EMPTY;
    }
    nrf_reset_ce(nrf);
    NRF_ERR_CHECK(nrf_rx_read_all(nrf));
    nrf_set_ce(nrf);

    return NRF_OK;
}

// GPIO functions

void nrf_set_ce(nrf24l01* nrf)
{
    nrf_ll_set_ce(nrf);
}

void nrf_reset_ce(nrf24l01* nrf)
{
    nrf_ll_reset_ce(nrf);
}
