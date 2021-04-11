#include "nrf24l01_ll.h"

#include <stdint.h>
#include <string.h>
#include <assert.h>

static const uint8_t ZERO_DATA_BYTES = 0;
static const uint8_t SINGLE_DATA_BYTE = 1;

// forward declarations
typedef enum NRF_PIN NRF_PIN;
static void nrf_ll_gpio(nrf24l01* nrf, NRF_PIN pin, NRF_PIN_STATE state);
static NRF_RESULT nrf_ll_spi(nrf24l01* nrf, uint8_t* tx, uint8_t* rx, uint8_t length);
static NRF_RESULT nrf_ll_send_command(nrf24l01* nrf, uint8_t command, const uint8_t* tx, uint8_t* rx, uint8_t len);

/* Commands */
typedef enum {
    NRF_CMD_R_REGISTER         = 0x00,
    NRF_CMD_W_REGISTER         = 0x20,
    NRF_CMD_R_RX_PAYLOAD       = 0x61,
    NRF_CMD_W_TX_PAYLOAD       = 0xA0,
    NRF_CMD_FLUSH_TX           = 0xE1,
    NRF_CMD_FLUSH_RX           = 0xE2,
    NRF_CMD_REUSE_TX_PL        = 0xE3,
    NRF_CMD_ACTIVATE           = 0x50,
    NRF_CMD_R_RX_PL_WID        = 0x60,
    NRF_CMD_W_ACK_PAYLOAD      = 0xA8,
    NRF_CMD_W_TX_PAYLOAD_NOACK = 0xB0,
    NRF_CMD_NOP                = 0xFF
} NRF_COMMAND;

/* GPIO Pins */
enum NRF_PIN {
    NRF_GPIO_CE,
    NRF_GPIO_CSN,
};

/* Command functions */
NRF_RESULT nrf_ll_spi_read(nrf24l01* nrf, uint8_t command, uint8_t* rx_data, uint8_t len)
{
    return nrf_ll_send_command(nrf, command, NULL, rx_data, len);
}

NRF_RESULT nrf_ll_spi_write(nrf24l01* nrf, uint8_t command, const uint8_t* tx_data, uint8_t len)
{
    return nrf_ll_send_command(nrf, command, tx_data, NULL, len);
}

NRF_RESULT nrf_ll_spi_cmd(nrf24l01* nrf, uint8_t command)
{
    return nrf_ll_send_command(nrf, command, NULL, NULL, ZERO_DATA_BYTES);
}

static NRF_RESULT nrf_ll_send_command(nrf24l01* nrf, uint8_t command, const uint8_t* tx_data, uint8_t* rx_data, uint8_t len) {
    static uint8_t tx_buffer[NRF_SPI_BUFFER_LENGTH];
    static uint8_t rx_buffer[NRF_SPI_BUFFER_LENGTH];
    
    // clear buffers - probably not necessary
    memset(tx_buffer, 0, NRF_SPI_BUFFER_LENGTH);
    memset(rx_buffer, 0, NRF_SPI_BUFFER_LENGTH);
                                
    // command in first byte
    tx_buffer[0] = command;
    
    // populate tx buffer
    if (tx_data != NULL) {
        memcpy(tx_buffer + 1, tx_data, len);
    }
                                   
    // do SPI transaction
    NRF_ERR_CHECK(nrf_ll_spi(nrf, tx_buffer, rx_buffer, len + 1));
    
    // store RX result
    if(rx_data != NULL) {
        memcpy(rx_data, rx_buffer + 1, len);
    }
    
    // store received status reg
    nrf->status = *(nrf24l01_reg_status*)&rx_buffer[0];
        
    return NRF_OK;
}

NRF_RESULT nrf_ll_cmd_r_register(nrf24l01* nrf, NRF_REGISTER reg, uint8_t* data, uint8_t len) {
    assert(len <= NRF_MAX_ADDRESS_WIDTH);
    return nrf_ll_spi_read(nrf, NRF_CMD_R_REGISTER | reg, data, len);
}

NRF_RESULT nrf_ll_cmd_w_register(nrf24l01* nrf, NRF_REGISTER reg, const uint8_t* data, uint8_t len) {
    assert(len <= NRF_MAX_ADDRESS_WIDTH);
    return nrf_ll_spi_write(nrf, NRF_CMD_W_REGISTER | reg, data, len);
}

NRF_RESULT nrf_ll_cmd_r_rx_payload(nrf24l01* nrf, uint8_t* data, uint8_t len)
{
    assert(len <= NRF_MAX_PAYLOAD_WIDTH);
    return nrf_ll_spi_read(nrf, NRF_CMD_R_RX_PAYLOAD, data, len);
}

NRF_RESULT nrf_ll_cmd_w_tx_payload(nrf24l01* nrf, const uint8_t* data, uint8_t len)
{
    assert(len <= NRF_MAX_PAYLOAD_WIDTH);
    return nrf_ll_spi_write(nrf, NRF_CMD_W_TX_PAYLOAD, data, len);
}

NRF_RESULT nrf_ll_cmd_flush_tx(nrf24l01* nrf)
{
    return nrf_ll_spi_cmd(nrf, NRF_CMD_FLUSH_TX);
}

NRF_RESULT nrf_ll_cmd_flush_rx(nrf24l01* nrf)
{
    return nrf_ll_spi_cmd(nrf, NRF_CMD_FLUSH_RX);
}

NRF_RESULT nrf_ll_cmd_reuse_tx_pl(nrf24l01* nrf)
{
    return nrf_ll_spi_cmd(nrf, NRF_CMD_REUSE_TX_PL);
}

NRF_RESULT nrf_ll_cmd_r_rx_pl_wid(nrf24l01* nrf, uint8_t* data)
{
    return nrf_ll_spi_read(nrf, NRF_CMD_R_RX_PL_WID, data, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_cmd_w_ack_payload(nrf24l01* nrf, const uint8_t* data, uint8_t len)
{
    assert(len <= NRF_MAX_PAYLOAD_WIDTH);
    return nrf_ll_spi_write(nrf, NRF_CMD_W_ACK_PAYLOAD, data, len);
}

NRF_RESULT nrf_ll_cmd_w_tx_payload_noack(nrf24l01* nrf, const uint8_t* data, uint8_t len)
{
    assert(len <= NRF_MAX_PAYLOAD_WIDTH);
    return nrf_ll_spi_write(nrf, NRF_CMD_W_TX_PAYLOAD_NOACK, data, len);
}

NRF_RESULT nrf_ll_cmd_nop(nrf24l01* nrf)
{
    return nrf_ll_spi_cmd(nrf, NRF_CMD_NOP);
}

/* Register Getters */

NRF_RESULT nrf_ll_get_config(nrf24l01* nrf, nrf24l01_reg_config* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_CONFIG, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_en_aa(nrf24l01* nrf, nrf_reg_en_aa* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_EN_AA, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_en_rx_addr(nrf24l01* nrf, nrf_reg_en_rxaddr* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_EN_RXADDR, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_setup_aw(nrf24l01* nrf, nrf24l01_reg_setup_aw* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_SETUP_AW, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_setup_retr(nrf24l01* nrf, nrf24l01_reg_setup_retr* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_SETUP_RETR, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rf_ch(nrf24l01* nrf, nrf24l01_reg_rf_ch* result)
{    
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RF_CH, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rf_setup(nrf24l01* nrf, nrf24l01_reg_rf_setup* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RF_SETUP, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_status(nrf24l01* nrf, nrf24l01_reg_status* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_STATUS, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_observe_tx(nrf24l01* nrf, nrf24l01_reg_observe_tx* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_OBSERVE_TX, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rpd(nrf24l01* nrf, nrf24l01_reg_rpd* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RPD, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_addr(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t* result, uint8_t* len)
{
    switch (pipe)
    {
    case NRF_PIPE_P0:
        {
            uint64_t addr;
            NRF_ERR_CHECK(nrf_ll_get_rx_addr_p0(nrf, (nrf24l01_reg_rx_addr_p0*)&addr));
            *(uint64_t*)result = addr & NRF_MAX_ADDR_WIDTH_MASK;
            *len = NRF_MAX_ADDRESS_WIDTH;
         }
        break;
    case NRF_PIPE_P1:
        {
            uint64_t addr;
            NRF_ERR_CHECK(nrf_ll_get_rx_addr_p1(nrf, (nrf24l01_reg_rx_addr_p1*)&addr));      
            *(uint64_t*)result = addr & NRF_MAX_ADDR_WIDTH_MASK;
            *len = NRF_MAX_ADDRESS_WIDTH;
        }
        break;
    case NRF_PIPE_P2:
        NRF_ERR_CHECK(nrf_ll_get_rx_addr_p2(nrf, (nrf24l01_reg_rx_addr_p2*)result));
        *len = 1;
        break;
    case NRF_PIPE_P3:
        NRF_ERR_CHECK(nrf_ll_get_rx_addr_p3(nrf, (nrf24l01_reg_rx_addr_p3*)result));
        *len = 1;
        break;
    case NRF_PIPE_P4:
        NRF_ERR_CHECK(nrf_ll_get_rx_addr_p4(nrf, (nrf24l01_reg_rx_addr_p4*)result));
        *len = 1;
        break;
    case NRF_PIPE_P5:
        NRF_ERR_CHECK(nrf_ll_get_rx_addr_p5(nrf, (nrf24l01_reg_rx_addr_p5*)result));
        *len = 1;
        break;
    default:
        return NRF_ERR_INVALID_PIPE;
        break;
    }
    return NRF_OK;
}

NRF_RESULT nrf_ll_get_rx_addr_p0(nrf24l01* nrf, nrf24l01_reg_rx_addr_p0* result)
{    
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_ADDR_P0, (uint8_t*) result, NRF_MAX_ADDRESS_WIDTH);
}

NRF_RESULT nrf_ll_get_rx_addr_p1(nrf24l01* nrf, nrf24l01_reg_rx_addr_p1* result)
{    
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_ADDR_P1, (uint8_t*) result, NRF_MAX_ADDRESS_WIDTH);
}

NRF_RESULT nrf_ll_get_rx_addr_p2(nrf24l01* nrf, nrf24l01_reg_rx_addr_p2* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_ADDR_P2, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_addr_p3(nrf24l01* nrf, nrf24l01_reg_rx_addr_p3* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_ADDR_P3, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_addr_p4(nrf24l01* nrf, nrf24l01_reg_rx_addr_p4* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_ADDR_P4, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_addr_p5(nrf24l01* nrf, nrf24l01_reg_rx_addr_p5* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_ADDR_P5, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_tx_addr(nrf24l01* nrf, uint64_t* result)
{
    uint64_t addr;
    NRF_ERR_CHECK(nrf_ll_cmd_r_register(nrf, NRF_REG_TX_ADDR, (uint8_t*) &addr, NRF_MAX_ADDRESS_WIDTH));
    *result = addr & NRF_MAX_ADDR_WIDTH_MASK;
    return NRF_OK;
}

NRF_RESULT nrf_ll_get_rx_pw(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t* result)
{
    switch (pipe)
    {
    case NRF_PIPE_P0:
        return nrf_ll_get_rx_pw_p0(nrf, (nrf24l01_reg_rx_pw_p0*)result);
    case NRF_PIPE_P1:
        return nrf_ll_get_rx_pw_p1(nrf, (nrf24l01_reg_rx_pw_p1*)result);
    case NRF_PIPE_P2:
        return nrf_ll_get_rx_pw_p2(nrf, (nrf24l01_reg_rx_pw_p2*)result);
    case NRF_PIPE_P3:
        return nrf_ll_get_rx_pw_p3(nrf, (nrf24l01_reg_rx_pw_p3*)result);
    case NRF_PIPE_P4:
        return nrf_ll_get_rx_pw_p4(nrf, (nrf24l01_reg_rx_pw_p4*)result);
    case NRF_PIPE_P5:
        return nrf_ll_get_rx_pw_p5(nrf, (nrf24l01_reg_rx_pw_p5*)result);
    default:
        return NRF_ERR_INVALID_PIPE;
        break;
    }
    return NRF_OK;
}

NRF_RESULT nrf_ll_get_rx_pw_p0(nrf24l01* nrf, nrf24l01_reg_rx_pw_p0* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_PW_P0, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_pw_p1(nrf24l01* nrf, nrf24l01_reg_rx_pw_p1* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_PW_P1, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_pw_p2(nrf24l01* nrf, nrf24l01_reg_rx_pw_p2* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_PW_P2, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_pw_p3(nrf24l01* nrf, nrf24l01_reg_rx_pw_p3* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_PW_P3, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_pw_p4(nrf24l01* nrf, nrf24l01_reg_rx_pw_p4* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_PW_P4, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_rx_pw_p5(nrf24l01* nrf, nrf24l01_reg_rx_pw_p5* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_RX_PW_P5, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_fifo_status(nrf24l01* nrf, nrf24l01_reg_fifo_status* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_FIFO_STATUS, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_dynpd(nrf24l01* nrf, nrf24l01_reg_dynpd* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_DYNPD, (uint8_t*) result, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_get_feature(nrf24l01* nrf, nrf24l01_reg_feature* result)
{
    return nrf_ll_cmd_r_register(nrf, NRF_REG_FEATURE, (uint8_t*) result, SINGLE_DATA_BYTE);
}

/* Register Setters */

NRF_RESULT nrf_ll_set_config(nrf24l01* nrf, nrf24l01_reg_config* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_CONFIG, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_en_aa(nrf24l01* nrf, nrf_reg_en_aa* value)
{
    assert(*value == (*value & NRF_PIPE_MASK));
    return nrf_ll_cmd_w_register(nrf, NRF_REG_EN_AA, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_en_rx_addr(nrf24l01* nrf, nrf_reg_en_rxaddr* value)
{
    assert(*value == (*value & NRF_PIPE_MASK));
    return nrf_ll_cmd_w_register(nrf, NRF_REG_EN_RXADDR, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_setup_aw(nrf24l01* nrf, nrf24l01_reg_setup_aw* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_SETUP_AW, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_setup_retr(nrf24l01* nrf, nrf24l01_reg_setup_retr* value)
{
    return nrf_ll_cmd_w_register(nrf, NRF_REG_SETUP_RETR, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_rf_ch(nrf24l01* nrf, nrf24l01_reg_rf_ch* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RF_CH, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_rf_setup(nrf24l01* nrf, nrf24l01_reg_rf_setup* value)
{
    value->reserved0_0 = 0;
    value->reserved1_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RF_SETUP, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_status(nrf24l01* nrf, nrf24l01_reg_status* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_STATUS, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_rx_addr(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t* value)
{
    switch (pipe)
    {
    case NRF_PIPE_P0:
        NRF_ERR_CHECK(nrf_ll_set_rx_addr_p0(nrf, (nrf24l01_reg_rx_addr_p0*)value));
        break;
    case NRF_PIPE_P1:
        NRF_ERR_CHECK(nrf_ll_set_rx_addr_p1(nrf, (nrf24l01_reg_rx_addr_p1*)value));
        break;
    case NRF_PIPE_P2:
        NRF_ERR_CHECK(nrf_ll_set_rx_addr_p2(nrf, (nrf24l01_reg_rx_addr_p2*)value));
        break;
    case NRF_PIPE_P3:
        NRF_ERR_CHECK(nrf_ll_set_rx_addr_p3(nrf, (nrf24l01_reg_rx_addr_p3*)value));
        break;
    case NRF_PIPE_P4:
        NRF_ERR_CHECK(nrf_ll_set_rx_addr_p4(nrf, (nrf24l01_reg_rx_addr_p4*)value));
        break;
    case NRF_PIPE_P5:
        NRF_ERR_CHECK(nrf_ll_set_rx_addr_p5(nrf, (nrf24l01_reg_rx_addr_p5*)value));
        break;
    default:
        return NRF_ERR_INVALID_PIPE;
        break;
    }
    return NRF_OK;
}

NRF_RESULT nrf_ll_set_rx_addr_p0(nrf24l01* nrf, nrf24l01_reg_rx_addr_p0* value)
{    
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_ADDR_P0, (uint8_t*) value, NRF_MAX_ADDRESS_WIDTH);
}

NRF_RESULT nrf_ll_set_rx_addr_p1(nrf24l01* nrf, nrf24l01_reg_rx_addr_p1* value)
{    
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_ADDR_P1, (uint8_t*) value, NRF_MAX_ADDRESS_WIDTH);
}

NRF_RESULT nrf_ll_set_rx_addr_p2(nrf24l01* nrf, nrf24l01_reg_rx_addr_p2* value)
{
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_ADDR_P2, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_rx_addr_p3(nrf24l01* nrf, nrf24l01_reg_rx_addr_p3* value)
{
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_ADDR_P3, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_rx_addr_p4(nrf24l01* nrf, nrf24l01_reg_rx_addr_p4* value)
{
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_ADDR_P4, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_rx_addr_p5(nrf24l01* nrf, nrf24l01_reg_rx_addr_p5* value)
{
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_ADDR_P5, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_tx_addr(nrf24l01* nrf, uint64_t* value)
{
    return nrf_ll_cmd_w_register(nrf, NRF_REG_TX_ADDR, (uint8_t*) value, NRF_MAX_ADDRESS_WIDTH);
}

NRF_RESULT nrf_ll_set_rx_pw(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t value)
{
    switch (pipe)
    {
    case NRF_PIPE_P0:
        return nrf_ll_set_rx_pw_p0(nrf, (nrf24l01_reg_rx_pw_p0*)&value);
    case NRF_PIPE_P1:
        return nrf_ll_set_rx_pw_p1(nrf, (nrf24l01_reg_rx_pw_p1*)&value);
    case NRF_PIPE_P2:
        return nrf_ll_set_rx_pw_p2(nrf, (nrf24l01_reg_rx_pw_p2*)&value);
    case NRF_PIPE_P3:
        return nrf_ll_set_rx_pw_p3(nrf, (nrf24l01_reg_rx_pw_p3*)&value);
    case NRF_PIPE_P4:
        return nrf_ll_set_rx_pw_p4(nrf, (nrf24l01_reg_rx_pw_p4*)&value);
    case NRF_PIPE_P5:
        return nrf_ll_set_rx_pw_p5(nrf, (nrf24l01_reg_rx_pw_p5*)&value);
    default:
        return NRF_ERR_INVALID_PIPE;
        break;
    }
    return NRF_OK;
}

NRF_RESULT nrf_ll_set_rx_pw_p0(nrf24l01* nrf, nrf24l01_reg_rx_pw_p0* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_PW_P0, (uint8_t*) value, SINGLE_DATA_BYTE);
}
NRF_RESULT nrf_ll_set_rx_pw_p1(nrf24l01* nrf, nrf24l01_reg_rx_pw_p1* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_PW_P1, (uint8_t*) value, SINGLE_DATA_BYTE);
}
NRF_RESULT nrf_ll_set_rx_pw_p2(nrf24l01* nrf, nrf24l01_reg_rx_pw_p2* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_PW_P2, (uint8_t*) value, SINGLE_DATA_BYTE);
}
NRF_RESULT nrf_ll_set_rx_pw_p3(nrf24l01* nrf, nrf24l01_reg_rx_pw_p3* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_PW_P3, (uint8_t*) value, SINGLE_DATA_BYTE);
}
NRF_RESULT nrf_ll_set_rx_pw_p4(nrf24l01* nrf, nrf24l01_reg_rx_pw_p4* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_PW_P4, (uint8_t*) value, SINGLE_DATA_BYTE);
}
NRF_RESULT nrf_ll_set_rx_pw_p5(nrf24l01* nrf, nrf24l01_reg_rx_pw_p5* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_RX_PW_P5, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_dynpd(nrf24l01* nrf, nrf24l01_reg_dynpd* value)
{
    assert(*value == (*value & NRF_PIPE_MASK));
    return nrf_ll_cmd_w_register(nrf, NRF_REG_DYNPD, (uint8_t*) value, SINGLE_DATA_BYTE);
}

NRF_RESULT nrf_ll_set_feature(nrf24l01* nrf, nrf24l01_reg_feature* value)
{
    value->reserved_0 = 0;
    return nrf_ll_cmd_w_register(nrf, NRF_REG_FEATURE, (uint8_t*) value, SINGLE_DATA_BYTE);
}

/* I/O Operations */

static void nrf_ll_gpio(nrf24l01* nrf, NRF_PIN pin, NRF_PIN_STATE state)
{
    switch (pin)
    {
    case NRF_GPIO_CE:
        switch (state)
        {
        case NRF_PIN_SET:
            return nrf->io.ce_set(nrf);
        case NRF_PIN_RESET:
            return nrf->io.ce_reset(nrf);
        default:
            break;
        }
        break;
    case NRF_GPIO_CSN:
        switch (state)
        {
        case NRF_PIN_SET:
            return nrf->io.csn_set(nrf);
        case NRF_PIN_RESET:
            return nrf->io.csn_reset(nrf);
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static NRF_RESULT nrf_ll_spi(nrf24l01* nrf, uint8_t* tx, uint8_t* rx, uint8_t length)
{
    nrf_ll_gpio(nrf, NRF_GPIO_CSN, NRF_PIN_RESET);
    NRF_RESULT err = nrf->io.spi_txrx(nrf, tx, rx, length);
    nrf_ll_gpio(nrf, NRF_GPIO_CSN, NRF_PIN_SET);
    return err == NRF_OK ? NRF_OK : NRF_ERR_SPI;
}

void nrf_ll_set_ce(nrf24l01* nrf)
{
    nrf_ll_gpio(nrf, NRF_GPIO_CE, NRF_PIN_SET);
}

void nrf_ll_reset_ce(nrf24l01* nrf)
{
    nrf_ll_gpio(nrf, NRF_GPIO_CE, NRF_PIN_RESET);
}
