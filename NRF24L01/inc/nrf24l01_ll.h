#pragma once

#include "nrf24l01_config.h"
#include "nrf24l01.h"

#ifdef NRF_STM32_HAL
#include "nrf24l01_cubemx.h"
#endif

#define NRF_RX_PIPE_COUNT 6
#define NRF_MAX_PAYLOAD_WIDTH  32
#define NRF_SPI_BUFFER_LENGTH NRF_MAX_PAYLOAD_WIDTH + 1

// exit with returned error
#define NRF_ERR_CHECK(x) { NRF_RESULT e = x; if(e != NRF_OK) { return e;} }

// consts 
static const uint64_t NRF_MAX_ADDR_WIDTH_MASK = 0xFFFFFFFFFF;  // 5 bytes
static const uint8_t NRF_RF_MAX_ARD_VALUE = 15;
static const uint8_t NRF_RF_MAX_ARC_VALUE = 15;
static const uint8_t NRF_MAX_ADDRESS_WIDTH = 5;
static const uint8_t NRF_PIPE_MASK = 0x3F;
static const uint8_t NRF_RF_CH_MASK = 0x7F;
static const uint8_t NRF_SETUP_AW_MASK = 0x03;

/* Registers */
typedef enum {
    NRF_REG_CONFIG      = 0x00,
    NRF_REG_EN_AA       = 0x01,
    NRF_REG_EN_RXADDR   = 0x02,
    NRF_REG_SETUP_AW    = 0x03,
    NRF_REG_SETUP_RETR  = 0x04,
    NRF_REG_RF_CH       = 0x05,
    NRF_REG_RF_SETUP    = 0x06,
    NRF_REG_STATUS      = 0x07,
    NRF_REG_OBSERVE_TX  = 0x08,
    NRF_REG_RPD         = 0x09,
    NRF_REG_RX_ADDR_P0  = 0x0A,
    NRF_REG_RX_ADDR_P1  = 0x0B,
    NRF_REG_RX_ADDR_P2  = 0x0C,
    NRF_REG_RX_ADDR_P3  = 0x0D,
    NRF_REG_RX_ADDR_P4  = 0x0E,
    NRF_REG_RX_ADDR_P5  = 0x0F,
    NRF_REG_TX_ADDR     = 0x10,
    NRF_REG_RX_PW_P0    = 0x11,
    NRF_REG_RX_PW_P1    = 0x12,
    NRF_REG_RX_PW_P2    = 0x13,
    NRF_REG_RX_PW_P3    = 0x14,
    NRF_REG_RX_PW_P4    = 0x15,
    NRF_REG_RX_PW_P5    = 0x16,
    NRF_REG_FIFO_STATUS = 0x17,
    NRF_REG_DYNPD       = 0x1C,
    NRF_REG_FEATURE     = 0x1D
} NRF_REGISTER;

/* Register bit maps */
typedef struct
{
    bool prim_rx : 1;
    bool pwr_up : 1;
    bool crc0 : 1;
    bool en_crc : 1;
    bool mask_max_rt : 1;
    bool mask_tx_ds : 1;
    bool mask_rx_dr : 1;
    bool reserved_0 : 1;
    
} nrf24l01_reg_config;

typedef uint8_t nrf_reg_en_aa;

typedef uint8_t nrf_reg_en_rxaddr;

typedef struct
{
    uint8_t aw : 2;
    int reserved_0 : 6;
    
} nrf24l01_reg_setup_aw;

typedef struct
{
    uint8_t arc : 4;
    uint8_t ard : 4;
    
} nrf24l01_reg_setup_retr;

typedef struct
{
    uint8_t rf_ch : 7;
    bool reserved_0 : 1;
    
} nrf24l01_reg_rf_ch;

typedef struct
{
    bool reserved0_0 : 1;
    uint8_t rf_pwr : 2;
    bool rf_dr_high : 1;
    bool pll_lock : 1;
    bool rf_dr_low : 1;
    bool reserved1_0 : 1;
    bool cont_wave : 1;
    
} nrf24l01_reg_rf_setup;

typedef nrf24l01_status nrf24l01_reg_status;

typedef struct
{
    uint8_t arc_cnt : 4;
    uint8_t plos_cnt : 4;
    
} nrf24l01_reg_observe_tx;

typedef struct
{
    bool rpd : 1;           // RO
    uint8_t reserved_0 : 7;
    
} nrf24l01_reg_rpd;

typedef struct
{
    uint8_t address[5];
    
} nrf24l01_reg_rx_addr_p0;

typedef struct
{
    uint8_t address[5];
    
} nrf24l01_reg_rx_addr_p1;

typedef struct
{
    uint8_t address;
    
} nrf24l01_reg_rx_addr_p2;

typedef struct
{
    uint8_t address;
    
} nrf24l01_reg_rx_addr_p3;

typedef struct
{
    uint8_t address;
    
} nrf24l01_reg_rx_addr_p4;

typedef struct
{
    uint8_t address;
    
} nrf24l01_reg_rx_addr_p5;

typedef struct
{
    uint8_t address[5];
    
} nrf24l01_reg_tx_addr;

typedef struct
{
    uint8_t rx_pw_p0 : 6;
    uint8_t reserved_0 : 2;

} nrf24l01_reg_rx_pw_p0;

typedef struct
{
    uint8_t rx_pw_p1 : 6;
    uint8_t reserved_0 : 2;

} nrf24l01_reg_rx_pw_p1;

typedef struct
{
    uint8_t rx_pw_p2 : 6;
    uint8_t reserved_0 : 2;

} nrf24l01_reg_rx_pw_p2;

typedef struct
{
    uint8_t rx_pw_p3 : 6;
    uint8_t reserved_0 : 2;

} nrf24l01_reg_rx_pw_p3;

typedef struct
{
    uint8_t rx_pw_p4 : 6;
    uint8_t reserved_0 : 2;

} nrf24l01_reg_rx_pw_p4;

typedef struct
{
    uint8_t rx_pw_p5 : 6;
    uint8_t reserved_0 : 2;
    
} nrf24l01_reg_rx_pw_p5;

typedef struct
{
    bool rx_empty : 1;    // RO
    bool rx_full : 1;    // RO
    uint8_t reserved0_0 : 2;
    bool tx_empty : 1;    // RO
    bool tx_full : 1;    // RO
    bool tx_reuse : 1;    // RO
    bool reserved1_0 : 1;

} nrf24l01_reg_fifo_status;

typedef uint8_t nrf24l01_reg_dynpd;

typedef struct
{
    bool en_dyn_ack : 1;
    bool en_ack_pay : 1;
    bool en_dpl : 1;;
    uint8_t reserved_0 : 5;
    
} nrf24l01_reg_feature;

// register getters
NRF_RESULT nrf_ll_get_config(nrf24l01* nrf, nrf24l01_reg_config* result);
NRF_RESULT nrf_ll_get_en_aa(nrf24l01* nrf, nrf_reg_en_aa* result);
NRF_RESULT nrf_ll_get_en_rx_addr(nrf24l01* nrf, nrf_reg_en_rxaddr* result);
NRF_RESULT nrf_ll_get_setup_aw(nrf24l01* nrf, nrf24l01_reg_setup_aw* result);
NRF_RESULT nrf_ll_get_setup_retr(nrf24l01* nrf, nrf24l01_reg_setup_retr* result);
NRF_RESULT nrf_ll_get_rf_ch(nrf24l01* nrf, nrf24l01_reg_rf_ch* result);
NRF_RESULT nrf_ll_get_rf_setup(nrf24l01* nrf, nrf24l01_reg_rf_setup* result);
NRF_RESULT nrf_ll_get_status(nrf24l01* nrf, nrf24l01_reg_status* result);
NRF_RESULT nrf_ll_get_observe_tx(nrf24l01* nrf, nrf24l01_reg_observe_tx* result);
NRF_RESULT nrf_ll_get_rpd(nrf24l01* nrf, nrf24l01_reg_rpd* result);
NRF_RESULT nrf_ll_get_rx_addr(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t* result, uint8_t* len);
NRF_RESULT nrf_ll_get_rx_addr_p0(nrf24l01* nrf, nrf24l01_reg_rx_addr_p0* result);
NRF_RESULT nrf_ll_get_rx_addr_p1(nrf24l01* nrf, nrf24l01_reg_rx_addr_p1* result);
NRF_RESULT nrf_ll_get_rx_addr_p2(nrf24l01* nrf, nrf24l01_reg_rx_addr_p2* result);
NRF_RESULT nrf_ll_get_rx_addr_p3(nrf24l01* nrf, nrf24l01_reg_rx_addr_p3* result);
NRF_RESULT nrf_ll_get_rx_addr_p4(nrf24l01* nrf, nrf24l01_reg_rx_addr_p4* result);
NRF_RESULT nrf_ll_get_rx_addr_p5(nrf24l01* nrf, nrf24l01_reg_rx_addr_p5* result);
NRF_RESULT nrf_ll_get_tx_addr(nrf24l01* nrf, uint64_t* result);
NRF_RESULT nrf_ll_get_rx_pw(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t* result);
NRF_RESULT nrf_ll_get_rx_pw_p0(nrf24l01* nrf, nrf24l01_reg_rx_pw_p0* result);
NRF_RESULT nrf_ll_get_rx_pw_p1(nrf24l01* nrf, nrf24l01_reg_rx_pw_p1* result);
NRF_RESULT nrf_ll_get_rx_pw_p2(nrf24l01* nrf, nrf24l01_reg_rx_pw_p2* result);
NRF_RESULT nrf_ll_get_rx_pw_p3(nrf24l01* nrf, nrf24l01_reg_rx_pw_p3* result);
NRF_RESULT nrf_ll_get_rx_pw_p4(nrf24l01* nrf, nrf24l01_reg_rx_pw_p4* result);
NRF_RESULT nrf_ll_get_rx_pw_p5(nrf24l01* nrf, nrf24l01_reg_rx_pw_p5* result);
NRF_RESULT nrf_ll_get_fifo_status(nrf24l01* nrf, nrf24l01_reg_fifo_status* result);
NRF_RESULT nrf_ll_get_dynpd(nrf24l01* nrf, nrf24l01_reg_dynpd* result);
NRF_RESULT nrf_ll_get_feature(nrf24l01* nrf, nrf24l01_reg_feature* result);

// register setters
NRF_RESULT nrf_ll_set_config(nrf24l01* nrf, nrf24l01_reg_config* value);
NRF_RESULT nrf_ll_set_en_aa(nrf24l01* nrf, nrf_reg_en_aa* value);
NRF_RESULT nrf_ll_set_en_rx_addr(nrf24l01* nrf, nrf_reg_en_rxaddr* value);
NRF_RESULT nrf_ll_set_setup_aw(nrf24l01* nrf, nrf24l01_reg_setup_aw* value);
NRF_RESULT nrf_ll_set_setup_retr(nrf24l01* nrf, nrf24l01_reg_setup_retr* value);
NRF_RESULT nrf_ll_set_rf_ch(nrf24l01* nrf, nrf24l01_reg_rf_ch* value);
NRF_RESULT nrf_ll_set_rf_setup(nrf24l01* nrf, nrf24l01_reg_rf_setup* value);
NRF_RESULT nrf_ll_set_status(nrf24l01* nrf, nrf24l01_reg_status* value);
NRF_RESULT nrf_ll_set_rx_addr(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t* value);
NRF_RESULT nrf_ll_set_rx_addr_p0(nrf24l01* nrf, nrf24l01_reg_rx_addr_p0* value);
NRF_RESULT nrf_ll_set_rx_addr_p1(nrf24l01* nrf, nrf24l01_reg_rx_addr_p1* value);
NRF_RESULT nrf_ll_set_rx_addr_p2(nrf24l01* nrf, nrf24l01_reg_rx_addr_p2* value);
NRF_RESULT nrf_ll_set_rx_addr_p3(nrf24l01* nrf, nrf24l01_reg_rx_addr_p3* value);
NRF_RESULT nrf_ll_set_rx_addr_p4(nrf24l01* nrf, nrf24l01_reg_rx_addr_p4* value);
NRF_RESULT nrf_ll_set_rx_addr_p5(nrf24l01* nrf, nrf24l01_reg_rx_addr_p5* value);
NRF_RESULT nrf_ll_set_tx_addr(nrf24l01* nrf, uint64_t* value);
NRF_RESULT nrf_ll_set_rx_pw(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t value);
NRF_RESULT nrf_ll_set_rx_pw_p0(nrf24l01* nrf, nrf24l01_reg_rx_pw_p0* value);
NRF_RESULT nrf_ll_set_rx_pw_p1(nrf24l01* nrf, nrf24l01_reg_rx_pw_p1* value);
NRF_RESULT nrf_ll_set_rx_pw_p2(nrf24l01* nrf, nrf24l01_reg_rx_pw_p2* value);
NRF_RESULT nrf_ll_set_rx_pw_p3(nrf24l01* nrf, nrf24l01_reg_rx_pw_p3* value);
NRF_RESULT nrf_ll_set_rx_pw_p4(nrf24l01* nrf, nrf24l01_reg_rx_pw_p4* value);
NRF_RESULT nrf_ll_set_rx_pw_p5(nrf24l01* nrf, nrf24l01_reg_rx_pw_p5* value);
NRF_RESULT nrf_ll_set_dynpd(nrf24l01* nrf, nrf24l01_reg_dynpd* value);
NRF_RESULT nrf_ll_set_feature(nrf24l01* nrf, nrf24l01_reg_feature* value);

// commands
NRF_RESULT nrf_ll_cmd_r_register(nrf24l01* dev, NRF_REGISTER reg, uint8_t* data, uint8_t len);
NRF_RESULT nrf_ll_cmd_w_register(nrf24l01* dev, NRF_REGISTER reg, const uint8_t* data, uint8_t len);
NRF_RESULT nrf_ll_cmd_r_rx_payload(nrf24l01* nrf, uint8_t* data, uint8_t len);
NRF_RESULT nrf_ll_cmd_w_tx_payload(nrf24l01* nrf, const uint8_t* data, uint8_t len);
NRF_RESULT nrf_ll_cmd_flush_tx(nrf24l01* nrf);
NRF_RESULT nrf_ll_cmd_flush_rx(nrf24l01* nrf);
NRF_RESULT nrf_ll_cmd_reuse_tx_pl(nrf24l01* nrf);
NRF_RESULT nrf_ll_cmd_r_rx_pl_wid(nrf24l01* dev, uint8_t* data);
NRF_RESULT nrf_ll_cmd_w_ack_payload(nrf24l01* nrf, const uint8_t* data, uint8_t len);
NRF_RESULT nrf_ll_cmd_w_tx_payload_noack(nrf24l01* nrf, const uint8_t* data, uint8_t len);
NRF_RESULT nrf_ll_cmd_nop(nrf24l01* nrf);

// io operations
void nrf_ll_set_ce(nrf24l01* nrf);
void nrf_ll_reset_ce(nrf24l01* nrf);
