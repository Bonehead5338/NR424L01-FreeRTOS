#pragma once
#pragma weak nrf_irq_isr

#include "nrf24l01_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef NRF_FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "task.h"
#endif // NRF_FREERTOS

#define NRF_SET_BITS_IF(reg, cond, mask) if (cond) { (reg) |= (mask); } else { (reg) &= ~(mask); }
#define NRF_SET_BITS(reg, mask, value) {(reg) &= ~(mask); (reg) |= ((value) & (mask));}
#define NRF_ADDR_FROM_P1(nrf, a) (a) = ((nrf)->rx_address[1] & ~(0xFF)) | (a);

typedef struct nrf24l01 nrf24l01;
typedef enum NRF_RESULT NRF_RESULT;

#ifdef NRF_FREERTOS
extern NRF_RESULT nrf_send_packet_freertos(nrf24l01* nrf, uint8_t* data, uint8_t len);
extern void nrf_freertos_isr(nrf24l01* nrf);
#endif // NRF_FREERTOS

typedef enum {
    NRF_TX,
    NRF_RX,
} NRF_TX_RX;

typedef enum {
    NRF_DATA_RATE_1MBPS   = 0x00,
    NRF_DATA_RATE_250KBPS = 0x10,
    NRF_DATA_RATE_2MBPS   = 0x01,
} NRF_DATA_RATE;

typedef enum {
    NRF_TX_PWR_M18dBm = 0x00,
    NRF_TX_PWR_M12dBm = 0x01,
    NRF_TX_PWR_M6dBm  = 0x02,
    NRF_TX_PWR_0dBm   = 0x03,
} NRF_TX_PWR;

typedef enum {
    NRF_ADDR_WIDTH_3 = 0x01,
    NRF_ADDR_WIDTH_4 = 0x02,
    NRF_ADDR_WIDTH_5 = 0x03,
} NRF_ADDR_WIDTH;

typedef enum {
    NRF_PIPE_P0 = 0x00,
    NRF_PIPE_P1 = 0x01,
    NRF_PIPE_P2 = 0x02,
    NRF_PIPE_P3 = 0x03,
    NRF_PIPE_P4 = 0x04,
    NRF_PIPE_P5 = 0x05,
} NRF_PIPE_NO;

typedef enum {
    NRF_P0_FLAG = 0x01,
    NRF_P1_FLAG = 0x02,
    NRF_P2_FLAG = 0x04,
    NRF_P3_FLAG = 0x08,
    NRF_P4_FLAG = 0x10,
    NRF_P5_FLAG = 0x20,
} NRF_PIPE_FLAG;

typedef enum {
    NRF_PIN_RESET = 0x00,
    NRF_PIN_SET = 0x01,
} NRF_PIN_STATE;

typedef enum {
    NRF_CRC_WIDTH_1B = 0,
    NRF_CRC_WIDTH_2B = 1
} NRF_CRC_WIDTH;

typedef enum {
    NRF_STATE_RX = 1,
    NRF_STATE_TX = 0
} NRF_TXRX_STATE;

enum NRF_RESULT {
    NRF_OK,
    NRF_ERR,
    NRF_ERR_INVALID_ARGUMENT,
    NRF_ERR_INVALID_PIPE,
    NRF_ERR_INVALID_LONG_ADDRESS,
    NRF_ERR_INVALID_SHORT_ADDRESS,
    NRF_ERR_SPI,
    NRF_ERR_TX_FIFO_FULL,
    NRF_ERR_TX_MAX_RT,
    NRF_ERR_RX_FIFO_EMPTY,
    NRF_ERR_MSG_LENGTH,
};

typedef enum {
    NRF_TX_DS,
    NRF_TX_MAX_RT,
} NRF_TX_RESULT;

typedef struct
{
    bool enabled : 1;
    bool auto_ack : 1;
    bool dynamic_payload : 1;
    uint8_t payload_width : 6;
    uint64_t address : 40;

} nrf24l01_rx_pipe;

typedef struct
{
    bool tx_full : 1;              // RO
    uint8_t rx_p_no : 3;           // RO
    bool max_rt : 1;
    bool tx_ds : 1;
    bool rx_dr : 1;
    bool reserved_0 : 1;

} nrf24l01_status;

typedef struct
{
    NRF_RESULT (*tx)(nrf24l01*, NRF_TX_RESULT tx_result);
    NRF_RESULT (*rx)(nrf24l01*);

} nrf24l01_callbacks;

typedef struct
{
    void(*ce_set)(nrf24l01*);
    void(*ce_reset)(nrf24l01*);
    void(*csn_set)(nrf24l01*);
    void(*csn_reset)(nrf24l01*);
    NRF_RESULT(*spi_txrx)(nrf24l01*, uint8_t* tx, uint8_t* rx, uint8_t len);

} nrf24l01_io_calls;

typedef struct
{
    bool max_retries : 1;
    bool tx_data_sent : 1;
    bool rx_data_ready : 1;
} nrf24l01_irq;

typedef struct
{
    uint8_t        rf_channel;
    NRF_DATA_RATE  data_rate;
    NRF_TX_PWR     tx_power;
    NRF_ADDR_WIDTH addr_width;
    uint8_t        retransmit_count;
    uint8_t        retransmit_delay;
    bool crc_en;
    NRF_CRC_WIDTH  crc_width;
    uint64_t tx_address;
    nrf24l01_rx_pipe rx_pipes[6];
    nrf24l01_irq irq;
    bool custom_irq_cb;
    nrf24l01_callbacks irq_cb;
    NRF_RESULT(*irq_isr)(nrf24l01*);

} nrf24l01_config;

typedef struct
{
    void* platform_conf;
    void(*platform_init)(nrf24l01*);

    nrf24l01_io_calls io;
    void(*rx_msg_cb)(nrf24l01*, NRF_PIPE_NO, uint8_t len);

} nrf24l01_init;

typedef struct
{
    uint8_t        rf_channel;
    NRF_DATA_RATE  data_rate;
    NRF_TX_PWR     tx_power;
    uint64_t tx_address;
    uint64_t rx_address;
} nrf24l01_esb_init;

#ifdef NRF_FREERTOS
typedef struct
{
    SemaphoreHandle_t spi_mutex;
    TaskHandle_t isr_handler_task;
    TaskHandle_t tx_task;
    TaskHandle_t rx_task;
    QueueHandle_t tx_queue;
    EventGroupHandle_t events;
} nrf24l01_freertos;

typedef struct {
    uint8_t* data;
    uint8_t length;
    uint8_t retries;
} nrf24l01_tx_packet;

#endif // NRF_FREERTOS

struct nrf24l01
{
    void* platform_conf;  // platform-specific config

    volatile uint8_t        tx_busy;
    volatile NRF_RESULT     tx_result;
    volatile uint8_t        rx_busy;
    volatile bool           irq_active;
    volatile NRF_TXRX_STATE state;

    nrf24l01_status status;
    uint8_t rx_msg[32];
    int rx_msg_len;

    // function pointers
    nrf24l01_io_calls io;
    nrf24l01_callbacks irq_cb;
    void(*rx_msg_cb)(nrf24l01*, NRF_PIPE_NO, uint8_t len);
    NRF_RESULT(*irq_isr)(nrf24l01*);

#ifdef NRF_FREERTOS
    nrf24l01_freertos freertos;
#endif // NRF_FREERTOS
};

/* Initialization routines */

/* Initialize bare nrf with platform hardware etc. for manual init setup
 *
 * */
void nrf_init(nrf24l01* nrf, nrf24l01_init* init);

/* Initialize nrf using config structure
 *
 * */
NRF_RESULT nrf_init_config(nrf24l01* nrf, nrf24l01_init* init, nrf24l01_config* config);

/* Initialize nrf with minimal config in enhanced shockburst mode with sane defaults
 *
 * */
NRF_RESULT nrf_init_enhanced_shockbust_default(nrf24l01* nrf, nrf24l01_init* init, nrf24l01_esb_init* esb_config);

/* Interrupt Service Routine (weak)
 *
 * You must call this function on Falling edge trigger detection interrupt
 * handler, typically, from HAL_GPIO_EXTI_Callback (stm32 HAL)
 *
 * */
void nrf_irq_isr(nrf24l01* nrf);

/* Interrupt Handler Routing
 *
 * This should be called outside of ISR context, e.g. by polling irq_active bit (set in ISR)
 *
 * This
 * */
NRF_RESULT nrf_irq_handler(nrf24l01* nrf);

/* Blocking Data Receiving
 *
 * Blocks until the data has arrived, then returns a pointer to received data.
 * Please note, once nrf_packet_received_callback routine is overridden, this
 * one will stop working. */
NRF_RESULT nrf_receive_packet(nrf24l01* nrf);

/* Blocking Data Sending
 *
 * If the AA is enabled (default), this method will return:
 *   NRF_OK - the data has been acknowledged by other party
 *   NRF_ERROR - the data has not been received (maximum retransmissions has
 * occurred) If the AA is disabled, returns NRF_OK once the data has been
 * transmitted (with no guarantee the data was actually received). */
NRF_RESULT nrf_send_packet(nrf24l01* nrf, uint8_t* data, uint8_t len);
NRF_RESULT nrf_send_packet_async(nrf24l01* nrf, uint8_t* data, uint8_t len);

/* Blocking Data Sending, with NO_ACK flag
 *
 * Disables the AA for this packet, thus this method always returns NRF_OK */
NRF_RESULT nrf_send_packet_noack(nrf24l01* nrf, uint8_t* data, uint8_t len);
NRF_RESULT nrf_send_packet_noack_async(nrf24l01* nrf, uint8_t* data, uint8_t len);

/* Non-Blocking Data Sending */
NRF_RESULT nrf_push_packet(nrf24l01* nrf, uint8_t* data, uint8_t len);

/* RF_SETUP */
NRF_RESULT nrf_set_data_rate(nrf24l01* nrf, NRF_DATA_RATE rate);
NRF_RESULT nrf_set_tx_power(nrf24l01* nrf, NRF_TX_PWR pwr);
NRF_RESULT nrf_set_ccw(nrf24l01* nrf, bool activate);

/* STATUS */
/* Note: Status is stored after every command so is possible to get status by doing a NOP and checking value,
 * or checking status after another command */
NRF_RESULT nrf_get_status(nrf24l01* nrf, nrf24l01_status* status);

NRF_RESULT nrf_clear_interrupts(nrf24l01* nrf);
NRF_RESULT nrf_clear_irq_max_rt(nrf24l01* nrf);
NRF_RESULT nrf_clear_irq_tx_ds(nrf24l01* nrf);
NRF_RESULT nrf_clear_irq_rx_dr(nrf24l01* nrf);

/* RF_CH */
NRF_RESULT nrf_set_rf_channel(nrf24l01* nrf, uint8_t ch);

/* SETUP_RETR setters*/
NRF_RESULT nrf_set_retransmission_count(nrf24l01* nrf, uint8_t count);
NRF_RESULT nrf_set_retransmission_delay(nrf24l01* nrf, uint8_t delay);

/* SETUP_RETR getters*/
NRF_RESULT nrf_get_retransmission_count(nrf24l01* nrf, uint8_t* count);
NRF_RESULT nrf_get_retransmission_delay(nrf24l01* nrf, uint8_t* delay);

/* SETUP_AW */
NRF_RESULT nrf_get_address_width(nrf24l01* nrf, NRF_ADDR_WIDTH* width);
NRF_RESULT nrf_set_address_width(nrf24l01* nrf, NRF_ADDR_WIDTH width);

/* EN_RXADDR */
NRF_RESULT nrf_enable_rx_pipes(nrf24l01* nrf, uint8_t pipe_mask);
NRF_RESULT nrf_enable_rx_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe);
NRF_RESULT nrf_disable_rx_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe);

/* EN_AA */
NRF_RESULT nrf_enable_rx_auto_ack_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe);
NRF_RESULT nrf_enable_rx_auto_ack(nrf24l01* nrf, uint8_t pipe_mask);

/* DYNPL */
NRF_RESULT nrf_enable_dyn_pl(nrf24l01* nrf, uint8_t pipe_mask);
NRF_RESULT nrf_enable_dyn_pl_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe);
NRF_RESULT nrf_disable_dyn_pl_pipe(nrf24l01* nrf, NRF_PIPE_NO pipe);

/* FEATURE setters */
NRF_RESULT nrf_enable_dyn_pl_feature(nrf24l01* nrf, bool enable);
NRF_RESULT nrf_enable_ack_pl_feature(nrf24l01* nrf, bool enable);
NRF_RESULT nrf_enable_dyn_ack_feature(nrf24l01* nrf, bool enable);

/* FEATURE getters */
NRF_RESULT nrf_get_dyn_pl_feature(nrf24l01* nrf, bool* enabled);
NRF_RESULT nrf_get_ack_pl_feature(nrf24l01* nrf, bool* enabled);
NRF_RESULT nrf_get_dyn_ack_feature(nrf24l01* nrf, bool* enabled);

/* CONFIG setters */
NRF_RESULT nrf_set_crc_en(nrf24l01* nrf, bool crc_en);
NRF_RESULT nrf_set_crc_width(nrf24l01* nrf, NRF_CRC_WIDTH width);
NRF_RESULT nrf_set_power_up(nrf24l01* nrf, bool power_up);
NRF_RESULT nrf_set_rx_tx(nrf24l01* nrf, NRF_TXRX_STATE rx);
NRF_RESULT nrf_enable_rx_data_ready_irq(nrf24l01* nrf, bool activate);
NRF_RESULT nrf_enable_tx_data_sent_irq(nrf24l01* nrf, bool activate);
NRF_RESULT nrf_enable_max_retransmit_irq(nrf24l01* nrf, bool activate);

/* CONFIG getters*/
NRF_RESULT nrf_get_crc_en(nrf24l01* nrf, bool* crc_en);
NRF_RESULT nrf_get_crc_width(nrf24l01* nrf, NRF_CRC_WIDTH* width);
NRF_RESULT nrf_get_power_up(nrf24l01* nrf, bool* power_up);
NRF_RESULT nrf_get_rx_tx(nrf24l01* nrf, NRF_TXRX_STATE* rx);

/* RX_ADDR_Px */
NRF_RESULT nrf_get_rx_address(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t* result);
NRF_RESULT nrf_set_rx_address(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t address);
// masked with address width
NRF_RESULT nrf_effective_rx_address(nrf24l01* nrf, NRF_PIPE_NO pipe, uint64_t* result);

/* TX_ADDR */
NRF_RESULT nrf_get_tx_address(nrf24l01* nrf, uint64_t* result);
NRF_RESULT nrf_set_tx_address(nrf24l01* nrf, uint64_t address);
// mask with address width
NRF_RESULT nrf_effective_tx_address(nrf24l01* nrf, uint64_t* result);

NRF_RESULT nrf_get_rx_payload_width(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t* width);
NRF_RESULT nrf_set_rx_payload_width(nrf24l01* nrf, NRF_PIPE_NO pipe, uint8_t width);

NRF_RESULT nrf_rx_message_available(nrf24l01* nrf, bool* available);

NRF_RESULT nrf_flush_rx_fifo(nrf24l01* nrf);
NRF_RESULT nrf_flush_tx_fifo(nrf24l01* nrf);
NRF_RESULT nrf_nop(nrf24l01* dev);  // stores status in status


NRF_RESULT nrf_rx_read_all(nrf24l01* nrf);

// I/O operations
void nrf_set_ce(nrf24l01* nrf);
void nrf_reset_ce(nrf24l01* nrf);

// set mode (clears and sets ce pin before/after, to be used on its own)
NRF_RESULT nrf_tx_mode(nrf24l01* nrf);
NRF_RESULT nrf_rx_mode(nrf24l01* nrf);
