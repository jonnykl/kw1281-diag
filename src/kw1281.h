#ifndef KW1281_H
#define KW1281_H


#include <stdint.h>
#include <zephyr.h>
#include <drivers/uart.h>


#define KW1281_MAX_BLOCK_LEN                    (255)
#define KW1281_MAX_BLOCK_DATA_LEN               (KW1281_MAX_BLOCK_LEN-3)
#define KW1281_BLOCK_END                        (0x03)

#define KW1281_BYTE_COMPLEMENT(x)               (~(x) & 0xFF)


#define KW1281_TX_THREAD_STACK_SIZE             (512)


enum kw1281_error {
    KW1281_ERROR_UNKNOWN,
    KW1281_ERROR_COLLISION,
    KW1281_ERROR_CONNECT_NO_SYNC,
    KW1281_ERROR_UART_CONFIG,
    KW1281_ERROR_SEND_NO_ACK,
    KW1281_ERROR_RECEIVE_BYTE_UNKNOWN,
    KW1281_ERROR_RECEIVE_BYTE_TIMEOUT,
    KW1281_ERROR_RECEIVE_BLOCK_INVALID_LENGTH,
    KW1281_ERROR_RECEIVE_BLOCK_INVALID_COUNTER,
    KW1281_ERROR_RECEIVE_BLOCK_INVALID_END,
    KW1281_ERROR_TRANSMIT_INVALID_ACK,
    KW1281_ERROR_OVERRUN
};

enum kw1281_protocol_state {
    KW1281_PROTOCOL_STATE_DISCONNECTED,

    KW1281_PROTOCOL_STATE_CONNECT_ADDR,
    KW1281_PROTOCOL_STATE_CONNECT_SYNC,
    KW1281_PROTOCOL_STATE_CONNECT_KEY_WORD,

    KW1281_PROTOCOL_STATE_RX_LEN,
    KW1281_PROTOCOL_STATE_RX_COUNTER,
    KW1281_PROTOCOL_STATE_RX_TITLE,
    KW1281_PROTOCOL_STATE_RX_DATA,
    KW1281_PROTOCOL_STATE_RX_END,

    KW1281_PROTOCOL_STATE_TX_WAIT,
    KW1281_PROTOCOL_STATE_TX_LEN,
    KW1281_PROTOCOL_STATE_TX_COUNTER,
    KW1281_PROTOCOL_STATE_TX_TITLE,
    KW1281_PROTOCOL_STATE_TX_DATA,
    KW1281_PROTOCOL_STATE_TX_END
};


struct kw1281_block {
    uint8_t length;
    uint8_t counter;
    uint8_t title;
    uint8_t data[KW1281_MAX_BLOCK_DATA_LEN];
};


struct kw1281_config {
    uint8_t inter_byte_time_ms;                 // time between two bytes on the k line
    uint16_t inter_block_time_ms;

    uint16_t key_word_ack_delay_ms;

    uint32_t timeout_receive_byte_ms;
    uint32_t timeout_sync_ms;

    const struct device *uart_dev;
};

struct kw1281_state {
    uint16_t key_word;
    uint8_t counter;
    enum kw1281_error error;
    struct kw1281_block rx_block;
    struct kw1281_block tx_block;
    uint8_t rx_counter;
    uint8_t tx_counter;
    struct k_timer timer_wait_tx;
    uint8_t rx_data;
    uint8_t tx_data;
    uint8_t address;
    enum kw1281_protocol_state protocol_state;
    uint8_t ack;
    uint8_t rx_data_valid;
    uint8_t tx_data_valid;
    uint8_t rx_block_valid;
    uint8_t tx_block_valid;
    uint8_t rx_enabled;

    uint8_t connect_request;
    uint8_t disconnect_request;

    uint8_t rx_buf[2];       // TODO
    struct uart_config uart_cfg;

    struct k_thread tx_thread_data;
    K_THREAD_STACK_DEFINE(tx_thread_stack, KW1281_TX_THREAD_STACK_SIZE);

    struct kw1281_config cfg;
};


void kw1281_init(struct kw1281_state *state, struct kw1281_config *config);
uint8_t kw1281_connect(struct kw1281_state *state, uint8_t addr);
uint8_t kw1281_send_block(struct kw1281_state *state, const struct kw1281_block *block);
uint8_t kw1281_receive_block(struct kw1281_state *state, struct kw1281_block *block);
void kw1281_disconnect(struct kw1281_state *state);
uint8_t kw1281_is_disconnected(struct kw1281_state *state);
enum kw1281_error kw1281_get_error(struct kw1281_state *state);
const char * kw1281_get_error_msg(struct kw1281_state *state);


#endif /* KW1281_H */
