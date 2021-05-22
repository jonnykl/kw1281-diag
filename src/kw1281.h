#ifndef KW1281_H
#define KW1281_H


#include <stdint.h>
#include <zephyr.h>
#include "soft_uart.h"


#define KW1281_MAX_BLOCK_LEN                    (255)
#define KW1281_MAX_BLOCK_DATA_LEN               (KW1281_MAX_BLOCK_LEN-3)
#define KW1281_BLOCK_END                        (0x03)

#define KW1281_BYTE_COMPLEMENT(x)               (~(x) & 0xFF)


enum kw1281_send_byte_wait {
    KW1281_SEND_BYTE_WAIT_TX,
    KW1281_SEND_BYTE_WAIT_ACK
};

enum kw1281_auto_counter {
    KW1281_AUTO_COUNTER,
    KW1281_MANUAL_COUNTER
};

enum kw1281_receive_byte_ack {
    KW1281_RECEIVE_BYTE_NO_ACK,
    KW1281_RECEIVE_BYTE_ACK
};

enum kw1281_error {
    KW1281_ERROR_UNKNOWN,
    KW1281_ERROR_CONNECT_NO_SYNC,
    KW1281_ERROR_UART_CONFIG,
    KW1281_ERROR_SEND_NO_ACK,
    KW1281_ERROR_RECEIVE_BYTE_UNKNOWN,
    KW1281_ERROR_RECEIVE_BYTE_TIMEOUT,
    KW1281_ERROR_RECEIVE_BLOCK_INVALID_LENGTH,
    KW1281_ERROR_RECEIVE_BLOCK_INVALID_COUNTER,
    KW1281_ERROR_RECEIVE_BLOCK_INVALID_END
};


struct kw1281_config {
    uint8_t inter_byte_time_ms;                 // time between two bytes on the k line
    uint16_t inter_block_time_ms;

    uint16_t key_word_ack_delay_ms;

    uint32_t timeout_receive_byte_ms;
    uint32_t timeout_sync_ms;

    struct soft_uart_config uart_cfg;
};

struct kw1281_state {
    uint8_t connected;
    uint16_t key_word;
    uint8_t counter;
    uint16_t uart_baudrate;
    enum kw1281_error error;

    struct k_timer timer_wait_byte;
    struct k_timer timer_wait_block;

    struct kw1281_config cfg;
};

struct kw1281_block {
    uint8_t length;
    uint8_t counter;
    uint8_t title;
    uint8_t data[KW1281_MAX_BLOCK_DATA_LEN];
};


void kw1281_init(struct kw1281_state *state, struct kw1281_config *config);
uint8_t kw1281_connect(struct kw1281_state *state, uint8_t addr);
uint8_t kw1281_send_byte(struct kw1281_state *state, uint8_t data, enum kw1281_send_byte_wait wait);
uint8_t kw1281_send_bytes(struct kw1281_state *state, const uint8_t *data, uint16_t len, enum kw1281_send_byte_wait wait);
uint8_t kw1281_send_block(struct kw1281_state *state, const struct kw1281_block *block, enum kw1281_auto_counter auto_counter);
uint8_t kw1281_receive_byte(struct kw1281_state *state, uint8_t *data, enum kw1281_receive_byte_ack ack);
uint8_t kw1281_receive_block(struct kw1281_state *state, struct kw1281_block *block, enum kw1281_auto_counter auto_counter);
void kw1281_error_reset(struct kw1281_state *state, enum kw1281_error erro);
void kw1281_disconnect(struct kw1281_state *state);
uint8_t kw1281_is_connected(struct kw1281_state *state);
enum kw1281_error kw1281_get_error(struct kw1281_state *state);
const char * kw1281_get_error_msg(struct kw1281_state *state);
uint8_t kw1281_get_counter(struct kw1281_state *state);
void kw1281_set_counter(struct kw1281_state *state, uint8_t counter);
uint16_t kw1281_get_key_word(struct kw1281_state *state);


#endif /* KW1281_H */
