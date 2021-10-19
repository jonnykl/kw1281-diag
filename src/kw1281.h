/*
 * Copyright 2021 Jonathan Klamroth
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef KW1281_H
#define KW1281_H


#include <stdint.h>
#include <zephyr.h>
#include <drivers/uart.h>


#define KW1281_MAX_BLOCK_LEN                    (255)
#define KW1281_MAX_BLOCK_DATA_LEN               (KW1281_MAX_BLOCK_LEN-3)
#define KW1281_BLOCK_END                        (0x03)
#define KW1281_KEY_WORD                         (1281)

#define KW1281_BYTE_COMPLEMENT(x)               (~(x) & 0xFF)


#define KW1281_TX_THREAD_STACK_SIZE             (512)


enum kw1281_error {
    KW1281_ERROR_UNKNOWN,
    KW1281_ERROR_NOT_CONNECTED,
    KW1281_ERROR_INVALID_ARGUMENT,
    KW1281_ERROR_CONNECT_TIMEOUT,
    KW1281_ERROR_RECEIVE_BLOCK_TIMEOUT,
    KW1281_ERROR_TRANSMIT_BLOCK_TIMEOUT,
};

enum kw1281_async_error_type {
    KW1281_ASYNC_ERROR_UNKNOWN,
    KW1281_ASYNC_ERROR_COLLISION,
    KW1281_ASYNC_ERROR_CONNECT_NO_SYNC,
    KW1281_ASYNC_ERROR_CONNECT_INVALID_KEY_WORD,
    KW1281_ASYNC_ERROR_UART_CONFIG,
    KW1281_ASYNC_ERROR_RECEIVE_BLOCK_INVALID_LENGTH,
    KW1281_ASYNC_ERROR_RECEIVE_BLOCK_INVALID_COUNTER,
    KW1281_ASYNC_ERROR_TRANSMIT_INVALID_ACK,
    KW1281_ASYNC_ERROR_OVERRUN,
};

enum kw1281_protocol_state {
    KW1281_PROTOCOL_STATE_DISCONNECTED,
    KW1281_PROTOCOL_STATE_DISCONNECT_REQUEST,
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
    KW1281_PROTOCOL_STATE_TX_END,
};


struct kw1281_state;
struct kw1281_async_error;

typedef void (*kw1281_async_error_callback_t)(const struct kw1281_state *state, struct kw1281_async_error *async_error);


struct kw1281_async_error {
    enum kw1281_async_error_type type;
};

struct kw1281_block {
    uint8_t length;
    uint8_t counter;
    uint8_t title;
    uint8_t data[KW1281_MAX_BLOCK_DATA_LEN];
};


struct kw1281_config {
    uint16_t default_baudrate;
    uint8_t inter_byte_time_ms;                 // time between two bytes on the k line
    uint16_t inter_block_time_ms;

    uint16_t key_word_ack_delay_ms;

    uint32_t timeout_connect_ms;
    uint32_t timeout_receive_block_ongoing_tx_ms;
    uint32_t timeout_receive_block_rx_start_ms;
    uint32_t timeout_receive_block_rx_ms;
    uint32_t timeout_transmit_block_ongoing_tx_ms;
    uint32_t timeout_transmit_block_tx_ms;

    const struct device *uart_dev;
    const struct device *slow_init_dev;
    int slow_init_pin;

    const struct device *status_led_dev;
    int status_led_pin;
    int status_led_active_low;
    uint32_t status_led_timeout_ms;

    kw1281_async_error_callback_t async_error_callback;
};

struct kw1281_state {
    // kernel objects
    struct k_mutex mutex;
    struct k_mutex dummy_mutex;
    struct k_condvar condvar_connect;
    struct k_condvar condvar_rx_block;
    struct k_condvar condvar_rx_block_started;
    struct k_condvar condvar_tx_block;
    struct k_work work_timer_slow_init_tx;
    struct k_work work_timer_tx;
    struct k_work work_timer_rx;
    struct k_work work_timer_status_led;
    struct k_timer timer_slow_init_tx;
    struct k_timer timer_tx;
    struct k_timer timer_rx;
    struct k_timer timer_status_led;

    // protocol data
    enum kw1281_protocol_state protocol_state;
    uint16_t key_word;
    uint8_t counter;

    // rx/tx data
    uint8_t rx_data;
    uint8_t tx_data;
    struct kw1281_block rx_block;
    struct kw1281_block tx_block;

    // internal state
    enum kw1281_error error;
    uint8_t key_word_parity_error;
    uint8_t rx_counter;
    uint8_t tx_counter;
    uint8_t address;
    uint8_t ack;
    uint8_t rx_data_valid;
    uint8_t tx_data_valid;
    uint8_t rx_block_valid;
    uint8_t rx_block_started;
    uint8_t tx_block_valid;
    uint8_t rx_enabled;
    uint8_t slow_init_started;

    // control
    uint8_t connect_request;
    uint8_t disconnect_request;

    // device configurations
    struct uart_config uart_cfg;

    // configuration
    struct kw1281_config cfg;
};


uint8_t kw1281_init(struct kw1281_state *state, const struct kw1281_config *config);
uint8_t kw1281_get_baudrate(struct kw1281_state *state, uint16_t *baudrate);
uint8_t kw1281_set_baudrate(struct kw1281_state *state, uint16_t baudrate);
uint8_t kw1281_connect(struct kw1281_state *state, uint8_t addr, uint8_t wait);
uint8_t kw1281_send_block(struct kw1281_state *state, const struct kw1281_block *block, uint8_t wait);
uint8_t kw1281_receive_block(struct kw1281_state *state, struct kw1281_block *block);
void kw1281_disconnect(struct kw1281_state *state);
uint8_t kw1281_is_disconnected(struct kw1281_state *state);
enum kw1281_error kw1281_get_error(struct kw1281_state *state);
const char * kw1281_get_error_msg(struct kw1281_state *state);
const char * kw1281_get_async_error_type_msg(enum kw1281_async_error_type async_error_type);


#endif /* KW1281_H */
