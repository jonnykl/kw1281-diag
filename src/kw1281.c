#include <string.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include "kw1281.h"
#include "soft_uart.h"


static const uint32_t well_known_baudrates[] = {1200, 2400, 4800, 9600, 10400};


void kw1281_init (struct kw1281_state *state, struct kw1281_config *config) {
    memcpy(&state->cfg, config, sizeof(struct kw1281_config));

    soft_uart_init(&state->cfg.uart_cfg);

    k_timer_init(&state->timer_wait_byte, NULL, NULL);
    k_timer_init(&state->timer_wait_block, NULL, NULL);

    state->connected = 0;
}

uint8_t kw1281_connect (struct kw1281_state *state, uint8_t addr) {
    // 5 baud init

    state->cfg.uart_cfg.baudrate = 5;
    state->cfg.uart_cfg.parity = SOFT_UART_PARITY_ODD;
    state->cfg.uart_cfg.data_bits = SOFT_UART_DATA_BITS_7;

    soft_uart_tx(&state->cfg.uart_cfg, addr);


    // sync baudrate

    int sync_status = soft_uart_sync(&state->cfg.uart_cfg, state->cfg.timeout_sync_ms);
    if (sync_status != SOFT_UART_OK) {
        return 0;
    }

    uint32_t baudrate = state->cfg.uart_cfg.baudrate;
    for (int i=0; i<ARRAY_SIZE(well_known_baudrates); i++) {
        uint32_t well_known_baudrate = well_known_baudrates[i];

        // detected baudrate within a +/- 5% range of well known baudrates?
        if (baudrate > well_known_baudrate/20*19 && baudrate < well_known_baudrate/20*21) {
            state->cfg.uart_cfg.baudrate = well_known_baudrate;
            break;
        }
    }


    // receive key word

    uint8_t key_word_lsb, key_word_msb;

    if (!kw1281_receive_byte(state, &key_word_lsb, KW1281_RECEIVE_BYTE_NO_ACK)) {
        return 0;
    }

    if (!kw1281_receive_byte(state, &key_word_msb, KW1281_RECEIVE_BYTE_NO_ACK)) {
        return 0;
    }

    k_msleep(state->cfg.key_word_ack_delay_ms);
    kw1281_send_byte(state, KW1281_BYTE_COMPLEMENT(key_word_msb) & 0x7F, KW1281_SEND_BYTE_WAIT_TX);

    state->key_word = ((uint16_t) key_word_msb)<<8 | key_word_lsb;


    // reconfigure uart

    state->cfg.uart_cfg.parity = SOFT_UART_PARITY_NONE;
    state->cfg.uart_cfg.data_bits = SOFT_UART_DATA_BITS_8;


    // reset timers

    k_timer_start(&state->timer_wait_byte, K_NO_WAIT, K_NO_WAIT);
    k_timer_start(&state->timer_wait_block, K_NO_WAIT, K_NO_WAIT);


    state->counter = 1;
    state->connected = 1;


    return 1;
}

uint8_t kw1281_send_byte (struct kw1281_state *state, uint8_t data, enum kw1281_send_byte_wait wait) {
    k_timer_status_sync(&state->timer_wait_byte);
    soft_uart_tx(&state->cfg.uart_cfg, data);
    k_timer_start(&state->timer_wait_byte, K_MSEC(state->cfg.inter_byte_time_ms), K_NO_WAIT);

    if (wait == KW1281_SEND_BYTE_WAIT_ACK) {
        uint8_t received_byte;
        if (!kw1281_receive_byte(state, &received_byte, KW1281_RECEIVE_BYTE_NO_ACK)) {
            return 0;
        }

        if (received_byte != KW1281_BYTE_COMPLEMENT(data)) {
            kw1281_error_reset(state, KW1281_ERROR_SEND_NO_ACK);
            return 0;
        }
    }

    return 1;
}

uint8_t kw1281_send_bytes (struct kw1281_state *state, const uint8_t *data, uint16_t len, enum kw1281_send_byte_wait wait) {
    for (; len>0; len--) {
        if (!kw1281_send_byte(state, *data, wait)) {
            return 0;
        }

        data++;
    }

    return 1;
}

uint8_t kw1281_send_block (struct kw1281_state *state, const struct kw1281_block *block, enum kw1281_auto_counter auto_counter) {
    if (!state->connected || block->length < 3) {
        return 0;
    }


    uint8_t data[KW1281_MAX_BLOCK_LEN];
    data[0] = block->length;
    data[1] = (auto_counter == KW1281_AUTO_COUNTER) ? state->counter++ : block->counter;
    data[2] = block->title;
    memcpy(data+3, block->data, block->length-3);


    k_timer_status_sync(&state->timer_wait_block);


    if (!kw1281_send_bytes(state, data, block->length, KW1281_SEND_BYTE_WAIT_ACK)) {
        return 0;
    }

    if (!kw1281_send_byte(state, KW1281_BLOCK_END, KW1281_SEND_BYTE_WAIT_TX)) {
        return 0;
    }


    k_timer_start(&state->timer_wait_block, K_MSEC(state->cfg.inter_block_time_ms), K_NO_WAIT);


    return 1;
}

uint8_t kw1281_receive_byte (struct kw1281_state *state, uint8_t *data, enum kw1281_receive_byte_ack ack) {
    int ret = soft_uart_rx(&state->cfg.uart_cfg, data, state->cfg.timeout_receive_byte_ms);
    if (ret == SOFT_UART_OK) {
        k_timer_start(&state->timer_wait_byte, K_MSEC(state->cfg.inter_byte_time_ms), K_NO_WAIT);

        if (ack == KW1281_RECEIVE_BYTE_ACK) {
            kw1281_send_byte(state, KW1281_BYTE_COMPLEMENT(*data), KW1281_SEND_BYTE_WAIT_TX);
        }

        return 1;
    } else if (ret == SOFT_UART_ERR_TIMEOUT) {
        kw1281_error_reset(state, KW1281_ERROR_RECEIVE_BYTE_TIMEOUT);
    } else {
        kw1281_error_reset(state, KW1281_ERROR_RECEIVE_BYTE_UNKNOWN);
    }

    return 0;
}

uint8_t kw1281_receive_block (struct kw1281_state *state, struct kw1281_block *block, enum kw1281_auto_counter auto_counter) {
    if (!state->connected) {
        return 0;
    }


    if (!kw1281_receive_byte(state, &block->length, KW1281_RECEIVE_BYTE_ACK)) {
        return 0;
    }

    if (block->length < 3) {
        kw1281_error_reset(state, KW1281_ERROR_RECEIVE_BLOCK_INVALID_LENGTH);
        return 0;
    }


    if (!kw1281_receive_byte(state, &block->counter, KW1281_RECEIVE_BYTE_ACK)) {
        return 0;
    }

    if (auto_counter == KW1281_AUTO_COUNTER) {
        if (block->counter != state->counter) {
            kw1281_error_reset(state, KW1281_ERROR_RECEIVE_BLOCK_INVALID_COUNTER);
            return 0;
        }

        state->counter++;
    }


    if (!kw1281_receive_byte(state, &block->title, KW1281_RECEIVE_BYTE_ACK)) {
        return 0;
    }


    for (uint8_t i=0; i<(block->length-3); i++) {
        if (!kw1281_receive_byte(state, block->data + i, KW1281_RECEIVE_BYTE_ACK)) {
            return 0;
        }
    }


    uint8_t end_byte;
    if (!kw1281_receive_byte(state, &end_byte, KW1281_RECEIVE_BYTE_NO_ACK)) {
        return 0;
    }

    if (end_byte != KW1281_BLOCK_END) {
        kw1281_error_reset(state, KW1281_ERROR_RECEIVE_BLOCK_INVALID_END);
        return 0;
    }


    k_timer_start(&state->timer_wait_block, K_MSEC(state->cfg.inter_block_time_ms), K_NO_WAIT);


    return 1;
}

void kw1281_error_reset (struct kw1281_state *state, enum kw1281_error error) {
    kw1281_disconnect(state);
    state->error = error;
}

void kw1281_disconnect (struct kw1281_state *state) {
    state->connected = 0;
}

uint8_t kw1281_is_connected (struct kw1281_state *state) {
    return state->connected;
}

enum kw1281_error kw1281_get_error (struct kw1281_state *state) {
    return state->error;
}

const char * kw1281_get_error_msg (struct kw1281_state *state) {
    switch (state->error) {
        case KW1281_ERROR_CONNECT_NO_SYNC:
            return "cannot not sync baudrate";

        case KW1281_ERROR_UART_CONFIG:
            return "wrong uart configuration";

        case KW1281_ERROR_SEND_NO_ACK:
            return "no or invalid ack received";

        case KW1281_ERROR_RECEIVE_BYTE_UNKNOWN:
            return "unknown error while receiving a byte";

        case KW1281_ERROR_RECEIVE_BYTE_TIMEOUT:
            return "timeout while receiving a byte";

        case KW1281_ERROR_RECEIVE_BLOCK_INVALID_LENGTH:
            return "length of the received block is invalid";

        case KW1281_ERROR_RECEIVE_BLOCK_INVALID_COUNTER:
            return "counter of the received block is invalid";

        case KW1281_ERROR_RECEIVE_BLOCK_INVALID_END:
            return "end of the received block is invalid";


        case KW1281_ERROR_UNKNOWN:
        default:
            return "unknown";
    }
}

uint8_t kw1281_get_counter (struct kw1281_state *state) {
    return state->counter;
}

void kw1281_set_counter (struct kw1281_state *state, uint8_t counter) {
    state->counter = counter;
}

uint16_t kw1281_get_key_word (struct kw1281_state *state) {
    return state->key_word;
}

