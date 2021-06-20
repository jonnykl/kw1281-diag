#include <string.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include "kw1281.h"


// TODO: timeouts
// TODO: error handing: uart_configure, k_timer_init


// state mutex must be locked when calling this function
static void kw1281_uart_tx (struct kw1281_state *state, uint8_t data, k_timeout_t delay) {
    state->tx_data = data;
    state->tx_data_valid = 1;
    k_timer_start(&state->timer_tx, delay, K_NO_WAIT);
}

// state mutex must be locked when calling this function
static void kw1281_uart_configure (struct kw1281_state *state, uint8_t data_bits, uint8_t parity) {
    uint8_t tmp;

    state->uart_cfg.data_bits = data_bits;
    state->uart_cfg.parity = parity;

    uart_configure(state->cfg.uart_dev, &state->uart_cfg);

    // clear uart rx buffer
    while (uart_poll_in(state->cfg.uart_dev, &tmp) == 0) {}
}


// this function must not be called directly, use kw1281_update instead
static uint8_t kw1281_update_step (struct kw1281_state *state) {
    enum kw1281_protocol_state new_state = state->protocol_state;
    uint8_t need_update = 0;
    uint8_t error = 0;


    if (state->disconnect_request) {
        state->disconnect_request = 0;

        if (state->rx_enabled) {
            k_timer_stop(&state->timer_rx);
            state->rx_enabled = 0;
        }

        state->protocol_state = KW1281_PROTOCOL_STATE_DISCONNECTED;
    }


    switch (state->protocol_state) {
        case KW1281_PROTOCOL_STATE_DISCONNECTED:
            if (state->connect_request) {
                state->connect_request = 0;

                state->slow_init_started = 0;
                state->tx_counter = 0;
                new_state = KW1281_PROTOCOL_STATE_CONNECT_ADDR;
                need_update = 1;
            }
            break;


        case KW1281_PROTOCOL_STATE_CONNECT_ADDR:
            if (!state->slow_init_started) {
                state->slow_init_started = 1;
                k_timer_start(&state->timer_slow_init, K_NO_WAIT, K_MSEC(200));
            } else if (state->tx_counter == 11) {
                // some MCUs do not support 7 data bits, so always use 8 data bits and handle the parity bit in this module
                kw1281_uart_configure(state, UART_CFG_DATA_BITS_8, UART_CFG_PARITY_NONE);

                k_timer_start(&state->timer_rx, K_NO_WAIT, K_MSEC(state->cfg.inter_byte_time_ms/5));
                state->rx_enabled = 1;

                new_state = KW1281_PROTOCOL_STATE_CONNECT_SYNC;
                need_update = 1;
            }
            break;


        case KW1281_PROTOCOL_STATE_TX_WAIT:
            if (state->tx_block_valid) {
                state->ack = 0;
                state->tx_counter = 0;
                new_state = KW1281_PROTOCOL_STATE_TX_LEN;

                need_update = 1;
            }
            break;


        case KW1281_PROTOCOL_STATE_TX_LEN:
            if (state->tx_counter < 1) {
                kw1281_uart_tx(state, state->tx_block.length, K_MSEC(state->cfg.inter_block_time_ms));
                state->tx_counter++;
            }
            break;

        case KW1281_PROTOCOL_STATE_TX_COUNTER:
            if (state->tx_counter < 1) {
                kw1281_uart_tx(state, state->tx_block.counter, K_MSEC(state->cfg.inter_byte_time_ms));
                state->tx_counter++;
            }
            break;

        case KW1281_PROTOCOL_STATE_TX_TITLE:
            if (state->tx_counter < 1) {
                kw1281_uart_tx(state, state->tx_block.title, K_MSEC(state->cfg.inter_byte_time_ms));
                state->tx_counter++;
            }
            break;

        case KW1281_PROTOCOL_STATE_TX_DATA:
            if (state->tx_counter < state->tx_block.length-3) {
                kw1281_uart_tx(state, state->tx_block.data[state->tx_counter], K_MSEC(state->cfg.inter_byte_time_ms));
                state->tx_counter++;
            }
            break;

        case KW1281_PROTOCOL_STATE_TX_END:
            if (state->tx_counter < 1) {
                kw1281_uart_tx(state, KW1281_BLOCK_END, K_MSEC(state->cfg.inter_byte_time_ms));
                state->tx_counter++;
            }
            break;


        default:
            // nothing to do here
            break;
    }


    if (state->rx_data_valid) {
        uint16_t ack_delay_ms;
        uint8_t send_ack = 0;

        switch (state->protocol_state) {
            case KW1281_PROTOCOL_STATE_DISCONNECTED:
            case KW1281_PROTOCOL_STATE_CONNECT_ADDR:
                // should not happen as rx should be disabled
                // ignore

                break;


            case KW1281_PROTOCOL_STATE_CONNECT_SYNC:
                if (state->rx_data == 0x55) {
                    state->ack = 0;
                    state->rx_counter = 0;
                    new_state = KW1281_PROTOCOL_STATE_CONNECT_KEY_WORD;
                } else {
                    state->error = KW1281_ERROR_CONNECT_NO_SYNC;
                    error = 1;
                }
                break;

            case KW1281_PROTOCOL_STATE_CONNECT_KEY_WORD:
                if (!state->ack) {
                    // TODO: check parity bit

                    if (state->rx_counter == 0) {
                        state->key_word = state->rx_data & 0x7F;
                    } else {
                        state->key_word |= (state->rx_data & 0x7F)<<7;
                    }

                    state->rx_counter++;
                    if (state->rx_counter == 2) {
                        state->ack = 1;

                        ack_delay_ms = state->cfg.key_word_ack_delay_ms;
                        send_ack = 1;
                    }
                } else {
                    if (!state->tx_data_valid && state->rx_data == state->tx_data) {
                        // TODO: verify key_word

                        state->ack = 0;
                        new_state = KW1281_PROTOCOL_STATE_RX_LEN;
                    } else {
                        state->error = KW1281_ERROR_COLLISION;
                        error = 1;
                    }
                }
                break;


            case KW1281_PROTOCOL_STATE_RX_LEN:
                if (!state->rx_block_valid) {
                    if (!state->ack) {
                        if (state->rx_data >= 3) {
                            state->rx_block.length = state->rx_data;

                            state->ack = 1;

                            ack_delay_ms = state->cfg.inter_byte_time_ms;
                            send_ack = 1;
                        } else {
                            state->error = KW1281_ERROR_RECEIVE_BLOCK_INVALID_LENGTH;
                            error = 1;
                        }
                    } else {
                        if (state->rx_data == state->tx_data) {
                            state->ack = 0;
                            new_state = KW1281_PROTOCOL_STATE_RX_COUNTER;
                        } else {
                            state->error = KW1281_ERROR_COLLISION;
                            error = 1;
                        }
                    }
                } else {
                    // TODO: buffer
                    state->error = KW1281_ERROR_OVERRUN;
                    error = 1;
                }
                break;

            case KW1281_PROTOCOL_STATE_RX_COUNTER:
                if (!state->ack) {
                    if (state->rx_data == state->counter) {
                        state->rx_block.counter = state->rx_data;

                        state->counter++;
                        state->ack = 1;

                        ack_delay_ms = state->cfg.inter_byte_time_ms;
                        send_ack = 1;
                    } else {
                        state->error = KW1281_ERROR_RECEIVE_BLOCK_INVALID_COUNTER;
                        error = 1;
                    }
                } else {
                    if (state->rx_data == state->tx_data) {
                        state->ack = 0;
                        new_state = KW1281_PROTOCOL_STATE_RX_TITLE;
                    } else {
                        state->error = KW1281_ERROR_COLLISION;
                        error = 1;
                    }
                }
                break;

            case KW1281_PROTOCOL_STATE_RX_TITLE:
                if (!state->ack) {
                    state->rx_block.title = state->rx_data;
                    state->ack = 1;

                    ack_delay_ms = state->cfg.inter_byte_time_ms;
                    send_ack = 1;
                } else {
                    if (state->rx_data == state->tx_data) {
                        if (state->rx_block.length > 3) {
                            state->rx_counter = 0;
                            new_state = KW1281_PROTOCOL_STATE_RX_TITLE;
                        } else {
                            new_state = KW1281_PROTOCOL_STATE_RX_END;
                        }
                    } else {
                        printk(">>> collision: %02x %02x\n", state->rx_data, state->tx_data);
                        state->error = KW1281_ERROR_COLLISION;
                        error = 1;
                    }
                }
                break;

            case KW1281_PROTOCOL_STATE_RX_DATA:
                if (!state->ack) {
                    state->rx_block.data[state->rx_counter] = state->rx_data;

                    state->rx_counter++;
                    state->ack = 1;

                    ack_delay_ms = state->cfg.inter_byte_time_ms;
                    send_ack = 1;
                } else {
                    if (state->rx_data == state->tx_data) {
                        if (state->rx_counter == state->rx_block.length-3) {
                            new_state = KW1281_PROTOCOL_STATE_RX_END;
                        } else {
                            state->ack = 0;
                        }
                    } else {
                        state->error = KW1281_ERROR_COLLISION;
                        error = 1;
                    }
                }
                break;

            case KW1281_PROTOCOL_STATE_RX_END:
                if (state->rx_data == KW1281_BLOCK_END) {
                    state->rx_block_valid = 1;

                    new_state = KW1281_PROTOCOL_STATE_TX_WAIT;
                    need_update = 1;
                } else {
                    state->error = KW1281_ERROR_COLLISION;
                    error = 1;
                }
                break;


            case KW1281_PROTOCOL_STATE_TX_WAIT:
                state->error = KW1281_ERROR_UNKNOWN;
                error = 1;
                break;


            case KW1281_PROTOCOL_STATE_TX_LEN:
            case KW1281_PROTOCOL_STATE_TX_COUNTER:
            case KW1281_PROTOCOL_STATE_TX_TITLE:
            case KW1281_PROTOCOL_STATE_TX_DATA:
                if (state->ack) {
                    if (state->rx_data == KW1281_BYTE_COMPLEMENT(state->tx_data)) {
                        switch (state->protocol_state) {
                            case KW1281_PROTOCOL_STATE_TX_LEN:
                                state->ack = 0;
                                state->tx_counter = 0;
                                new_state = KW1281_PROTOCOL_STATE_TX_COUNTER;
                                break;

                            case KW1281_PROTOCOL_STATE_TX_COUNTER:
                                state->ack = 0;
                                state->tx_counter = 0;
                                new_state = KW1281_PROTOCOL_STATE_TX_TITLE;
                                break;

                            case KW1281_PROTOCOL_STATE_TX_TITLE:
                                if (state->tx_block.length > 3) {
                                    state->rx_counter = 0;
                                    state->tx_counter = 0;
                                    state->ack = 0;
                                    new_state = KW1281_PROTOCOL_STATE_TX_DATA;
                                } else {
                                    new_state = KW1281_PROTOCOL_STATE_TX_END;
                                }
                                break;

                            case KW1281_PROTOCOL_STATE_TX_DATA:
                                state->rx_counter++;
                                if (state->rx_counter == state->tx_block.length-3) {
                                    state->tx_counter = 0;
                                    new_state = KW1281_PROTOCOL_STATE_TX_END;
                                }
                                break;

                            default:
                                // should never happen
                                // ignore
                                break;
                        }
                    } else {
                        state->error = KW1281_ERROR_TRANSMIT_INVALID_ACK;
                        error = 1;
                    }
                } else {
                    if (!state->tx_data_valid && state->rx_data == state->tx_data) {
                        state->ack = 1;
                    } else {
                        state->error = KW1281_ERROR_COLLISION;
                        error = 1;
                    }
                }
                break;

            case KW1281_PROTOCOL_STATE_TX_END:
                if (!state->tx_data_valid && state->rx_data == KW1281_BLOCK_END) {
                    state->tx_block_valid = 0;

                    state->ack = 0;
                    new_state = KW1281_PROTOCOL_STATE_RX_LEN;
                } else {
                    state->error = KW1281_ERROR_RECEIVE_BLOCK_INVALID_END;
                    error = 1;
                }
                break;
        }

        if (send_ack) {
            kw1281_uart_tx(state, KW1281_BYTE_COMPLEMENT(state->rx_data), K_MSEC(ack_delay_ms));
        }

        state->rx_data_valid = 0;
    }


    if (error) {
        printk("> error: %d\n", state->error);
        state->disconnect_request = 1;
        new_state = KW1281_PROTOCOL_STATE_DISCONNECTED;
        need_update = 1;
    }


    printk("> update state: state=%d, new_state=%d\n", state->protocol_state, new_state);
    state->protocol_state = new_state;

    return need_update;
}

// state mutex must be locked when calling this function
static void kw1281_update (struct kw1281_state *state) {
    while (kw1281_update_step(state)) {}
}


static void kw1281_tx (struct k_timer *timer) {
    struct kw1281_state *state = k_timer_user_data_get(timer);
    k_mutex_lock(&state->mutex, K_FOREVER);

    uart_poll_out(state->cfg.uart_dev, state->tx_data);
    state->tx_data_valid = 0;
    kw1281_update(state);

    k_mutex_unlock(&state->mutex);
}

static void kw1281_rx (struct k_timer *timer) {
    struct kw1281_state *state = k_timer_user_data_get(timer);
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (!state->rx_data_valid) {
        if (uart_poll_in(state->cfg.uart_dev, &state->rx_data) == 0) {
            state->rx_data_valid = 1;
            kw1281_update(state);
        }
    } else {
        state->error = KW1281_ERROR_OVERRUN;
        state->disconnect_request = 1;
        kw1281_update(state);
    }

    k_mutex_unlock(&state->mutex);
}

static void kw1281_slow_init_tx (struct k_timer *timer) {
    struct kw1281_state *state = k_timer_user_data_get(timer);
    k_mutex_lock(&state->mutex, K_FOREVER);

    uint8_t bit = state->tx_counter;

    if (bit == 0) {
        gpio_pin_set(state->cfg.slow_init_dev, state->cfg.slow_init_pin, 0);
    } else if (bit == 9) {
        gpio_pin_set(state->cfg.slow_init_dev, state->cfg.slow_init_pin, 1);
    } else if (bit == 10) {
        k_timer_stop(timer);
    } else {
        uint8_t data_bit = (state->address >> (bit-1)) & 1;
        gpio_pin_set(state->cfg.slow_init_dev, state->cfg.slow_init_pin, data_bit);

        // calculate parity
        state->address ^= data_bit<<7;
    }

    state->tx_counter++;
    kw1281_update(state);

    k_mutex_unlock(&state->mutex);
}


void kw1281_init (struct kw1281_state *state, const struct kw1281_config *config) {
    memcpy(&state->cfg, config, sizeof(struct kw1281_config));

    k_mutex_init(&state->mutex);

    state->uart_cfg.baudrate = state->cfg.baudrate;
    state->uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    state->uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

    state->protocol_state = KW1281_PROTOCOL_STATE_DISCONNECTED;
    state->rx_data_valid = 0;
    state->tx_data_valid = 0;
    state->rx_block_valid = 0;
    state->tx_block_valid = 0;
    state->connect_request = 0;
    state->disconnect_request = 0;
    state->rx_enabled = 0;

    gpio_pin_configure(state->cfg.slow_init_dev, state->cfg.slow_init_pin, GPIO_OUTPUT_HIGH);

    k_timer_init(&state->timer_slow_init, kw1281_slow_init_tx, NULL);
    k_timer_user_data_set(&state->timer_slow_init, state);

    k_timer_init(&state->timer_tx, kw1281_tx, NULL);
    k_timer_user_data_set(&state->timer_tx, state);

    k_timer_init(&state->timer_rx, kw1281_rx, NULL);
    k_timer_user_data_set(&state->timer_rx, state);
}

uint8_t kw1281_connect (struct kw1281_state *state, uint8_t addr) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->protocol_state != KW1281_PROTOCOL_STATE_DISCONNECTED) {
        return 0;
    }

    state->counter = 1;

    state->address = addr | 0x80;       // prepare for (odd) parity calculation
    state->connect_request = 1;

    kw1281_update(state);

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_send_block (struct kw1281_state *state, const struct kw1281_block *block) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->protocol_state == KW1281_PROTOCOL_STATE_DISCONNECTED || state->tx_block_valid || block->length < 3) {
        return 0;
    }

    state->tx_block = *block;
    state->tx_block.counter = state->counter++;
    state->tx_block_valid = 1;

    kw1281_update(state);

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_receive_block (struct kw1281_state *state, struct kw1281_block *block) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->protocol_state == KW1281_PROTOCOL_STATE_DISCONNECTED || !state->rx_block_valid) {
        return 0;
    }

    *block = state->rx_block;
    state->rx_block_valid = 0;

    k_mutex_unlock(&state->mutex);
    return 1;
}

void kw1281_disconnect (struct kw1281_state *state) {
    k_mutex_lock(&state->mutex, K_FOREVER);
    state->disconnect_request = 1;
    k_mutex_unlock(&state->mutex);
}

uint8_t kw1281_is_disconnected (struct kw1281_state *state) {
    uint8_t is_disconnected;

    k_mutex_lock(&state->mutex, K_FOREVER);
    is_disconnected = state->protocol_state == KW1281_PROTOCOL_STATE_DISCONNECTED;
    k_mutex_unlock(&state->mutex);

    return is_disconnected;
}

enum kw1281_error kw1281_get_error (struct kw1281_state *state) {
    enum kw1281_error error;

    k_mutex_lock(&state->mutex, K_FOREVER);
    error = state->error;
    k_mutex_unlock(&state->mutex);

    return error;
}

const char * kw1281_get_error_msg (struct kw1281_state *state) {
    switch (kw1281_get_error(state)) {
        case KW1281_ERROR_COLLISION:
            return "received data did not match sent data";

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

        case KW1281_ERROR_TRANSMIT_INVALID_ACK:
            return "invalid ack received";

        case KW1281_ERROR_OVERRUN:
            return "receive buffer overrun";


        case KW1281_ERROR_UNKNOWN:
        default:
            return "unknown";
    }
}

