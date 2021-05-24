#include <string.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include "kw1281.h"


// TODO: timeouts


static void kw1281_uart_configure (struct kw1281_state *state, uint32_t baudrate, uint8_t parity, uint8_t data_bits) {
    if (state->rx_enabled) {
        uart_rx_disable(state->cfg.uart_dev);
        state->rx_enabled = 0;
    }

    state->uart_cfg.baudrate = baudrate;
    state->uart_cfg.parity = parity;
    state->uart_cfg.data_bits = data_bits;

    uart_configure(state->cfg.uart_dev, &state->uart_cfg);
    uart_rx_enable(state->cfg.uart_dev, state->rx_buf, sizeof(state->rx_buf), SYS_FOREVER_MS);

    state->rx_enabled = 1;
}

static void kw1281_uart_tx (struct kw1281_state *state, uint8_t data, k_timeout_t delay) {
    state->tx_data = data;
    k_timer_start(&state->timer_wait_tx, delay, K_NO_WAIT);
    state->tx_data_valid = 1;
}

static uint8_t kw1281_update_step (struct kw1281_state *state) {
    enum kw1281_protocol_state new_state = state->protocol_state;
    uint8_t need_update = 0;
    uint8_t error = 0;


    if (state->disconnect_request) {
        state->disconnect_request = 0;

        if (state->rx_enabled) {
            uart_rx_disable(state->cfg.uart_dev);
            state->rx_enabled = 0;
        }

        state->protocol_state = KW1281_PROTOCOL_STATE_DISCONNECTED;
    }


    switch (state->protocol_state) {
        case KW1281_PROTOCOL_STATE_DISCONNECTED:
            if (state->connect_request) {
                state->connect_request = 0;

                state->tx_counter = 0;
                new_state = KW1281_PROTOCOL_STATE_CONNECT_ADDR;
                need_update = 1;
            }
            break;


        case KW1281_PROTOCOL_STATE_CONNECT_ADDR:
            if (state->tx_counter < 1) {
                kw1281_uart_configure(state, 5, UART_CFG_PARITY_ODD, UART_CFG_DATA_BITS_7);

                kw1281_uart_tx(state, state->address & 0x7F, K_NO_WAIT);
                state->tx_counter++;
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
                // should not happen as rx should be disabled
                // ignore

                break;


            case KW1281_PROTOCOL_STATE_CONNECT_ADDR:
                if (!state->tx_data_valid && state->rx_data == state->address) {
                    kw1281_uart_configure(state, 10400, UART_CFG_PARITY_ODD, UART_CFG_DATA_BITS_7);
                    new_state = KW1281_PROTOCOL_STATE_CONNECT_SYNC;
                } else {
                    state->error = KW1281_ERROR_COLLISION;
                    error = 1;
                }
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
                    if (state->rx_counter == 0) {
                        state->key_word = state->rx_data;
                    } else {
                        state->key_word |= state->rx_data<<8;
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

                        kw1281_uart_configure(state, 10400, UART_CFG_PARITY_NONE, UART_CFG_DATA_BITS_8);

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
        state->disconnect_request = 1;
        new_state = KW1281_PROTOCOL_STATE_DISCONNECTED;
        need_update = 1;
    }


    state->protocol_state = new_state;

    return need_update;
}

static void kw1281_update (struct kw1281_state *state) {
    while (kw1281_update_step(state)) {}
}

static void kw1281_uart_callback (const struct device *dev, struct uart_event *event, void *data) {
    struct kw1281_state *state = data;

    if (event->type == UART_RX_RDY) {
        for (int i=0; i<event->data.rx.len; i++) {
            state->rx_data = event->data.rx.buf[event->data.rx.offset+i];
            state->rx_data_valid = 1;

            kw1281_update(state);
        }
    }
}


static void kw1281_thread_tx (void *arg0, void *arg1, void *arg2) {
    struct kw1281_state *state = arg0;

    for (;;) {
        if (state->tx_data_valid) {
            k_timer_status_sync(&state->timer_wait_tx);
            uart_tx(state->cfg.uart_dev, &state->tx_data, 1, SYS_FOREVER_MS);
            state->tx_data_valid = 0;

            // TODO: here or in tx callback?
            kw1281_update(state);       // TODO: thread sync
        } else {
            k_msleep(1);        // TODO: tx_data_ready as condition variable
        }
    }
}


void kw1281_init (struct kw1281_state *state, struct kw1281_config *config) {
    memcpy(&state->cfg, config, sizeof(struct kw1281_config));

    uart_callback_set(state->cfg.uart_dev, kw1281_uart_callback, state);

    k_timer_init(&state->timer_wait_tx, NULL, NULL);

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

    k_thread_create(&state->tx_thread_data, state->tx_thread_stack, K_THREAD_STACK_SIZEOF(state->tx_thread_stack), kw1281_thread_tx, state, NULL, NULL, 5 /* TODO */, 0, K_NO_WAIT);
}

uint8_t kw1281_connect (struct kw1281_state *state, uint8_t addr) {
    if (!kw1281_is_disconnected(state)) {
        return 0;
    }

    state->counter = 1;

    state->address = addr;
    state->connect_request = 1;

    return 1;
}

uint8_t kw1281_send_block (struct kw1281_state *state, const struct kw1281_block *block) {
    if (kw1281_is_disconnected(state) || state->tx_block_valid || block->length < 3) {
        return 0;
    }

    state->tx_block = *block;
    state->tx_block.counter = state->counter++;
    state->tx_block_valid = 1;

    kw1281_update(state);

    return 1;
}

uint8_t kw1281_receive_block (struct kw1281_state *state, struct kw1281_block *block) {
    if (kw1281_is_disconnected(state) || !state->rx_block_valid) {
        return 0;
    }

    *block = state->rx_block;
    state->rx_block_valid = 0;

    return 1;
}

void kw1281_disconnect (struct kw1281_state *state) {
    state->disconnect_request = 1;
}

uint8_t kw1281_is_disconnected (struct kw1281_state *state) {
    return state->protocol_state == KW1281_PROTOCOL_STATE_DISCONNECTED;
}

enum kw1281_error kw1281_get_error (struct kw1281_state *state) {
    return state->error;
}

const char * kw1281_get_error_msg (struct kw1281_state *state) {
    switch (state->error) {
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

