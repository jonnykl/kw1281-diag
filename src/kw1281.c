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

#include <string.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include "kw1281.h"


static void kw1281_dispatch_async_error (struct kw1281_state *state, enum kw1281_async_error_type type) {
    if (state->cfg.async_error_callback != NULL) {
        struct kw1281_async_error async_error = {
            .type = type
        };

        state->cfg.async_error_callback(state, &async_error);
    }
}

// state mutex must be locked when calling this function
static void kw1281_uart_tx (struct kw1281_state *state, uint8_t data, k_timeout_t delay) {
    state->tx_data = data;
    state->tx_data_valid = 1;
    k_timer_start(&state->timer_tx, delay, K_NO_WAIT);
}

// state mutex must be locked when calling this function
static uint8_t kw1281_uart_configure (struct kw1281_state *state) {
    uint8_t tmp;

    state->uart_cfg.baudrate = state->cfg.baudrate;

    if (uart_configure(state->cfg.uart_dev, &state->uart_cfg) < 0) {
        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_UART_CONFIG);
        return 0;
    }

    // clear uart rx buffer
    while (uart_poll_in(state->cfg.uart_dev, &tmp) == 0) {}

    return 1;
}


// - this function must not be called directly, use kw1281_update instead
// - state->protocol_state may only be updated in this function
static uint8_t kw1281_update_step (struct kw1281_state *state) {
    enum kw1281_protocol_state new_state = state->protocol_state;
    uint8_t need_update = 0;
    uint8_t error = 0;


    if (state->disconnect_request || state->protocol_state == KW1281_PROTOCOL_STATE_DISCONNECT_REQUEST) {
        state->disconnect_request = 0;

        if (state->rx_enabled) {
            k_timer_stop(&state->timer_rx);
            state->rx_enabled = 0;
        }

        if (state->rx_auto_baud_enabled) {
            // ignore errors / return code
            gpio_pin_interrupt_configure(state->cfg.uart_rx_dev, state->cfg.uart_rx_pin, GPIO_INT_DISABLE);
            state->rx_auto_baud_enabled = 0;
        }

        k_mutex_lock(&state->dummy_mutex, K_NO_WAIT);
        k_condvar_wait(&state->condvar_connect, &state->dummy_mutex, K_NO_WAIT);
        k_condvar_wait(&state->condvar_rx_block, &state->dummy_mutex, K_NO_WAIT);
        k_condvar_wait(&state->condvar_rx_block_started, &state->dummy_mutex, K_NO_WAIT);
        k_condvar_wait(&state->condvar_tx_block, &state->dummy_mutex, K_NO_WAIT);
        k_mutex_unlock(&state->dummy_mutex);

        state->key_word_parity_error = 0;
        state->rx_data_valid = 0;
        state->tx_data_valid = 0;
        state->rx_block_valid = 0;
        state->rx_block_started = 0;
        state->tx_block_valid = 0;

        k_timer_stop(&state->timer_slow_init_tx);
        k_timer_stop(&state->timer_tx);

        new_state = KW1281_PROTOCOL_STATE_DISCONNECTED;
        state->protocol_state = new_state;
    }


    switch (state->protocol_state) {
        case KW1281_PROTOCOL_STATE_DISCONNECTED:
            if (state->connect_request) {
                state->connect_request = 0;

                if (kw1281_uart_configure(state)) {
                    state->slow_init_started = 0;
                    state->tx_counter = 0;
                    new_state = KW1281_PROTOCOL_STATE_CONNECT_ADDR;
                } else {
                    new_state = KW1281_PROTOCOL_STATE_DISCONNECT_REQUEST;
                }

                need_update = 1;
            }
            break;


        case KW1281_PROTOCOL_STATE_CONNECT_ADDR:
            if (!state->slow_init_started) {
                state->slow_init_started = 1;
                k_timer_start(&state->timer_slow_init_tx, K_NO_WAIT, K_MSEC(200));
            } else if (state->tx_counter == 11) {
                if (!state->cfg.auto_baud) {
                    k_timer_start(&state->timer_rx, K_NO_WAIT, K_MSEC(state->cfg.inter_byte_time_ms/5));
                    state->rx_enabled = 1;
                } else {
                    state->rx_auto_baud_counter = 0;
                    state->rx_auto_baud_enabled = 1;

                    if (gpio_pin_interrupt_configure(state->cfg.uart_rx_dev, state->cfg.uart_rx_pin, GPIO_INT_EDGE_BOTH) < 0) {
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_UNKNOWN);
                        error = 1;
                    }
                }

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
            case KW1281_PROTOCOL_STATE_DISCONNECT_REQUEST:
            case KW1281_PROTOCOL_STATE_CONNECT_ADDR:
                // should not happen as rx should be disabled or disconnect is requested
                // ignore

                break;


            case KW1281_PROTOCOL_STATE_CONNECT_SYNC:
                if (state->rx_data == 0x55) {
                    state->key_word_parity_error = 0;
                    state->ack = 0;
                    state->rx_counter = 0;
                    new_state = KW1281_PROTOCOL_STATE_CONNECT_KEY_WORD;
                } else {
                    kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_CONNECT_NO_SYNC);
                    error = 1;
                }
                break;

            case KW1281_PROTOCOL_STATE_CONNECT_KEY_WORD:
                if (!state->ack) {
                    uint8_t parity = state->rx_data;
                    parity = parity | (parity>>4);
                    parity = parity | (parity>>2);
                    parity = parity | (parity>>1);

                    if ((parity & 1) != 1) {
                        state->key_word_parity_error = 1;
                    }

                    if (state->rx_counter == 0) {
                        state->key_word = state->rx_data & 0x7F;
                    } else {
                        state->key_word |= (state->rx_data & 0x7F)<<7;
                    }

                    state->rx_counter++;
                    if (state->rx_counter == 2) {
                        if (!state->key_word_parity_error) {
                            state->ack = 1;

                            ack_delay_ms = state->cfg.key_word_ack_delay_ms;
                            send_ack = 1;
                        } else {
                            new_state = KW1281_PROTOCOL_STATE_CONNECT_SYNC;
                        }
                    }
                } else {
                    if (!state->tx_data_valid && state->rx_data == state->tx_data) {
                        if (state->key_word == KW1281_KEY_WORD) {
                            state->ack = 0;
                            new_state = KW1281_PROTOCOL_STATE_RX_LEN;

                            k_condvar_signal(&state->condvar_connect);
                        } else {
                            kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_CONNECT_INVALID_KEY_WORD);
                            error = 1;
                        }
                    } else {
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
                        error = 1;
                    }
                }
                break;


            case KW1281_PROTOCOL_STATE_RX_LEN:
                if (!state->rx_block_valid) {
                    if (!state->ack) {
                        if (state->rx_data >= 3) {
                            state->rx_block_started = 1;
                            state->rx_block.length = state->rx_data;

                            state->ack = 1;

                            ack_delay_ms = state->cfg.inter_byte_time_ms;
                            send_ack = 1;

                            k_condvar_signal(&state->condvar_rx_block_started);
                        } else {
                            kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_RECEIVE_BLOCK_INVALID_LENGTH);
                            error = 1;
                        }
                    } else {
                        if (state->rx_data == state->tx_data) {
                            state->ack = 0;
                            new_state = KW1281_PROTOCOL_STATE_RX_COUNTER;
                        } else {
                            kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
                            error = 1;
                        }
                    }
                } else {
                    // should never happen if this libaries is used correctly as blocks
                    // are sent/received alternatingly -> before every transmission of
                    // a block the received block should be read

                    kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_OVERRUN);
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
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_RECEIVE_BLOCK_INVALID_COUNTER);
                        error = 1;
                    }
                } else {
                    if (state->rx_data == state->tx_data) {
                        state->ack = 0;
                        new_state = KW1281_PROTOCOL_STATE_RX_TITLE;
                    } else {
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
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
                            state->ack = 0;
                            state->rx_counter = 0;
                            new_state = KW1281_PROTOCOL_STATE_RX_DATA;
                        } else {
                            new_state = KW1281_PROTOCOL_STATE_RX_END;
                        }
                    } else {
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
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
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
                        error = 1;
                    }
                }
                break;

            case KW1281_PROTOCOL_STATE_RX_END:
                if (state->rx_data == KW1281_BLOCK_END) {
                    state->rx_block_valid = 1;

                    new_state = KW1281_PROTOCOL_STATE_TX_WAIT;
                    need_update = 1;

                    k_condvar_signal(&state->condvar_rx_block);
                } else {
                    kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
                    error = 1;
                }
                break;


            case KW1281_PROTOCOL_STATE_TX_WAIT:
                kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_UNKNOWN);
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
                                    state->tx_counter = 0;
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

                        need_update = 1;
                    } else {
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_TRANSMIT_INVALID_ACK);
                        error = 1;
                    }
                } else {
                    if (!state->tx_data_valid && state->rx_data == state->tx_data) {
                        state->ack = 1;
                    } else {
                        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
                        error = 1;
                    }
                }
                break;

            case KW1281_PROTOCOL_STATE_TX_END:
                if (!state->tx_data_valid && state->rx_data == KW1281_BLOCK_END) {
                    state->tx_block_valid = 0;

                    state->ack = 0;
                    new_state = KW1281_PROTOCOL_STATE_RX_LEN;

                    k_condvar_signal(&state->condvar_tx_block);
                } else {
                    kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_COLLISION);
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
        new_state = KW1281_PROTOCOL_STATE_DISCONNECT_REQUEST;
        need_update = 1;
    }


    //printk("> update state: state=%d, new_state=%d\n", state->protocol_state, new_state);
    state->protocol_state = new_state;

    return need_update;
}

// state mutex must be locked when calling this function
static void kw1281_update (struct kw1281_state *state) {
    while (kw1281_update_step(state)) {}
}


// state mutex must be locked when calling this function
static void kw1281_update_status_led (struct kw1281_state *state) {
    if (state->cfg.status_led_dev != NULL) {
        // ignore errors / return code
        gpio_pin_toggle(state->cfg.status_led_dev, state->cfg.status_led_pin);
        k_timer_start(&state->timer_status_led, K_MSEC(state->cfg.status_led_timeout_ms), K_NO_WAIT);
    }
}


static void kw1281_slow_init_tx (struct k_work *work) {
    struct kw1281_state *state = CONTAINER_OF(work, struct kw1281_state, work_timer_slow_init_tx);
    k_mutex_lock(&state->mutex, K_FOREVER);

    uint8_t bit = state->tx_counter;

    if (bit == 0) {
        gpio_pin_set(state->cfg.slow_init_dev, state->cfg.slow_init_pin, 0);
    } else if (bit == 9) {
        gpio_pin_set(state->cfg.slow_init_dev, state->cfg.slow_init_pin, 1);
    } else if (bit == 10) {
        k_timer_stop(&state->timer_slow_init_tx);
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

static void kw1281_tx (struct k_work *work) {
    struct kw1281_state *state = CONTAINER_OF(work, struct kw1281_state, work_timer_tx);
    k_mutex_lock(&state->mutex, K_FOREVER);

    uart_poll_out(state->cfg.uart_dev, state->tx_data);
    state->tx_data_valid = 0;
    kw1281_update(state);

    k_mutex_unlock(&state->mutex);
}

static void kw1281_rx (struct k_work *work) {
    struct kw1281_state *state = CONTAINER_OF(work, struct kw1281_state, work_timer_rx);
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->rx_enabled) {
        if (!state->rx_data_valid) {
            if (uart_poll_in(state->cfg.uart_dev, &state->rx_data) == 0) {
                state->rx_data_valid = 1;
                kw1281_update(state);
                kw1281_update_status_led(state);
            }
        } else {
            kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_OVERRUN);
            state->disconnect_request = 1;
            kw1281_update(state);
        }
    }

    k_mutex_unlock(&state->mutex);
}

static void kw1281_rx_auto_baud (struct k_work *work) {
    struct kw1281_state *state = CONTAINER_OF(work, struct kw1281_state, work_rx_auto_baud);
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (gpio_pin_interrupt_configure(state->cfg.uart_rx_dev, state->cfg.uart_rx_pin, GPIO_INT_DISABLE) < 0) {
        kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_UNKNOWN);
        state->disconnect_request = 1;
        kw1281_update(state);

        k_mutex_unlock(&state->mutex);
        return;
    }

    if (state->rx_auto_baud_enabled) {
        // 0x55 =  01010101
        // 11111 0 10101010 1 11111
        //       |          |
        // start (inc)      end (excl)
        // -> 9 bits

        uint32_t avg = 0;
        uint32_t delta;
        int32_t deviation;

        for (int i=0; i<9; i++) {
            delta = state->rx_auto_baud_timestamps[i+1] - state->rx_auto_baud_timestamps[i];
            state->rx_auto_baud_timestamps[i] = delta;

            avg += delta;
        }

        avg /= 9;

        for (int i=0; i<9; i++) {
            deviation = ((int32_t) state->rx_auto_baud_timestamps[i]) - ((int32_t) avg);
            deviation = deviation > 0 ? deviation : -deviation;

            // allow a deviation of 25% of the average
            if (deviation > avg/4) {
                kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_CONNECT_NO_SYNC);
                state->disconnect_request = 1;
                kw1281_update(state);

                k_mutex_unlock(&state->mutex);
                return;
            }
        }

        state->cfg.baudrate = ((sys_clock_hw_cycles_per_sec()*2 / avg) + 1)/2;

        if (!kw1281_uart_configure(state)) {
            kw1281_dispatch_async_error(state, KW1281_ASYNC_ERROR_UNKNOWN);
            state->disconnect_request = 1;
            kw1281_update(state);

            k_mutex_unlock(&state->mutex);
            return;
        }

        k_timer_start(&state->timer_rx, K_NO_WAIT, K_MSEC(state->cfg.inter_byte_time_ms/5));
        state->rx_enabled = 1;

        state->rx_data = 0x55;
        state->rx_data_valid = 1;

        kw1281_update(state);
        kw1281_update_status_led(state);
    }

    k_mutex_unlock(&state->mutex);
}

static void kw1281_status_led (struct k_work *work) {
    struct kw1281_state *state = CONTAINER_OF(work, struct kw1281_state, work_timer_status_led);
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->cfg.status_led_dev != NULL) {
        // ignore errors / return code
        gpio_pin_set(state->cfg.status_led_dev, state->cfg.status_led_pin, 0);
    }

    k_mutex_unlock(&state->mutex);
}


static void kw1281_timer_slow_init_tx_handler (struct k_timer *timer) {
    struct kw1281_state *state = k_timer_user_data_get(timer);
    k_work_submit(&state->work_timer_slow_init_tx);
}

static void kw1281_timer_tx_handler (struct k_timer *timer) {
    struct kw1281_state *state = k_timer_user_data_get(timer);
    k_work_submit(&state->work_timer_tx);
}

static void kw1281_timer_rx_handler (struct k_timer *timer) {
    struct kw1281_state *state = k_timer_user_data_get(timer);
    k_work_submit(&state->work_timer_rx);
}

static void kw1281_timer_status_led_handler (struct k_timer *timer) {
    struct kw1281_state *state = k_timer_user_data_get(timer);
    k_work_submit(&state->work_timer_status_led);
}


static void kw1281_uart_rx_callback (const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct kw1281_state *state = CONTAINER_OF(cb, struct kw1281_state, uart_rx_callback);

    uint32_t t = sys_clock_cycle_get_32();

    if (state->rx_auto_baud_counter == 10) {
        return;
    }

    state->rx_auto_baud_timestamps[state->rx_auto_baud_counter] = t;

    if (++state->rx_auto_baud_counter == 10) {
        k_work_submit(&state->work_rx_auto_baud);
    }
}


uint8_t kw1281_init (struct kw1281_state *state, const struct kw1281_config *config) {
    memcpy(&state->cfg, config, sizeof(struct kw1281_config));

    if (k_mutex_init(&state->mutex) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    if (k_mutex_init(&state->dummy_mutex) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    if (k_condvar_init(&state->condvar_connect) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    if (k_condvar_init(&state->condvar_rx_block) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    if (k_condvar_init(&state->condvar_rx_block_started) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    if (k_condvar_init(&state->condvar_tx_block) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    // some MCUs do not support 7 data bits, so always use 8 data bits and handle the parity bit in this module
    state->uart_cfg.baudrate = state->cfg.baudrate;
    state->uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
    state->uart_cfg.parity = UART_CFG_PARITY_NONE;
    state->uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    state->uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

    state->protocol_state = KW1281_PROTOCOL_STATE_DISCONNECTED;
    state->key_word_parity_error = 0;
    state->rx_data_valid = 0;
    state->tx_data_valid = 0;
    state->rx_block_valid = 0;
    state->rx_block_started = 0;
    state->tx_block_valid = 0;
    state->connect_request = 0;
    state->disconnect_request = 0;
    state->rx_enabled = 0;
    state->rx_auto_baud_enabled = 0;

    if (gpio_pin_configure(state->cfg.slow_init_dev, state->cfg.slow_init_pin, GPIO_OUTPUT_HIGH) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    if (gpio_pin_configure(state->cfg.uart_rx_dev, state->cfg.uart_rx_pin, GPIO_INPUT) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    gpio_init_callback(&state->uart_rx_callback, kw1281_uart_rx_callback, BIT(state->cfg.uart_rx_pin));

    if (gpio_add_callback(state->cfg.uart_rx_dev, &state->uart_rx_callback) < 0) {
        state->error = KW1281_ERROR_UNKNOWN;
        return 0;
    }

    if (state->cfg.status_led_dev != NULL) {
        gpio_flags_t flags = GPIO_OUTPUT_INACTIVE;

        if (state->cfg.status_led_active_low) {
            flags |= GPIO_ACTIVE_LOW;
        } else {
            flags |= GPIO_ACTIVE_HIGH;
        }

        if (gpio_pin_configure(state->cfg.status_led_dev, state->cfg.status_led_pin, flags) < 0) {
            state->error = KW1281_ERROR_UNKNOWN;
            return 0;
        }
    }

    k_work_init(&state->work_timer_slow_init_tx, kw1281_slow_init_tx);
    k_work_init(&state->work_timer_tx, kw1281_tx);
    k_work_init(&state->work_timer_rx, kw1281_rx);
    k_work_init(&state->work_timer_status_led, kw1281_status_led);
    k_work_init(&state->work_rx_auto_baud, kw1281_rx_auto_baud);

    k_timer_init(&state->timer_slow_init_tx, kw1281_timer_slow_init_tx_handler, NULL);
    k_timer_user_data_set(&state->timer_slow_init_tx, state);

    k_timer_init(&state->timer_tx, kw1281_timer_tx_handler, NULL);
    k_timer_user_data_set(&state->timer_tx, state);

    k_timer_init(&state->timer_rx, kw1281_timer_rx_handler, NULL);
    k_timer_user_data_set(&state->timer_rx, state);

    k_timer_init(&state->timer_status_led, kw1281_timer_status_led_handler, NULL);
    k_timer_user_data_set(&state->timer_status_led, state);

    return 1;
}

uint8_t kw1281_get_baudrate (struct kw1281_state *state, uint16_t *baudrate) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    *baudrate = state->cfg.baudrate;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_set_baudrate (struct kw1281_state *state, uint16_t baudrate) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    state->cfg.baudrate = baudrate;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_get_auto_baud (struct kw1281_state *state, uint8_t *auto_baud) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    *auto_baud = state->cfg.auto_baud;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_set_auto_baud (struct kw1281_state *state, uint8_t auto_baud) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    state->cfg.auto_baud = auto_baud;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_get_inter_byte_time_ms (struct kw1281_state *state, uint8_t *inter_byte_time_ms) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    *inter_byte_time_ms = state->cfg.inter_byte_time_ms;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_set_inter_byte_time_ms (struct kw1281_state *state, uint8_t inter_byte_time_ms) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    state->cfg.inter_byte_time_ms = inter_byte_time_ms;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_get_inter_block_time_ms (struct kw1281_state *state, uint16_t *inter_block_time_ms) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    *inter_block_time_ms = state->cfg.inter_block_time_ms;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_set_inter_block_time_ms (struct kw1281_state *state, uint16_t inter_block_time_ms) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    state->cfg.inter_block_time_ms = inter_block_time_ms;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_get_key_word_ack_delay_ms (struct kw1281_state *state, uint16_t *key_word_ack_delay_ms) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    *key_word_ack_delay_ms = state->cfg.key_word_ack_delay_ms;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_set_key_word_ack_delay_ms (struct kw1281_state *state, uint16_t key_word_ack_delay_ms) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    state->cfg.key_word_ack_delay_ms = key_word_ack_delay_ms;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_get_timeout_multiplier (struct kw1281_state *state, uint16_t *timeout_multiplier) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    *timeout_multiplier = state->cfg.timeout_multiplier;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_set_timeout_multiplier (struct kw1281_state *state, uint16_t timeout_multiplier) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    state->cfg.timeout_multiplier = timeout_multiplier;

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_connect (struct kw1281_state *state, uint8_t addr, uint8_t wait) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->protocol_state != KW1281_PROTOCOL_STATE_DISCONNECTED) {
        state->error = KW1281_ERROR_NOT_CONNECTED;

        k_mutex_unlock(&state->mutex);
        return 0;
    }

    state->counter = 1;

    state->address = addr | 0x80;       // prepare for (odd) parity calculation
    state->connect_request = 1;

    kw1281_update(state);

    if (wait) {
        if (k_condvar_wait(&state->condvar_connect, &state->mutex, K_MSEC(KW1281_TIMEOUT(state, connect_ms))) < 0) {
            state->disconnect_request = 1;
            kw1281_update(state);

            state->error = KW1281_ERROR_CONNECT_TIMEOUT;

            k_mutex_unlock(&state->mutex);
            return 0;
        }
    }

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_send_block (struct kw1281_state *state, const struct kw1281_block *block, uint8_t wait) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->protocol_state == KW1281_PROTOCOL_STATE_DISCONNECTED) {
        state->error = KW1281_ERROR_NOT_CONNECTED;

        k_mutex_unlock(&state->mutex);
        return 0;
    }

    if (block->length < 3) {
        state->error = KW1281_ERROR_INVALID_ARGUMENT;

        k_mutex_unlock(&state->mutex);
        return 0;
    }

    if (state->tx_block_valid) {
        if (k_condvar_wait(&state->condvar_tx_block, &state->mutex, K_MSEC(KW1281_TIMEOUT(state, transmit_block_ongoing_tx_ms)))) {
            state->disconnect_request = 1;
            kw1281_update(state);

            state->error = KW1281_ERROR_TRANSMIT_BLOCK_TIMEOUT;

            k_mutex_unlock(&state->mutex);
            return 0;
        }
    }

    state->tx_block = *block;
    state->tx_block.counter = state->counter++;
    state->tx_block_valid = 1;

    kw1281_update(state);

    if (wait && state->tx_block_valid) {
        if (k_condvar_wait(&state->condvar_tx_block, &state->mutex, K_MSEC(KW1281_TIMEOUT(state, transmit_block_tx_ms)))) {
            state->disconnect_request = 1;
            kw1281_update(state);

            state->error = KW1281_ERROR_TRANSMIT_BLOCK_TIMEOUT;

            k_mutex_unlock(&state->mutex);
            return 0;
        }
    }

    k_mutex_unlock(&state->mutex);
    return 1;
}

uint8_t kw1281_receive_block (struct kw1281_state *state, struct kw1281_block *block) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    if (state->protocol_state == KW1281_PROTOCOL_STATE_DISCONNECTED) {
        state->error = KW1281_ERROR_NOT_CONNECTED;

        k_mutex_unlock(&state->mutex);
        return 0;
    }

    if (!state->rx_block_valid) {
        if (state->tx_block_valid) {
            // no block received and currently transmitting a block -> wait until transmission is done
            if (k_condvar_wait(&state->condvar_tx_block, &state->mutex, K_MSEC(KW1281_TIMEOUT(state, receive_block_ongoing_tx_ms))) < 0) {
                state->error = KW1281_ERROR_RECEIVE_BLOCK_TIMEOUT;

                k_mutex_unlock(&state->mutex);
                return 0;
            }
        }

        // now we must receive a block

        if (!state->rx_block_started) {
            if (k_condvar_wait(&state->condvar_rx_block_started, &state->mutex, K_MSEC(KW1281_TIMEOUT(state, receive_block_rx_start_ms))) < 0) {
                state->disconnect_request = 1;
                kw1281_update(state);

                state->error = KW1281_ERROR_RECEIVE_BLOCK_TIMEOUT;

                k_mutex_unlock(&state->mutex);
                return 0;
            }
        }

        if (!state->rx_block_valid) {
            if (k_condvar_wait(&state->condvar_rx_block, &state->mutex, K_MSEC(KW1281_TIMEOUT(state, receive_block_rx_ms))) < 0) {
                state->disconnect_request = 1;
                kw1281_update(state);

                state->error = KW1281_ERROR_RECEIVE_BLOCK_TIMEOUT;

                k_mutex_unlock(&state->mutex);
                return 0;
            }
        }
    }

    *block = state->rx_block;
    state->rx_block_valid = 0;
    state->rx_block_started = 0;

    k_mutex_unlock(&state->mutex);
    return 1;
}

void kw1281_disconnect (struct kw1281_state *state) {
    k_mutex_lock(&state->mutex, K_FOREVER);

    state->disconnect_request = 1;
    kw1281_update(state);

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
        case KW1281_ERROR_NOT_CONNECTED:
            return "not connected";

        case KW1281_ERROR_INVALID_ARGUMENT:
            return "invalid argument";

        case KW1281_ERROR_CONNECT_TIMEOUT:
            return "connect timeout";

        case KW1281_ERROR_RECEIVE_BLOCK_TIMEOUT:
            return "receive block timeout";

        case KW1281_ERROR_TRANSMIT_BLOCK_TIMEOUT:
            return "transmit block timeout";


        case KW1281_ERROR_UNKNOWN:
        default:
            return "unknown";
    }
}

const char * kw1281_get_async_error_type_msg (enum kw1281_async_error_type async_error_type) {
    switch (async_error_type) {
        case KW1281_ASYNC_ERROR_COLLISION:
            return "received data did not match sent data";

        case KW1281_ASYNC_ERROR_CONNECT_NO_SYNC:
            return "cannot not sync baudrate";

        case KW1281_ASYNC_ERROR_CONNECT_INVALID_KEY_WORD:
            return "received key word is invalid";

        case KW1281_ASYNC_ERROR_UART_CONFIG:
            return "wrong uart configuration";

        case KW1281_ASYNC_ERROR_RECEIVE_BLOCK_INVALID_LENGTH:
            return "length of the received block is invalid";

        case KW1281_ASYNC_ERROR_RECEIVE_BLOCK_INVALID_COUNTER:
            return "counter of the received block is invalid";

        case KW1281_ASYNC_ERROR_TRANSMIT_INVALID_ACK:
            return "invalid ack received";

        case KW1281_ASYNC_ERROR_OVERRUN:
            return "receive buffer overrun";


        case KW1281_ASYNC_ERROR_UNKNOWN:
        default:
            return "unknown";
    }
}

