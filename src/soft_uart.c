#include <zephyr.h>
#include <drivers/gpio.h>

#include "soft_uart.h"


// TODO
#define SOFT_UART_RX_THREAD_STACK_SIZE              256
#define SOFT_UART_RX_THREAD_PRIORITY                -5
#define SOFT_UART_TX_THREAD_PRIORITY                -5


// TODO
static struct k_thread soft_uart_rx_thread_data;
static K_THREAD_STACK_DEFINE(soft_uart_rx_thread_stack, SOFT_UART_RX_THREAD_STACK_SIZE);
static k_tid_t soft_uart_rx_thread_id;

static int foo_value = 0;


// TODO: use PWM capture instead (if this makes sense ...)
// if not:
// gpio callback (falling edge) ->
// - save current cycle count
// - disable interrupt
// - start rx
// OR:
// gpio callback (both edges) ->
// - save current cycle count to list
// - evaluate received data in thread

static void soft_uart_rx_gpio_handler (const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int pin = 0;
    while (pins != 1) {
        pin += 1;
        pins >>= 1;
    }

    gpio_pin_interrupt_configure(dev, pin, GPIO_INT_DISABLE);

    foo_value = !foo_value;
    gpio_pin_set_raw(dev, 1, foo_value);
}

static void soft_uart_rx_thread (void *p1, void *p2, void *p3) {
    struct soft_uart_config *config = p1;

    static struct gpio_callback callback;
    gpio_init_callback(&callback, soft_uart_rx_gpio_handler, BIT(config->pin_gpio_rx));
    gpio_add_callback(config->dev_gpio_rx, &callback);

    gpio_pin_interrupt_configure(config->dev_gpio_rx, config->pin_gpio_rx, GPIO_INT_EDGE_FALLING);
    //gpio_pin_interrupt_configure(config->dev_gpio_rx, config->pin_gpio_rx, GPIO_INT_DISABLE);
    //gpio_pin_interrupt_configure(config->dev_gpio_rx, config->pin_gpio_rx, GPIO_INT_EDGE_FALLING);

    for (;;) {
        //gpio_pin_interrupt_configure(config->dev_gpio_rx, config->pin_gpio_rx, GPIO_INT_EDGE_FALLING);
        //gpio_pin_interrupt_configure(config->dev_gpio_rx, config->pin_gpio_rx, GPIO_INT_DISABLE);
        k_msleep(1000);
    }
}


void soft_uart_init (struct soft_uart_config *config) {
    gpio_pin_configure(config->dev_gpio_tx, config->pin_gpio_tx, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(config->dev_gpio_rx, config->pin_gpio_rx, GPIO_INPUT | GPIO_PULL_UP);

    // debug
    gpio_pin_configure(config->dev_gpio_rx, 1, GPIO_OUTPUT_LOW);
    foo_value = 0;

    soft_uart_rx_thread_id = k_thread_create(&soft_uart_rx_thread_data,
            soft_uart_rx_thread_stack, K_THREAD_STACK_SIZEOF(soft_uart_rx_thread_stack),
            soft_uart_rx_thread, config, NULL, NULL,
            SOFT_UART_RX_THREAD_PRIORITY, 0, K_NO_WAIT);
}


int soft_uart_tx (struct soft_uart_config *config, uint8_t data) {
    uint32_t bit_cycles =  sys_clock_hw_cycles_per_sec() / config->baudrate;

    uint16_t data_with_parity;

    uint8_t num_bits;
    if (config->data_bits == SOFT_UART_DATA_BITS_7) {
        num_bits = 7;
        data_with_parity = data & 0x7F;
    } else if (config->data_bits == SOFT_UART_DATA_BITS_8) {
        num_bits = 8;
        data_with_parity = data & 0xFF;
    } else {
        return SOFT_UART_ERR_CONFIG;
    }

    uint8_t parity;
    if (config->parity == SOFT_UART_PARITY_EVEN) {
        parity = 0;
    } else if (config->parity == SOFT_UART_PARITY_ODD) {
        parity = 1;
    } else if (config->parity != SOFT_UART_PARITY_NONE) {
        return SOFT_UART_ERR_CONFIG;
    }

    // parity bits
    if (config->parity != SOFT_UART_PARITY_NONE) {
        for (int i=0; i<num_bits; i++) {
            if (data & (1<<i)) {
                parity ^= 1;
            }
        }

        if (parity) {
            data_with_parity |= 1<<num_bits;
        }

        num_bits++;
    }

    // start bit
    data_with_parity <<= 1;
    num_bits++;

    // stop bit
    data_with_parity |= 1<<num_bits;
    num_bits++;


    k_tid_t tid = k_current_get();
    int old_priority = k_thread_priority_get(tid);

    // temporarily increase priority
    k_thread_priority_set(tid, SOFT_UART_TX_THREAD_PRIORITY);

    uint32_t t_start = k_cycle_get_32();

    // transmit bits
    for (int i=0; i<num_bits; i++) {
        int bit = data_with_parity & 1;
        gpio_pin_set_raw(config->dev_gpio_tx, config->pin_gpio_tx, bit);

        data_with_parity >>= 1;

        // skip equal bits
        while (i < (num_bits-1) && (data_with_parity & 1) == bit) {
            i++;
            data_with_parity >>= 1;
        }

        SOFT_UART_DELAY_UNTIL(t_start, (i+1)*bit_cycles);
    }

    k_thread_priority_set(tid, old_priority);


    return SOFT_UART_OK;
}

int soft_uart_rx (struct soft_uart_config *config, uint8_t *data, uint32_t timeout_ms) {
    uint32_t t_timeout_start = k_cycle_get_32();
    uint32_t timeout_cycles = sys_clock_hw_cycles_per_sec() / 1000 * timeout_ms;

    uint32_t bit_cycles =  sys_clock_hw_cycles_per_sec() / config->baudrate;

    uint16_t data_with_parity = 0;

    uint8_t num_bits;
    if (config->data_bits == SOFT_UART_DATA_BITS_7) {
        num_bits = 7;
    } else if (config->data_bits == SOFT_UART_DATA_BITS_8) {
        num_bits = 8;
    } else {
        return SOFT_UART_ERR_CONFIG;
    }

    uint8_t parity = 0;
    if (config->parity == SOFT_UART_PARITY_EVEN) {
        parity = 0;
        num_bits++;
    } else if (config->parity == SOFT_UART_PARITY_ODD) {
        parity = 1;
        num_bits++;
    } else if (config->parity != SOFT_UART_PARITY_NONE) {
        return SOFT_UART_ERR_CONFIG;
    }

    while (timeout_ms == 0 || (k_cycle_get_32()-t_timeout_start) < timeout_cycles) {
        if (gpio_pin_get_raw(config->dev_gpio_rx, config->pin_gpio_rx)) {
            continue;
        }

        uint32_t t_start = k_cycle_get_32();
        SOFT_UART_DELAY_UNTIL(t_start, bit_cycles/4);
        t_start += bit_cycles/4;

        for (int i=0; i<(num_bits+2); i++) {
            int bit = gpio_pin_get_raw(config->dev_gpio_rx, config->pin_gpio_rx);
            if (i == 0) {
                if (bit != 0) {
                    return SOFT_UART_ERR_FRAME;
                }
            } else if (i == num_bits+1) {
                if (bit != 1) {
                    return SOFT_UART_ERR_FRAME;
                }
            } else {
                data_with_parity >>= 1;
                data_with_parity |= bit<<(num_bits-1);

                if (bit) {
                    parity ^= 1;
                }
            }

            SOFT_UART_DELAY_UNTIL(t_start, (i+1)*bit_cycles);
        }

        if (config->parity != SOFT_UART_PARITY_NONE) {
            if (parity != 0) {
                return SOFT_UART_ERR_PARITY;
            }

            *data = data_with_parity & ~(1<<(num_bits-1));
        } else {
            *data = data_with_parity;
        }

        return SOFT_UART_OK;
    }

    return SOFT_UART_ERR_TIMEOUT;
}

// TODO
int soft_uart_sync (struct soft_uart_config *config, uint32_t timeout_ms) {
    uint32_t t_timeout_start = k_cycle_get_32();
    uint32_t timeout_cycles = sys_clock_hw_cycles_per_sec() / 1000 * timeout_ms;

    uint32_t bit_time_deltas[9];
    uint32_t t_now;
    uint32_t t_bit_start = 0;
    uint8_t state = 0;
    uint8_t wait_for_level = 0;

    while (timeout_ms == 0 || (k_cycle_get_32()-t_timeout_start) < timeout_cycles) {
        if (gpio_pin_get_raw(config->dev_gpio_rx, config->pin_gpio_rx) != wait_for_level) {
            continue;
        }

        t_now = k_cycle_get_32();

        if (state > 0) {
            bit_time_deltas[state-1] = t_now - t_bit_start;
            if (state == 9) {
                uint32_t sum = 0;
                for (int i=0; i<9; i++) {
                    sum += bit_time_deltas[i];
                }

                uint32_t avg = sum/9;

                for (int i=0; i<9; i++) {
                    int32_t deviation = ((int32_t) bit_time_deltas[i]) - ((int32_t) avg);
                    deviation = deviation > 0 ? deviation : -deviation;

                    // allow a deviation of 25% from the avergae
                    if (deviation > avg/4) {
                        return SOFT_UART_ERR_TIMING;
                    }
                }

                config->baudrate = ((sys_clock_hw_cycles_per_sec()*2 / avg) + 1)/2;

                return SOFT_UART_OK;
            }

            // 0x55 =  01010101
            // 11111 0 10101010 1 11111
            //       |          |
            // start (inc)      end (excl)
            // -> 9 bits
        }

        wait_for_level = !wait_for_level;

        t_bit_start = t_now;
        state++;
    }

    return SOFT_UART_ERR_TIMEOUT;
}

