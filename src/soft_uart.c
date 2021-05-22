#include <zephyr.h>
#include <drivers/gpio.h>

#include "soft_uart.h"


void soft_uart_init (struct soft_uart_config *config) {
    gpio_pin_configure(config->dev_gpio_tx, config->pin_gpio_tx, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(config->dev_gpio_rx, config->pin_gpio_rx, GPIO_INPUT | GPIO_PULL_UP);
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

    uint32_t t_start = k_cycle_get_32();
    gpio_pin_set_raw(config->dev_gpio_tx, config->pin_gpio_tx, 0);

    while((k_cycle_get_32()-t_start) < 1*bit_cycles);

    for (int i=0; i<num_bits; i++) {
        gpio_pin_set_raw(config->dev_gpio_tx, config->pin_gpio_tx, data_with_parity & 1);
        while((k_cycle_get_32()-t_start) < (i+2)*bit_cycles);

        data_with_parity >>= 1;
    }

    gpio_pin_set_raw(config->dev_gpio_tx, config->pin_gpio_tx, 1);
    while((k_cycle_get_32()-t_start) < (num_bits+2)*bit_cycles);


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
        while((k_cycle_get_32()-t_start) < bit_cycles/4);
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

            while((k_cycle_get_32()-t_start) < (i+1)*bit_cycles);
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

