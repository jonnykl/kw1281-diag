#ifndef SOFT_UART_H
#define SOFT_UART_H


#include <stdint.h>
#include <device.h>


#define SOFT_UART_SLEEP_TICKS_THRESHOLD                 (3)

#define SOFT_UART_DELAY_UNTIL(t_start, delay)                                       \
    do {                                                                            \
        uint32_t elapsed_cycles = (k_cycle_get_32()-(t_start));                     \
        if ((delay) > elapsed_cycles) {                                             \
            uint32_t remaining_ticks = ((uint64_t) ((delay)-elapsed_cycles)) *      \
                CONFIG_SYS_CLOCK_TICKS_PER_SEC / sys_clock_hw_cycles_per_sec();     \
            if (remaining_ticks >= SOFT_UART_SLEEP_TICKS_THRESHOLD) {               \
                k_timeout_t timeout = {                                             \
                    .ticks = remaining_ticks-1                                      \
                };                                                                  \
                k_sleep(timeout);                                                   \
            }                                                                       \
        }                                                                           \
        while((k_cycle_get_32()-(t_start)) < (delay));                              \
    } while (0)



#define SOFT_UART_OK                    (0)
#define SOFT_UART_ERR_TIMEOUT           (-1)
#define SOFT_UART_ERR_FRAME             (-2)
#define SOFT_UART_ERR_TIMING            (-3)
#define SOFT_UART_ERR_CONFIG            (-4)
#define SOFT_UART_ERR_PARITY            (-5)

#define SOFT_UART_PARITY_NONE           (0)
#define SOFT_UART_PARITY_EVEN           (1)
#define SOFT_UART_PARITY_ODD            (2)

#define SOFT_UART_DATA_BITS_7           (0)
#define SOFT_UART_DATA_BITS_8           (1)


struct soft_uart_config {
    const struct device *dev_gpio_tx;
    int pin_gpio_tx;

    const struct device *dev_gpio_rx;
    int pin_gpio_rx;

    uint32_t baudrate;
    uint8_t parity;
    uint8_t data_bits;
};


void soft_uart_init(struct soft_uart_config *config);

int soft_uart_tx(struct soft_uart_config *config, uint8_t data);
int soft_uart_rx(struct soft_uart_config *config, uint8_t *data, uint32_t timeout_ms);
int soft_uart_sync(struct soft_uart_config *config, uint32_t timeout_ms);


#endif /* SOFT_UART_H */
