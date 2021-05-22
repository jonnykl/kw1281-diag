#ifndef SOFT_UART_H
#define SOFT_UART_H


#include <stdint.h>
#include <device.h>


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
