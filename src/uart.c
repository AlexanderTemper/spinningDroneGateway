#include <errno.h>
#include <init.h>
#include <uart.h>
#include <sys/ring_buffer.h>
#include <zephyr.h>

#include "uart.h"
#include "globals.h"

struct device *uart_dev;

#include <logging/log.h>
LOG_MODULE_REGISTER( drone_uart, LOG_LEVEL_DBG);

typedef struct userData_s {
    ringbuffer_t *rx;
    ringbuffer_t *tx;
    struct device *uart_dev;
} userData_t;

static userData_t userdata;
// Uart Interrupt Handler
static void interrupt_handler(void *user_data)
{
    userData_t *ud = (userData_t *) user_data;

    struct device *dev = ud->uart_dev;
    static u32_t drop_tx;
    u8_t buf[64];
    size_t wrote, len;
    uart_irq_update(dev);

    if (uart_irq_tx_ready(dev)) {
        len = ring_buf_get(&ud->tx->rb, buf, sizeof(buf));
        if (!len) {
            uart_irq_tx_disable(dev);
        } else {
            printk("Tx send ");
            for (int i = 0; i < len; i++) {
                printk("%d ", buf[i]);
            }
            printk("\n");
            wrote = uart_fifo_fill(dev, buf, len);
        }
    }

    if (uart_irq_rx_ready(dev)) {
        len = uart_fifo_read(dev, buf, sizeof(buf));
        wrote = ring_buf_put(&ud->rx->rb, buf, len);
        if (wrote < len) {
            drop_tx = drop_tx + (len - wrote);
            LOG_ERR("Drop %u bytes tx", drop_tx);
        }
    }
}

void uart_start_tx(void)
{
    uart_irq_tx_enable(userdata.uart_dev);
}

static int uart_init(struct device *dev)
{
    ARG_UNUSED(dev);
    struct device *uart_dev = device_get_binding("UART_1");
    if (uart_dev == NULL) {
        printk("Could not get Uart device\n");
        return -EPERM;
    }

    userdata.rx = &FC_rx;
    userdata.tx = &FC_tx;
    userdata.uart_dev = uart_dev;
    // set up uart irq
    uart_irq_callback_user_data_set(uart_dev, interrupt_handler, &userdata);
    uart_irq_rx_enable(uart_dev);

    return 0;
}

SYS_INIT( uart_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
