#ifndef BLUETOOTH_UART_H_
#define BLUETOOTH_UART_H_

void bluetoothUartInit();
void bluetoothUartNotify();

#define BT_ADDR_UART_0 0x80
#define BT_ADDR_UART_1 0x08
#define BT_ADDR_UART_2 0x00
#define BT_ADDR_UART_3 0x80
#define BT_ADDR_UART_4 0x08
#define BT_ADDR_UART_5 0xc0

#define BT_UUID_UART BT_UUID_DECLARE_16(0x8880)
#define BT_UUID_UART_TX BT_UUID_DECLARE_16(0x8881)
#define BT_UUID_UART_RX BT_UUID_DECLARE_16(0x8882)

#endif /*BLUETOOTH_UART_H_*/
