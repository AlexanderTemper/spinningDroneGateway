#ifndef BLUETOOTH_UART_H_
#define BLUETOOTH_UART_H_



void bluetoothUartInit();
void sdg_notify(uint16_t dis);

#define BT_UUID_UART BT_UUID_DECLARE_16(0x8880)
#define BT_UUID_UART_TX BT_UUID_DECLARE_16(0x8881)
#define BT_UUID_UART_RX BT_UUID_DECLARE_16(0x8882)




#endif /*BLUETOOTH_UART_H_*/
