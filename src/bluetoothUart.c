#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <init.h>
#include <uart.h>
#include <sys/ring_buffer.h>
#include <zephyr.h>


#include "bluetoothUart.h"

static struct bt_conn *default_conn;

#define LOG_LEVEL CONFIG_BT_GATT_HRS_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(drone,LOG_LEVEL_DBG);


#define RING_BUF_SIZE (64 * 4)


u8_t rx_Buffer[RING_BUF_SIZE];
u8_t tx_Buffer[RING_BUF_SIZE];

static struct bridge_data {
	struct device *uart_dev;
	struct ring_buf rx_ringbuf;
	struct ring_buf tx_ringbuf;
} bridgeBuffer;

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	size_t wrote;
	struct bridge_data *bridge =  attr->user_data;
	struct ring_buf *ringbuf = &bridge->rx_ringbuf;
	struct device *dev = bridge->uart_dev;

	wrote = ring_buf_put(ringbuf, buf, len);
	if (wrote < len) {
		LOG_ERR("Drop %u bytes rx", len - wrote);
	}

	if(wrote > 0) {
		uart_irq_tx_enable(dev);
	}
	return len;
}

static void sdgmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	ARG_UNUSED(attr);
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Drone notifications %s", notif_enabled ? "enabled" : "disabled");
}


BT_GATT_SERVICE_DEFINE(drone_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_UART),
	BT_GATT_CHARACTERISTIC(BT_UUID_UART_TX, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(sdgmc_ccc_cfg_changed),
	BT_GATT_CHARACTERISTIC(BT_UUID_UART_RX, BT_GATT_CHRC_WRITE , BT_GATT_PERM_WRITE, NULL, write_signed, &bridgeBuffer),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
};

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		default_conn = bt_conn_ref(conn);
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

// Uart Interrupt Handler
static void interrupt_handler(void *user_data)
{
	struct bridge_data *bridge = user_data;
	struct device *dev = bridge->uart_dev;
	static u32_t drop_tx;
	u8_t buf[64];
	size_t wrote, len;
	uart_irq_update(dev);

	if (uart_irq_tx_ready(dev)) {
		len = ring_buf_get(&bridge->rx_ringbuf, buf,sizeof(buf));
		if (!len) {
			uart_irq_tx_disable(dev);
		} else {
			wrote = uart_fifo_fill(dev, buf, len);
		}
	}

	if (uart_irq_rx_ready(dev)) {
		len = uart_fifo_read(dev, buf, sizeof(buf));
		wrote = ring_buf_put(&bridge->tx_ringbuf, buf, len);
		if (wrote < len) {
			drop_tx = drop_tx + (len - wrote);
			LOG_ERR("Drop %u bytes tx", drop_tx);
		}
	}
}



static int Uart_init(struct device *dev)
{
	ARG_UNUSED(dev);


	return 0;
}


void bluetoothUartInit(){
	int err;

	static bt_addr_le_t testid;
	testid.type = BT_ADDR_LE_RANDOM;
	testid.a.val[0] = 0x80;
	testid.a.val[1] = 0x08;
	testid.a.val[2] = 0x00;
	testid.a.val[3] = 0x80;
	testid.a.val[4] = 0x08;
	testid.a.val[5] = 0xc0;
	bt_set_id_addr(&testid);
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	struct bridge_data *bridge = &bridgeBuffer;
	struct device *uart_dev = device_get_binding("UART_1");
	if (uart_dev == NULL) {
		printk("Could not get Uart device\n");
		return;
	}

	bridge->uart_dev = uart_dev;
	ring_buf_init(&bridge->rx_ringbuf, sizeof(rx_Buffer), rx_Buffer);
	ring_buf_init(&bridge->tx_ringbuf, sizeof(tx_Buffer), tx_Buffer);

	uart_irq_callback_user_data_set(uart_dev, interrupt_handler, bridge);
	uart_irq_rx_enable(uart_dev);
}


void sdg_notify()
{
	struct bridge_data *bridge = &bridgeBuffer;
	size_t len;
	u8_t buf[20];

	if(!ring_buf_is_empty(&bridge->tx_ringbuf)){
		size_t gesamt = ring_buf_capacity_get(&bridge->tx_ringbuf);
		size_t frei = ring_buf_space_get(&bridge->tx_ringbuf);
		if( (gesamt - frei) > 20 ){ // Wait for data
			len = ring_buf_get(&bridge->tx_ringbuf, buf,sizeof(buf));
			printk("len %d\n", len);
			if (len >= 20){
				bt_gatt_notify(NULL, &drone_svc.attrs[1], &buf, len);
			}
		}
	}
}

SYS_INIT(Uart_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
