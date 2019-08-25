#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <init.h>


#include "bluetoothUart.h"

static struct bt_conn *default_conn;

#define LOG_LEVEL CONFIG_BT_GATT_HRS_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(drone);

static u8_t rx_Buffer[20];

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, u16_t len, u16_t offset,
			    u8_t flags)
{
	u8_t *value = attr->user_data;

	if (offset + len > sizeof(rx_Buffer)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	
	return len;
}

static void sdgmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	ARG_UNUSED(attr);
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}


BT_GATT_SERVICE_DEFINE(drone_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_UART),
	BT_GATT_CHARACTERISTIC(BT_UUID_UART_TX, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(sdgmc_ccc_cfg_changed),
	BT_GATT_CHARACTERISTIC(BT_UUID_UART_RX, BT_GATT_CHRC_WRITE , BT_GATT_PERM_WRITE, NULL, write_signed, rx_Buffer),
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


static int Uart_init(struct device *dev)
{
	ARG_UNUSED(dev);

	memset(rx_Buffer,0,sizeof(rx_Buffer));

	return 0;
}

void bluetoothUartInit(){
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);
}


void sdg_notify(uint16_t dis)
{
	static u8_t sdg[2];

	sdg[0] = (u8_t)(dis >> 8);
	sdg[1] = (u8_t)dis;

	bt_gatt_notify(NULL, &drone_svc.attrs[1], &sdg, sizeof(sdg));
}

SYS_INIT(Uart_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
