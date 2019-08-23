#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <device.h>
#include <drivers/gpio.h> 
#include <drivers/sensor.h>
#include <errno.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <uart.h> 
#include <zephyr.h>
#include <zephyr/types.h>

#define LED_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED	DT_ALIAS_LED0_GPIOS_PIN

#define SLEEP_TIME 	100


uint16_t distance_mm = 0;
struct bt_conn *default_conn;


#define LOG_LEVEL CONFIG_BT_GATT_HRS_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(drone);

static void sdgmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	ARG_UNUSED(attr);
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}

#define BT_UUID_SDG BT_UUID_DECLARE_16(0x8880)
#define BT_UUID_SDG_MEASUREMENT BT_UUID_DECLARE_16(0x8881)

BT_GATT_SERVICE_DEFINE(drone_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SDG),
	BT_GATT_CHARACTERISTIC(BT_UUID_SDG_MEASUREMENT, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(sdgmc_ccc_cfg_changed),
);

/*	BT_GATT_CHARACTERISTIC(BT_UUID_HRS_BODY_SENSOR, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_blsc, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_HRS_CONTROL_POINT, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
			       */


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


static void sdg_notify(void)
{
	static u8_t sdg[2];

	sdg[0] = (u8_t)(distance_mm >> 8);
	sdg[1] = (u8_t)distance_mm;

	bt_gatt_notify(NULL, &drone_svc.attrs[1], &sdg, sizeof(sdg));
}

void main(void)
{
	int cnt = 0;
	struct device *dev;

	dev = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);
  
    printk("Hello World!\n"); 

    struct device *dev_vlx = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	struct sensor_value value;
	int ret;

	if (dev_vlx == NULL) {
		printk("Could not get VL53L0X device\n");
		return;
	}
	
	
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);


	while (1) {
		gpio_pin_write(dev, LED, cnt % 2);
		cnt++;
        ret = sensor_sample_fetch(dev_vlx);
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
			return;
		}

		ret = sensor_channel_get(dev_vlx, SENSOR_CHAN_PROX, &value);
		//printk("prox is %d\n", value.val1);

		ret = sensor_channel_get(dev_vlx, SENSOR_CHAN_DISTANCE, &value);
		distance_mm = sensor_value_to_double(&value)*1000;
		
		
		//printf("distance is %i\n",distance_mm );
		sdg_notify();

		
		k_sleep(SLEEP_TIME);
	}
}
