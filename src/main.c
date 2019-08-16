#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h> 
#include <drivers/sensor.h>
#include <sys/printk.h>
#include <uart.h> 
#include <stdio.h>

#define LED_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED	DT_ALIAS_LED0_GPIOS_PIN

#define SLEEP_TIME 	50

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

	while (1) {
		gpio_pin_write(dev, LED, cnt % 2);
		cnt++;
        ret = sensor_sample_fetch(dev_vlx);
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
			return;
		}

		ret = sensor_channel_get(dev_vlx, SENSOR_CHAN_PROX, &value);
		printk("prox is %d\n", value.val1);

		ret = sensor_channel_get(dev_vlx, SENSOR_CHAN_DISTANCE, &value);
		printf("distance is %.3fm\n", sensor_value_to_double(&value));

		k_sleep(SLEEP_TIME);
	}
}
