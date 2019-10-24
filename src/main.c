#include <device.h>
#include <drivers/gpio.h> 
#include <drivers/sensor.h>
#include <errno.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <sys/ring_buffer.h>

#include "bluetoothle.h"
#include "globals.h"
#include "uart.h"
#include "msp.h"
#include "controller.h"

#define LED_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED	DT_ALIAS_LED0_GPIOS_PIN

#define SLEEP_TIME 	10

uint16_t distance_mm = 0;
s16_t thrust_alt = 0;

// global definitions
ringbuffer_t PC_rx;
ringbuffer_t PC_tx;
ringbuffer_t FC_rx;
ringbuffer_t FC_tx;

/**
 * 1 old but still valid data
 * 0 new distance data
 * -1 unkown error
 * -2 timeout of last valid read
 * -3 senor is recovering distance is last valid read (before recover)
 */
int fetch_distance(struct device *dev, u16_t *distance)
{
    static s64_t last_valid_sample_time = 0;
    static u32_t valid_timeout = 0;
    static u16_t d = 0;
    static u16_t sensor_health = 1;
    static u16_t sensor_recover = 0;

    // last valid value
    *distance = d;

    struct sensor_value value;
    int ret = sensor_sample_fetch(dev);

    if (ret != -EBUSY && ret != 0) { // unknown error
        sensor_health = 0;
        sensor_recover = 0;
        return -1;
    }

    if (ret == 0 && sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &value) == 0) { // no error and new valid value
        last_valid_sample_time = k_uptime_get_32();
        valid_timeout = 0;

        if (sensor_health == 0) {
            sensor_recover++;

            if (sensor_recover == 15) { // ca 500ms
                sensor_health = 1;
                sensor_recover = 0;
            } else { // we still recover
                return -3;
            }
        }

        if (sensor_health == 1) {
            d = sensor_value_to_double(&value) * 1000; // range in mm
            *distance = d;
            return 0;
        }

    }

    valid_timeout = valid_timeout + k_uptime_delta_32(&last_valid_sample_time);

    if (valid_timeout > 100) { // 100ms
        sensor_health = 0;
        sensor_recover = 0;
        return -2;
    }
    return 1; // load distance from buffer
}

void init_ringbuffer(ringbuffer_t *ringbuffer)
{
    ring_buf_init(&ringbuffer->rb, sizeof(ringbuffer->buffer), ringbuffer->buffer);
}
void main(void)
{
    struct device *dev;

    dev = device_get_binding(LED_PORT);
    /* Set LED pin as output */
    gpio_pin_configure(dev, LED, GPIO_DIR_OUT);

    printk("Hello World!\n");

    struct device *dev_vlx = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
    if (dev_vlx == NULL) {
        printk("Could not get VL53L0X device\n");
        return;
    }

    init_ringbuffer(&PC_rx);
    init_ringbuffer(&PC_tx);
    init_ringbuffer(&FC_rx);
    init_ringbuffer(&FC_tx);

    //u32_t cycles_spent = 0;
    while (1) {

        gpio_pin_write(dev, LED, 1);
        if (fetch_distance(dev_vlx, &distance_mm) == 0) {

            getAltitudeThrottle(getEstimatedAltitude(distance_mm),200);
            //printf("distance is %i|%i took %u\n", getEstimatedAltitude(distance_mm), distance_mm, SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32() - cycles_spent) / 1000);
            //cycles_spent = k_cycle_get_32();
        }
        processMSP();


        bluetoothUartNotify();


    }
}
