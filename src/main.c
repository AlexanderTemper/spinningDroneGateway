#include <device.h>
#include <drivers/gpio.h> 
#include <drivers/sensor.h>
#include <logging/log.h>
#include <errno.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <sys/ring_buffer.h>
#include <math.h>

#include "bluetoothle.h"
#include "filter.h"
#include "globals.h"
#include "uart.h"
#include "msp.h"
#include "controller.h"


#define LED_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED	DT_ALIAS_LED0_GPIOS_PIN
#define SLEEP_TIME 	10
#define MSP_RC_TO_FC 20
#define MSP_ATTITUDE_FETCH_TIME 10


LOG_MODULE_REGISTER( mainthread, CONFIG_LOG_DEFAULT_LEVEL);
// global definitions
u16_t thrust_alt = 0;
u8_t watchdogPC = 0;

ringbuffer_t PC_rx;
ringbuffer_t PC_tx;
ringbuffer_t FC_rx;
ringbuffer_t FC_tx;


void init_sensor(tof_controller_t *sensor, struct device *dev)
{
    sensor->range = 0;
    sensor->dev = dev;
    sensor->time_last_read = 0;
}

/**
 * get range data from sensor
 * the sensor can timeout TOF_TIMEOUT_MS
 * the sensor has an minimal polling interval of TOF_POLLING_TIME_MS
 */
void fetch_distance(tof_controller_t *sensor)
{
    struct sensor_value value;
    int ret = sensor_sample_fetch(sensor->dev);

    if ((k_uptime_get() - sensor->time_last_read) <= TOF_POLLING_TIME_MS) {
        return;
    }

    if (ret != -EBUSY && ret != 0) { // unknown error of Sensor
        filterReset(&sensor->filter);
        sensor->range = -EIO;
        return;
    }

    if (ret == -EBUSY) {
        if ((k_uptime_get() - sensor->time_last_read) >= TOF_TIMEOUT_MS) { //reset data on Timeout
            filterReset(&sensor->filter);
            sensor->range = -EIO;
        }
        return;
    }

    if (sensor_channel_get(sensor->dev, SENSOR_CHAN_DISTANCE, &value) == 0) { // no error and new valid value
        sensor->range = sensor_value_to_double(&value) * 1000; // range in mm;
        sensor->time_between_reads = k_uptime_get() - sensor->time_last_read;
        sensor->time_last_read = k_uptime_get();
        sensor->isNew = true;
        if (sensor->range > TOF_MAX_RANGE) {
            sensor->range = TOF_MAX_RANGE;
        }
        sensor->range = filterApply(&sensor->filter, sensor->range);
        return;
    }

    filterReset(&sensor->filter);
    sensor->range = -EIO;
}

void init_ringbuffer(ringbuffer_t *ringbuffer)
{
    ring_buf_init(&ringbuffer->rb, sizeof(ringbuffer->buffer), ringbuffer->buffer);
}

#define MSP_RC_TO_FC 20
#define MSP_ATTITUDE_FETCH_TIME 10
#define SENSOR_TIME 33
void main(void)
{
    LOG_INF("start main thread");
    // Set LED pin as output
    struct device *setup_status_led_dev;
    setup_status_led_dev = device_get_binding(LED_PORT);
    gpio_pin_configure(setup_status_led_dev, LED, GPIO_OUTPUT);
    gpio_pin_set(setup_status_led_dev, LED, 0);

    init_sensor(&tof_front, device_get_binding(DT_INST_1_ST_VL53L0X_LABEL));
    init_sensor(&tof_down, device_get_binding(DT_INST_0_ST_VL53L0X_LABEL));
    if (tof_front.dev == NULL) {
        printk("Could not get VL53L0X frontFace\n");
        return;
    }
    if (tof_down.dev == NULL) {
        printk("Could not get VL53L0X downFace\n");
        return;
    }

    // setup ringbuffes for communication with pc and fc
    init_ringbuffer(&PC_rx);
    init_ringbuffer(&PC_tx);
    init_ringbuffer(&FC_rx);
    init_ringbuffer(&FC_tx);

    s64_t attitudeFetchTime = 0;
    s64_t rcSendToFCTime = 0;
    s64_t sensorTime = 0;
    s64_t currentTime = 0;
    // all setups done
    gpio_pin_set(setup_status_led_dev, LED, 1);

    LOG_INF("start main while loop");

//    biquadFilter_t tof_filter_front;
//    biquadFilterInit(&tof_filter_front, 33, (100000/SENSOR_TIME) , BIQUAD_Q, FILTER_LPF);
    expFilterInit(&tof_front.filter.expFilter, 0.3f);
//    tof_front.filter.biquadFilter = &tof_filter_front;

//    biquadFilter_t tof_filter_ground;
//    biquadFilterInit(&tof_filter_ground, 33, (100000/SENSOR_TIME) , BIQUAD_Q, FILTER_LPF);
    expFilterInit(&tof_down.filter.expFilter, 0.3f);
//    tof_down.filter.biquadFilter = &tof_filter_ground;


    filterInit(&tof_front.filter);
    while (1) {

        bluetoothUartNotify();
        currentTime = k_uptime_get();

        processMSP(); // Process the MSP Buffer and get new information form PC and FC

        if (currentTime >= sensorTime) {
            sensorTime = currentTime + SENSOR_TIME;
            fetch_distance(&tof_front);
            fetch_distance(&tof_down);
            //printk("SENSOR %i,%i,%i\n", tof_front.range,tof_down.range,0);
        }

        if (currentTime >= attitudeFetchTime) {
            attitudeFetchTime = currentTime + MSP_ATTITUDE_FETCH_TIME;
            requestAttitude();
        }

        // Process Controller
        tick();

        if (currentTime >= rcSendToFCTime) {
            rcSendToFCTime = currentTime + MSP_RC_TO_FC;
            //printk("delta %d\n",delta);
            // TODO make a timeout for the data so if connection is lost the quadcopter is landing
            if (watchdogPC < 50) { //1sec Timeout
                watchdogPC++;
                //printk("rc to fc %i,%i,%i,%i,%i,%i\n", rcControl.rcdata.roll,rcControl.rcdata.pitch,rcControl.rcdata.yaw,rcControl.rcdata.throttle,rcControl.rcdata.mode,rcControl.rcdata.arm);
                sendRCtoFC();
            } else if (watchdogPC == 50) {
                resetController();
                printk("Watchdog was not reseted\n");
                watchdogPC++;
            }
        }

    }
}

