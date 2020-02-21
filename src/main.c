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
#include <math.h>

#include "bluetoothle.h"
#include "filter.h"
#include "globals.h"
#include "uart.h"
#include "msp.h"
#include "controller.h"

#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/
#define LED_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED	DT_ALIAS_LED0_GPIOS_PIN

#define SLEEP_TIME 	10

uint16_t distance_mm = 0;
uint16_t distance2_mm = 0;
s16_t thrust_alt = 0;
u8_t watchdogPC = 0;

// global definitions
ringbuffer_t PC_rx;
ringbuffer_t PC_tx;
ringbuffer_t FC_rx;
ringbuffer_t FC_tx;

typedef struct {
    s64_t last_valid_sample_time;
    u32_t valid_timeout;
    u16_t d;
    u16_t sensor_health;
    u16_t sensor_recover;
    struct device *dev;
    u16_t distance;
} distance;

distance frontFace;
distance downFace;

tof_controller_t tof_front;

void init_sensor(distance *dis)
{
    dis->d = 0;
    dis->valid_timeout = 0;
    dis->last_valid_sample_time = 0;
    dis->sensor_health = 1;
    dis->sensor_recover = 0;
    dis->distance = 0;
}
/**
 * 1 old but still valid data
 * 0 new distance data
 * -1 unkown error
 * -2 timeout of last valid read
 * -3 senor is recovering distance is last valid read (before recover)
 * recover == amount of readings after error (out range or unknown error
 */
int fetch_distance(distance *dis, int recover)
{

    // last valid value
    dis->distance = dis->d;

    struct sensor_value value;
    int ret = sensor_sample_fetch(dis->dev);

    if (ret != -EBUSY && ret != 0) { // unknown error
        dis->sensor_health = 0;
        dis->sensor_recover = 0;
        return -1;
    }

    if (ret == 0 && sensor_channel_get(dis->dev, SENSOR_CHAN_DISTANCE, &value) == 0) { // no error and new valid value
        dis->last_valid_sample_time = k_uptime_get_32();
        dis->valid_timeout = 0;

        if (dis->sensor_health == 0) {
            dis->sensor_recover++;

            if (dis->sensor_recover == recover) {
                dis->sensor_health = 1;
                dis->sensor_recover = 0;
            } else { // we still recover
                return -3;
            }
        }

        if (dis->sensor_health == 1) {
            dis->d = sensor_value_to_double(&value) * 1000; // range in mm
            dis->distance = dis->d;
            return 0;
        }

    }

    dis->valid_timeout = dis->valid_timeout + k_uptime_delta_32(&dis->last_valid_sample_time);

    if (dis->valid_timeout > 100) { // 100ms
        dis->sensor_health = 0;
        dis->sensor_recover = 0;
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
    gpio_pin_configure(dev, LED, GPIO_OUTPUT);

    printk("Hello World!\n");

    init_sensor(&frontFace);
    frontFace.dev = device_get_binding(DT_INST_1_ST_VL53L0X_LABEL);
    init_sensor(&downFace);
    downFace.dev = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
    if (frontFace.dev == NULL) {
        printk("Could not get VL53L0X frontFace\n");
        return;
    }

    if (downFace.dev == NULL) {
        printk("Could not get VL53L0X downFace\n");
        return;
    }

    init_ringbuffer(&PC_rx);
    init_ringbuffer(&PC_tx);
    init_ringbuffer(&FC_rx);
    init_ringbuffer(&FC_tx);

    gpio_pin_set(dev, LED, 1);
    u32_t cycles_spent = 0;
    u32_t cycles_spent1 = 0;
#define MSP_ATTITUDE_FETCH_TIME 1000
    s64_t attitudeFetchTime = 0;

#define MSP_RC_TO_FC 20
    s64_t rcSendToFCTime = 0;

    s64_t currentTime = 0;

    //biquadFilter_t tof_filter;
    //biquadFilterInit(&tof_filter, 10, 500, BIQUAD_Q, FILTER_LPF); //TODO 500 auf refrashrate

    while (1) {

        bluetoothUartNotify();
        currentTime = k_uptime_get_32();

//        int ret = fetch_distance(&frontFace,1);
//
//        if (ret == 0) {
//            tof_front.time = (SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32() - cycles_spent) / 1000);
//            tof_front.range = biquadFilterApply(&tof_filter, frontFace.distance);
//            cycles_spent = k_cycle_get_32();
//        } else if(ret == 1) { //work with bufferd data
//            tof_front.time = 0;
//            tof_front.range = frontFace.distance;
//        } else { //did not get valid data
//            tof_front.range = -1;
//        }
//
        if (fetch_distance(&downFace, 15) == 0) {
            getAltitudeThrottle(getEstimatedAltitude(downFace.distance), 200);
            printk("distanceDown %i |took %u\n", downFace.distance, SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32() - cycles_spent1) / 1000);
            cycles_spent1 = k_cycle_get_32();
        }
        if (fetch_distance(&frontFace, 1) == 0) {
            getAltitudeThrottle(getEstimatedAltitude(frontFace.distance), 200);
            printk("distanceFront %i |took %u\n", frontFace.distance, SYS_CLOCK_HW_CYCLES_TO_NS(k_cycle_get_32() - cycles_spent) / 1000);
            cycles_spent = k_cycle_get_32();
        }

        processMSP();

        if (currentTime >= attitudeFetchTime) {

            attitudeFetchTime = currentTime + MSP_ATTITUDE_FETCH_TIME;
            fetchAttitude();
        }

        // Process Controller
        tick();

        if (currentTime >= rcSendToFCTime) {
            rcSendToFCTime = currentTime + MSP_RC_TO_FC;
            //printk("delta %d\n",delta);
            // TODO make a timeout for the data so if connection is lost the quadcopter is landing
            if (watchdogPC < 50) { //1sec Timeout
                watchdogPC++;
                //printk("rc to fc %i,%i,%i,%i,%i\n", rcControl.rcdata.roll,rcControl.rcdata.pitch,rcControl.rcdata.yaw,rcControl.rcdata.throttle,rcControl.rcdata.arm);
                sendRCtoFC();
            } else if (watchdogPC == 50) {
                resetController();
                printk("Watchdog was not reseted\n");
                watchdogPC++;
            }
        }

    }
}

