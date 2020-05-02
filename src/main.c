#include <device.h>
#include <drivers/gpio.h> 

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
#include "sensor.h"


#define LED_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED	DT_ALIAS_LED0_GPIOS_PIN
#define SLEEP_TIME 	10
#define MSP_RC_TO_FC 20
#define MSP_ATTITUDE_FETCH_TIME 10


LOG_MODULE_REGISTER( mainthread, CONFIG_LOG_DEFAULT_LEVEL);
// global definitions
u8_t watchdogPC = 0;

ringbuffer_t PC_rx;
ringbuffer_t PC_tx;
ringbuffer_t FC_rx;
ringbuffer_t FC_tx;

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

//    init_sensor(&tof_front, device_get_binding(DT_INST_1_ST_VL53L0X_LABEL), 0);
//    init_sensor(&tof_down, device_get_binding(DT_INST_0_ST_VL53L0X_LABEL), 20);
//    if (tof_front.dev == NULL) {
//        printk("Could not get VL53L0X frontFace\n");
//        return;
//    }
//    if (tof_down.dev == NULL) {
//        printk("Could not get VL53L0X downFace\n");
//        return;
//    }

    // setup ringbuffers for communication with pc and fc
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
//    expFilterInit(&tof_front.filter.expFilter, 0.3f);
//    tof_front.filter.biquadFilter = &tof_filter_front;

//    biquadFilter_t tof_filter_ground;
//    biquadFilterInit(&tof_filter_ground, 33, (100000/SENSOR_TIME) , BIQUAD_Q, FILTER_LPF);
//    expFilterInit(&tof_down.filter.expFilter, 0.3f);
//    tof_down.filter.biquadFilter = &tof_filter_ground;

    s64_t stopwatch = 0;
    stopwatch = k_uptime_get();
    int time_att = 0;
    while (1) {
        bluetoothUartNotify();
        currentTime = k_uptime_get();

        processMSP(); // Process the MSP Buffer and get new information form PC and FC

//        if (currentTime >= sensorTime) {
//            sensorTime = currentTime + SENSOR_TIME;
//            fetch_distance_sensor(&tof_front);
//            fetch_distance_sensor(&tof_down);
//            //printk("SENSOR %i,%i,%i\n", tof_front.range,tof_down.range,0);
//        }

        if (currentTime >= attitudeFetchTime) {
            int stopwatch =  MSP_ATTITUDE_FETCH_TIME + currentTime - attitudeFetchTime;
            attitudeFetchTime = currentTime + MSP_ATTITUDE_FETCH_TIME;
            int request_count = requestAttitude();
            if(request_count > 1){ // FC did not respond to request
                time_att = time_att + stopwatch;
            } else {
                time_att = stopwatch;
            }
        }

        // Process Controller
        tick(time_att);

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

