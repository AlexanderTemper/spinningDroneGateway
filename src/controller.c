#include <zephyr.h>
#include <init.h>
#include <math.h>

#include "controller.h"
#include "globals.h"
#include "streambuf.h"
#include "msp.h"

#define M_PI        3.14159265358979323846
#define HOLD_MM 300

altHoldPid_t altHold;
rc_control_t rcControl;
att_data_t att_data;

tof_controller_t tof_front;
tof_controller_t tof_down;

flight_mode modus = IDLE;
int16_t headFreeModeHold = 0;

filter_t climb_rate_filter;
biquadFilter_t filterb;


//u32_t microsDiff(u32_t previousC, u32_t currentC)
//{
//    return SYS_CLOCK_HW_CYCLES_TO_NS(currentC - previousC) / 1000;
//}

void reset_tof(tof_controller_t *tof)
{
    tof->integral_e = 0;
    tof->l_error = 0;
    tof->integral_e = 0;
    tof->derivative1 = 0;
    tof->derivative2 = 0;
    tof->last_froce = 0;
}
// last data from pc
u16_t chan[RC_CHANAL_COUNT];
void resetController()
{
    reset_tof(&tof_down);
    thrust_alt = 0;
    modus = IDLE;
    for (int i = 0; i < RC_CHANAL_COUNT; i++) {
        chan[i] = 0;
    }
    rcControl.rcdata.throttle = 1000;
    printk("Controller reseted\n");
}

int get_alt_error(tof_controller_t *tof, u16_t target_distance){
    int offsetSensor = 20; //offset sensor is away from collison
    return target_distance - (tof->range - offsetSensor);
}
void resetHoldMode()
{
    reset_tof(&tof_down);
    thrust_alt = 0;
    tof_down.l_error = get_alt_error(&tof_down, HOLD_MM);
    expFilterInit(&climb_rate_filter.expFilter, 0.3f);
    biquadFilterInit(&filterb, 33, (100000/33) , BIQUAD_Q, FILTER_LPF);
    tof_down.filter.biquadFilter = &filterb;
    printk("Reset Hold Mode\n");
}

int pushController(tof_controller_t *tof)
{
    float refreshTime = tof->time_between_reads;
    float pterm = tof->pterm;
    //float iterm = tof->iterm;
    float dterm = tof->dterm;
    int offsetSensor = 0; //offset sensor is away from collison

    if (tof->range < 0) { // No Sensor Value
        return 0;
    }

    int error = 1000 - (tof->range - offsetSensor);
    if (error < 0) { // we are not in danger zone
        return 0;
    }

    int ki = 0, kp = 0, kd = 0;
    kp = constrain(pterm * error, -500, +500);

    float derivative = ((error - tof->l_error) / refreshTime) * 33;
    float derivativeSum = tof->derivative1 + tof->derivative2 + derivative;
    tof->derivative2 = tof->derivative1;
    tof->derivative1 = derivative;
    float derivativeFiltered = derivativeSum / 3;
    kd = constrain(dterm * derivativeFiltered, -250, +250);

    tof->l_error = error;
    int force = kp + ki + kd;

    //printk("[%i,%i] | [p=%i,d=%i,i=%i] |force=%i\n", tof->range, error, kp, kd, ki, force);
    //printk("SENSOR %i,%i,%i\n", kp, kd,tof->range);
    if (force < 0) {
        return 0;
    }
    tof->last_froce = force;
    return tof->last_froce;
}

void calcPushback(int16_t *roll, int16_t *pitch, tof_controller_t *tof)
{
    int force = 0;
    force = pushController(tof);

    switch (tof->direction) {
    case FRONT:
        *pitch = -force;
        break;
    case REAR:
        *pitch = force;
        break;
    case RIGHT:
        *roll = -force;
        break;
    case LEFT:
        *roll = force;
        break;
    }
}

void calcHeadFree(int16_t *roll, int16_t *pitch)
{
    float radDiff = (att_data.yaw - headFreeModeHold) * M_PI / 180.0f;
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);

    int16_t rcCommand_PITCH = *pitch * cosDiff + *roll * sinDiff;
    *roll = *roll * cosDiff - *pitch * sinDiff;
    *pitch = rcCommand_PITCH;
}

void tick()
{
    if (chan[RC_ARM] < 1600) { // Hard reset
        modus = IDLE;
    }

    //Throttle controller
    switch (modus) {
    case IDLE:
        rcControl.rcdata.throttle = 1000;
        if (chan[RC_ARM] >= 1600) {
            modus = ARMED;
        }
        break;
    case ARMED:
        if (chan[MODE_SWITCH] >= 1200) {
            modus = HOLD;
            resetHoldMode();
            headFreeModeHold = att_data.yaw;
            //printk("takeoff start\n");
        } else {
            rcControl.rcdata.throttle = constrain(chan[RC_THROTTLE], 1000, 1800);
        }
        break;
    case HOLD:  // Manipulate RC Data
        if (chan[MODE_SWITCH] < 1200) {
            modus = NORMALIZE;
        } else {

            // Normalize RC Data
            int16_t roll = (int16_t) chan[RC_ROLL] - 1500; //norm data to -500..500
            int16_t pitch = (int16_t) chan[RC_PITCH] - 1500;

            calcHeadFree(&roll, &pitch);

            // update data with push forces from tof sensors
            int16_t pushRoll = 0, pushPitch = 0;
            calcPushback(&pushRoll, &pushPitch, &tof_front);
            roll = constrain(roll + pushRoll, -500, 500);
            pitch = constrain(pitch + pushPitch, -500, 500);

            if (tof_down.isNew) { //Only update if new data is available
                getAltitudeThrottle(&tof_down,HOLD_MM,thrust_alt);
                tof_down.isNew = false;
            }
            // update throttle bases on altitude  hold
            rcControl.rcdata.throttle = constrain(chan[RC_THROTTLE] + thrust_alt, 1100, 1800);


            // shift rc data back to transmit
            rcControl.rcdata.roll = constrain(roll + 1500, 1000, 2000);
            rcControl.rcdata.pitch = constrain(pitch + 1500, 1000, 2000);
            rcControl.rcdata.yaw = chan[RC_YAW];

            rcControl.rcdata.arm = chan[RC_ARM];
            rcControl.rcdata.mode = chan[RC_MODE];

            //printk("alt hold [%d %i %d]\n", chan[RC_THROTTLE], thrust_alt, rcControl.rcdata.throttle);
            //printk("heading diff [%i] [%d] [%d]\n", (att_data.yaw - headFreeModeHold), att_data.yaw, headFreeModeHold);

            return;
        }
        break;
    case NORMALIZE:
        if (rcControl.rcdata.throttle == chan[RC_THROTTLE]) {
            modus = ARMED;
            //printk("landing done\n");
        } else {
            // Todo Timeout
            static s64_t landingTimer = 0;
            s64_t currentTime = k_uptime_get();
            if (currentTime >= landingTimer) {
                landingTimer = currentTime + 50;
                rcControl.rcdata.throttle = constrain(rcControl.rcdata.throttle - 10, chan[RC_THROTTLE], 1800); // decrease to RC_THRUST
            }

            //printk("landing [%d]\n", rcControl.rcdata.throttle);
        }

        break;
    default:
        modus = IDLE;
        rcControl.rcdata.throttle = 1000;
    }

    rcControl.rcdata.roll = chan[RC_ROLL];
    rcControl.rcdata.pitch = chan[RC_PITCH];
    rcControl.rcdata.yaw = chan[RC_YAW];

    rcControl.rcdata.arm = chan[RC_ARM];
    rcControl.rcdata.mode = chan[RC_MODE];
}
void rc_data_frame_received(sbuf_t *src)
{
    watchdogPC = 0; // copy data
    for (int i = 0; i < RC_CHANAL_COUNT; i++) {
        chan[i] = sbufReadU16(src);
    }
}


u16_t getAltitudeThrottle(tof_controller_t *tof, u16_t target_distance,u16_t current_thr)
{
    float refreshTime = tof->time_between_reads;
    float pterm = altHold.climb_p;
    float iterm = altHold.climb_i;
    float dterm = altHold.climb_d;
    float palt =  altHold.alt_p;

    if (tof->range < 0) { // No Sensor Value
        return 0;
    }

//    int error = get_alt_error(tof, target_distance);
//    int climb_rate = constrain(palt * error, -200, +200);// mm/s
//    int current_climb_rate_unfiltered = (tof->l_error - error)*(1000/refreshTime);// mm/ms
//    int current_climb_rate = filterApply(&climb_rate_filter, current_climb_rate_unfiltered);
//    int climb_rate_error = climb_rate - current_climb_rate;
//
//    //printk("%i,%i,%i\n", climb_rate,current_climb_rate,climb_rate_error);
//
//    int16_t ki = 0, kp = 0, kd = 0;
//    kp = constrain(pterm * climb_rate_error, -400, +400);
//
//    tof->integral_e = tof->integral_e + iterm * climb_rate_error;
//    tof->integral_e = constrain( tof->integral_e, 1000, 2000);
//    ki = tof->integral_e;
//
//    float derivative = ((climb_rate_error - tof->l_error) / refreshTime) * 33;
//    float derivativeSum = tof->derivative1 + tof->derivative2 + derivative;
//    tof->derivative2 = tof->derivative1;
//    tof->derivative1 = derivative;
//    float derivativeFiltered = derivativeSum / 3;
//    kd = constrain(dterm * derivativeFiltered, -250, +250);
    //printk("%i,%i,%i,%i \n", (int)derivative, (int)derivativeSum, (int)derivativeFiltered,kd);



    s16_t error = get_alt_error(tof,target_distance);

    tof->integral_e = constrain(tof->integral_e + error, -32000, +32000);

    s16_t derivative = error - tof->l_error;

    s16_t kp = constrain(pterm * error, -400, +400);
    s16_t ki = constrain(iterm * tof->integral_e, -500, +500);
    s16_t kd = constrain(dterm * derivative, -250, +250);

    tof->l_error = error;
    int force = kp + ki + kd;

    //update thr
    //current_thr = constrain(force, 1000, 2000);


    //printk("%i,%i,%i\n", kp,ki,kd);
    //printk("[%i,%i] | [p=%i,d=%i,i=%i] |thr=%i\n", tof->range, error, kp, kd, ki, thrust_alt);
    uint8_t data[8];
    data[0] = (uint8_t)((0x00FF) & kp);
    data[1] = (uint8_t)((0x00FF) & kp >> 8);

    data[2] = (uint8_t)((0x00FF) & ki);
    data[3] = (uint8_t)((0x00FF) & ki >> 8);

    data[4] = (uint8_t)((0x00FF) & kd);
    data[5] = (uint8_t)((0x00FF) & kd >> 8);


    int thr = constrain(chan[RC_THROTTLE] + thrust_alt, 1100, 1800);
    data[6] = (uint8_t)((0x00FF) & thr);
    data[7] = (uint8_t)((0x00FF) & thr >> 8);


    send_debug(data, 8);

    thrust_alt = force;
    return 0;
}

void setAltitudePID(u16_t p, u16_t i, u16_t d, u16_t p_alt)
{
    altHold.climb_p = ((float) p) / 1000;
    altHold.climb_i = ((float) i) / 1000;
    altHold.climb_d = ((float) d) / 1000;
    altHold.alt_p = ((float) p_alt) / 1000;

    printk("\n------------------- \n set pid %d %d %d %d \n-------------------\n", p, i, d,p_alt);
}
void setPushPID(u16_t p, u16_t i, u16_t d)
{
    tof_front.pterm = ((float) p) / 1000;
    tof_front.iterm = ((float) i) / 1000;
    tof_front.dterm = ((float) d) / 1000;
    printk("\n------------------- \n set pid %d %d %d \n-------------------\n", p, i, d);
}

static int init_Controller(struct device *dev)
{
    ARG_UNUSED(dev);
    setAltitudePID(0, 0, 0, 0);
    setPushPID(0,0,0);
    return 0;
}

SYS_INIT( init_Controller, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
