#include <zephyr.h>
#include <init.h>
#include <math.h>

#include "controller.h"
#include "globals.h"
#include "streambuf.h"

#define M_PI        3.14159265358979323846

altHoldPid_t altHold;
rc_control_t rcControl;
att_data_t att_data;

tof_controller_t tof_front;
tof_controller_t tof_down;

flight_mode modus = IDLE;
int16_t headFreeModeHold = 0;

u32_t microsDiff(u32_t previousC, u32_t currentC)
{
    return SYS_CLOCK_HW_CYCLES_TO_NS(currentC - previousC) / 1000;
}

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
    printk("Controller reseted\n");
}

void resetHoldMode()
{
    reset_tof(&tof_down);
    thrust_alt = 0;
    printk("Reset Hold Mode\n");
}

int pushController(tof_controller_t *tof)
{
    float refreshTime = tof->time_between_reads;
    float pterm = tof->pterm;
    float iterm = tof->iterm;
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
            s64_t currentTime = k_uptime_get_32();
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

u16_t getAltitudeThrottle(tof_controller_t *tof, u16_t target_distance)
{

    float refreshTime = tof->time_between_reads;
    float pterm = altHold.P;
    float iterm = altHold.I;
    float dterm = altHold.D;
    int offsetSensor = 20; //offset sensor is away from collison

    if (tof->range < 0) { // No Sensor Value
        return 0;
    }

    //printk("%i |took %lld ms\n",tof->range,tof->time_between_reads);

    int error = target_distance - (tof->range - offsetSensor);

    int ki = 0, kp = 0, kd = 0;
    kp = constrain(pterm * error, -400, +400);

    tof->integral_e = constrain(tof->integral_e + error, -32000, +32000);
    ki = constrain(iterm * tof->integral_e, -500, +500);

    float derivative = ((error - tof->l_error) / refreshTime) * 33;
    float derivativeSum = tof->derivative1 + tof->derivative2 + derivative;
    tof->derivative2 = tof->derivative1;
    tof->derivative1 = derivative;
    float derivativeFiltered = derivativeSum / 3;
    kd = constrain(dterm * derivativeFiltered, -250, +250);
    //printk("%i,%i,%i,%i \n", (int)derivative, (int)derivativeSum, (int)derivativeFiltered,kd);

    tof->l_error = error;
    thrust_alt = kp + ki + kd;


    //printk("[%i,%i] | [p=%i,d=%i,i=%i] |thr=%i\n", tof->range, error, kp, kd, ki, thrust_alt);
    return 0;
}

void setPID(u16_t p, u16_t i, u16_t d)
{
    altHold.P = ((float) p) / 1000;
    altHold.I = ((float) i) / 1000;
    altHold.D = ((float) d) / 1000;
    printk("\n------------------- \n set pid %d %d %d \n-------------------\n", p, i, d);
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
    setPID(0, 0, 0);
    return 0;
}

SYS_INIT( init_Controller, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
