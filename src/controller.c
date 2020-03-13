#include <zephyr.h>
#include <init.h>
#include <math.h>

#include "controller.h"
#include "sensor.h"
#include "globals.h"
#include "streambuf.h"
#include "msp.h"

#define M_PI        3.14159265358979323846
#define HOLD_MM 300

altHoldPid_t altHold;
rc_control_t rcControl;
att_data_t att_data;

flight_mode modus = IDLE;
int16_t headFreeModeHold = 0;

static s16_t force_altitude = 0;
static s16_t force_roll = 0;
static s16_t force_pitch = 0;

// last data from pc
u16_t chan[RC_CHANAL_COUNT];
void resetController()
{
    reset_sensor(&tof_down, HOLD_MM);
    force_altitude = force_roll = force_pitch = 0;
    modus = IDLE;
    for (int i = 0; i < RC_CHANAL_COUNT; i++) {
        chan[i] = 0;
    }
    rcControl.rcdata.throttle = 1000;
    printk("Controller reseted\n");
}

void resetHoldMode()
{
    force_altitude = force_roll = force_pitch = 0;
    reset_sensor(&tof_down, HOLD_MM);
    printk("Reset Hold Mode\n");
}

int pushController(tof_controller_t *tof)
{
    float refreshTime = tof->time_between_reads;
    float pterm = tof->pterm;
    u16_t max_range = tof->iterm * 1000; //range where controller is starting pushing
    float dterm = tof->dterm;

    if (tof->range < 0) { // No Sensor Value
        return 0;
    }

    s16_t error = range_diff_sensor(tof, max_range);
    // p
    s16_t kp = constrain(pterm * error, -500, +500);
    if (kp < 0) {
        kp = 0;
    }
    // d
    float derivative = ((error - tof->l_error) / refreshTime) * 33;
    float derivativeSum = tof->derivative1 + tof->derivative2 + derivative;
    tof->derivative2 = tof->derivative1;
    tof->derivative1 = derivative;
    float derivativeFiltered = derivativeSum / 3;
    s16_t kd = constrain(dterm * derivativeFiltered, -500, +500);
    if (kd < 0) {
        kd = 0;
    }

    int force = kp + kd;
    //printk("[p=%i,d=%i] @ %i\n", kp, kd, max_range);
    if (force < 0) { //only push away
        force = 0;
    }
    tof->l_error = error;

//    uint8_t data[8];
//    data[0] = (uint8_t)((0x00FF) & kp);
//    data[1] = (uint8_t)((0x00FF) & kp >> 8);
//
//    data[2] = (uint8_t)((0x00FF) & kd);
//    data[3] = (uint8_t)((0x00FF) & kd >> 8);
//
////    data[4] = (uint8_t)((0x00FF) & kd);
////    data[5] = (uint8_t)((0x00FF) & kd >> 8);
//    data[4] = 0;
//    data[5] = 0;
//
//    data[6] = (uint8_t)((0x00FF) & force);
//    data[7] = (uint8_t)((0x00FF) & force >> 8);

//    send_debug(data, 8);
    return force;
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

s16_t getAltitudeThrottle(tof_controller_t *tof, u16_t target_distance)
{
    float refreshTime = tof->time_between_reads;
    float pterm = altHold.p;
    float iterm = altHold.i;
    float dterm = altHold.d;

    if (tof->range < 0) { // No Sensor Value
        return 0;
    }
    s16_t error = range_diff_sensor(tof, target_distance);
    // p
    s16_t kp = constrain(pterm * error, -400, +400);
    // i
    tof->integral_e = constrain(tof->integral_e + iterm * error, -500, +500);
    s16_t ki = constrain(tof->integral_e, -500, +500);
    // d
    float derivative = ((error - tof->l_error) / refreshTime) * 33;
    float derivativeSum = tof->derivative1 + tof->derivative2 + derivative;
    tof->derivative2 = tof->derivative1;
    tof->derivative1 = derivative;
    float derivativeFiltered = derivativeSum / 3;
    s16_t kd = constrain(dterm * derivativeFiltered, -250, +250);

    tof->l_error = error;
    return kp + ki + kd;
}

void tick(int att_timeout)
{
    int error = 0;
    if (att_timeout > 200) {
        printk("Timeout attitude Flight Controller\n");
        error = 1;
    }

    if (chan[RC_ARM] < 1600 || att_timeout > 200) { // Hard reset
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
            if (tof_front.isNew) {
                int16_t pushRoll = 0, pushPitch = 0;
                calcPushback(&pushRoll, &pushPitch, &tof_front);
                force_roll = pushRoll;
                force_pitch = pushPitch;
                tof_front.isNew = false;
            }

            if (tof_down.isNew) { //Only update if new data is available
                force_altitude = getAltitudeThrottle(&tof_down, HOLD_MM);
                tof_down.isNew = false;
            }

            // update throttle bases on altitude  hold
            rcControl.rcdata.throttle = constrain(chan[RC_THROTTLE] + force_altitude, 1100, 1800);
            // update roll/pith with push values
            roll = constrain(roll + force_roll, -500, 500);
            pitch = constrain(pitch + force_pitch, -500, 500);

            // set RC Data
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
        } else {
            static s64_t landingTimer = 0;
            s64_t currentTime = k_uptime_get();
            if (currentTime >= landingTimer) {
                landingTimer = currentTime + 50;
                rcControl.rcdata.throttle = constrain(rcControl.rcdata.throttle - 10, chan[RC_THROTTLE], 1800); // decrease to RC_THRUST
            }
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

    set_debug(modus, error);
}
void rc_data_frame_received(sbuf_t *src)
{
    watchdogPC = 0; // copy data
    for (int i = 0; i < RC_CHANAL_COUNT; i++) {
        chan[i] = sbufReadU16(src);
    }
}

void setAltitudePID(u16_t p, u16_t i, u16_t d)
{
    altHold.p = ((float) p) / 1000;
    altHold.i = ((float) i) / 1000;
    altHold.d = ((float) d) / 1000;

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
    setAltitudePID(0, 0, 0);
    setPushPID(0, 0, 0);
    return 0;
}

SYS_INIT( init_Controller, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
