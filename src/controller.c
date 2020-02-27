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
u32_t previousC;
s16_t last_error;
s16_t integral;

flight_mode modus = IDLE;
int16_t headFreeModeHold = 0;

u32_t microsDiff(u32_t previousC, u32_t currentC)
{
    return SYS_CLOCK_HW_CYCLES_TO_NS(currentC - previousC) / 1000;
}

#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)
#define AccDeadband 5 // in mm
u16_t getEstimatedAltitude(u16_t distance)
{
    static float last_distance;
    last_distance = last_distance + 0.3f * ((float) distance - last_distance);
    //printk("%i,%i,", distance,(int)last_distance);
    return (u16_t) last_distance;
}
// last data from pc
u16_t chan[RC_CHANAL_COUNT];
void resetController()
{
    integral = 0;
    thrust_alt = 0;
    modus = IDLE;
    for (int i = 0; i < RC_CHANAL_COUNT; i++) {
        chan[i] = 0;
    }
    printk("Controller reseted\n");
}

void resetHoldMode()
{
    integral = 0;
    thrust_alt = 0;
    printk("Reset Hold Mode\n");
}

int pushController(tof_controller_t *tof)
{

    return 0;

    //printk("%i |took %u ms\n",tof->range,tof->time);
//    float cycleTime = tof->time;
//    float pterm = 0.1;
//    float dterm = 0;
//    int offsetSensor = 0; //offset sensor is away from collison
//
//    if (tof->range < 0) { // No Sensor Value
//        return 0;
//    }
//
//    if (tof->time == 0) { // return old force if sensor data ist not available
//        //printk("no new data available get old data %i , %i\n", tof->range, tof->last_froce);
//        return tof->last_froce;
//    }
//
//    int error = 1000 - (tof->range - offsetSensor);
//    if (error < 0) { // we are not in danger zone
//        return 0;
//    }
//
//    int ki = 0, kp = 0, kd = 0;
//    kp = constrain(pterm * error, -500, +500);
//
//    float derivative = ((error - tof->l_error) / cycleTime) * 4000;
//    float derivativeSum = tof->derivative1 + tof->derivative2 + derivative;
//    tof->derivative2 = tof->derivative1;
//    tof->derivative1 = derivative;
//    float derivativeFiltered = derivativeSum / 3;
//    kd = constrain(dterm * derivativeFiltered, -50, +50);
//
//    tof->l_error = error;
//    int force = kp + ki + kd;
//
//    printk("error %i , p %i d, %i = %i\n", error, kp, kd, force);
//    tof->time = 0; // Reset time and save the force (so this can called faster then new data arrives
//    if (force < 0) {
//        return 0;
//    }
//    tof->last_froce = force;
//    return tof->last_froce;
}

void calcPushback(int16_t *roll, int16_t *pitch, tof_controller_t *tof)
{
    //ROS_INFO("bevore %i [%i,%i,%i] ",heading - headFreeModeHold, rcCommand[THROTTLE],*roll,*pitch);
    float radDiff = (att_data.yaw - headFreeModeHold) * M_PI / 180.0f;
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);

    // TODO Calc it in Drone Frame not in Earth Frame
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
            int16_t pushRoll, pushPitch;
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

u16_t getAltitudeThrottle(u16_t distance, u16_t target_distance)
{

    u32_t currentC = k_cycle_get_32();
    u32_t dTime;

    dTime = microsDiff(previousC, currentC);
    if (dTime < UPDATE_INTERVAL) {
        return 0;
    }
    previousC = currentC;

    // calculate sonar altitude only if the sonar is facing downwards(<25deg) todo
    //    if (tiltAngle > 250)
    //        sonarAlt = -1;
    //    else
    //        sonarAlt = sonarAlt * (900.0f - tiltAngle) / 900.0f;

    //float accAltFilter = ((float) (distance - last_distance_filter)) / (dTime * 1e-5f);  // acc in cm/s

    s16_t error = target_distance - distance;

    integral = constrain(integral + error, -32000, +32000);

    s16_t derivative = error - last_error;

    s16_t kp = constrain(altHold.P * error, -400, +400);
    s16_t ki = constrain(altHold.I * integral, -500, +500);
    s16_t kd = constrain(altHold.D * derivative, -250, +250);

    thrust_alt = kp + ki + kd;
    //u16_t p = error * altHold >> 4

    // TODO Deadband
    //printk("%i,%i,%i  %i %i %i %i\n", kp, ki, kd, thrust_alt, error, derivative, integral);
    last_error = error;
    return 0;
}

void setPID(u16_t p, u16_t i, u16_t d)
{
    altHold.P = ((float) p) / 1000;
    altHold.I = ((float) i) / 1000;
    altHold.D = ((float) d) / 1000;
    printk("\n------------------- \n set pid %d %d %d \n-------------------\n", p, i, d);
}

static int init_Controller(struct device *dev)
{
    ARG_UNUSED(dev);
    setPID(0, 0, 0);
    return 0;
}

SYS_INIT( init_Controller, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
