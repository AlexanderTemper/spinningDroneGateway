#include <zephyr.h>
#include <init.h>

#include "controller.h"
#include "globals.h"

altHoldPid_t altHold;
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

u16_t getAltitudeThrottle(u16_t distance, u16_t target_distance)
{
    static u32_t previousC;
    static s16_t last_error;
    static s16_t integral;

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

    integral = constrain(integral + error, -10000, +10000);

    s16_t derivative = error - last_error;

    s16_t kp = constrain(altHold.P * error, -250, +250);
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
    printk("\n------------------- \n set pid %d %d %d \n-------------------\n",p,i,d);
}

static int init_Controller(struct device *dev)
{
    ARG_UNUSED(dev);
    setPID(0, 0, 0);
    return 0;
}

SYS_INIT( init_Controller, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
