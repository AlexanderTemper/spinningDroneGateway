#ifndef CONTROLLER_H_
#define CONTROLLER_H_

typedef struct altHoldPid_s {
    float P;
    float I;
    float D;
} altHoldPid_t;

u16_t getEstimatedAltitude(u16_t distance);
u16_t getAltitudeThrottle(u16_t distance, u16_t target_distance);


#endif /*CONTROLLER_H_*/
