#ifndef CONTROLLER_H_
#define CONTROLLER_H_

typedef struct altHoldPid_s {
    float P;
    float I;
    float D;
} altHoldPid_t;

extern altHoldPid_t altHold;
u16_t getEstimatedAltitude(u16_t distance);
u16_t getAltitudeThrottle(u16_t distance, u16_t target_distance);
void setPID(u16_t p, u16_t i, u16_t d);

#endif /*CONTROLLER_H_*/
