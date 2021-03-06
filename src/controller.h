#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include "streambuf.h"

#define RC_CHANAL_COUNT 6
#define ARM_SWITCH 4
#define MODE_SWITCH 5




typedef enum {
    IDLE = 0, ARMED, HOLD, TAKEOFF, NORMALIZE
} flight_mode;
 extern flight_mode modus;

typedef struct altHoldPid_s {
    float p;
    float i;
    float d;
} altHoldPid_t;

enum {
    RC_ROLL = 0, RC_PITCH, RC_YAW, RC_THROTTLE, RC_ARM, RC_MODE
};

typedef union {
    struct {
        u16_t roll;
        u16_t pitch;
        u16_t yaw;
        u16_t throttle;
        u16_t arm;
        u16_t mode;
    };
    u16_t raw[6];
} rcdata_t;

typedef union {
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    };
    int16_t raw[3];
} att_data_t;

typedef struct {
    rcdata_t rcdata;
    s64_t last_frame_time;
    s64_t last_valid_frame_time;
    bool is_valid;
} rc_control_t;

extern altHoldPid_t altHold;
extern rc_control_t rcControl;
extern att_data_t att_data;

void setAltitudePID(u16_t p, u16_t i, u16_t d);
void setPushPID(u16_t p, u16_t i, u16_t d);
void resetController();
void rc_data_frame_received(sbuf_t *src);
void tick(int att_timeout);

#endif /*CONTROLLER_H_*/
