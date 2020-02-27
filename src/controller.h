#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include "streambuf.h"

#define RC_CHANAL_COUNT 6
#define ARM_SWITCH 4
#define MODE_SWITCH 5

#define TOF_TIMEOUT_MS 80
#define TOF_POLLING_TIME_MS 2
#define TOF_MAX_RANGE 1500

typedef enum {
    FRONT = 0,
    LEFT,
    RIGHT,
    REAR
}tofDirection;


typedef enum {
    IDLE = 0, ARMED, HOLD, NORMALIZE
} flight_mode;
 extern flight_mode modus;

typedef struct tof_controller_s{
    // pid interna
    int l_error;
    int integral_e;
    float derivative1;
    float derivative2;
    int last_froce;

    //general
    int range;
    s64_t time_last_read;
    //between reads
    s64_t time_between_reads;
    tofDirection direction;
    struct device *dev;
} tof_controller_t;


extern tof_controller_t tof_front;
extern tof_controller_t tof_down;

typedef struct altHoldPid_s {
    float P;
    float I;
    float D;
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

u16_t getEstimatedAltitude(u16_t distance);
u16_t getAltitudeThrottle(u16_t distance, u16_t target_distance);
void setPID(u16_t p, u16_t i, u16_t d);
void resetController();
void rc_data_frame_received(sbuf_t *src);
void tick();

#endif /*CONTROLLER_H_*/
