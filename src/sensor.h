#ifndef SENSOR_H_
#define SENSOR_H_

#include "filter.h"
typedef enum {
    FRONT = 0, LEFT, RIGHT, REAR
} tofDirection;

typedef struct tof_controller_s {
    struct device *dev;
    tofDirection direction;
    int range_offset;
    filter_t filter;

    // pid
    int l_error;
    float integral_e;
    float derivative1;
    float derivative2;
    float pterm;
    float iterm;
    float dterm;

    int range;
    bool isNew;
    s64_t time_last_read;
    s64_t time_between_reads;



} tof_controller_t;

extern tof_controller_t tof_front;
extern tof_controller_t tof_down;

void init_sensor(tof_controller_t *sensor, struct device *dev, int range_offset);
void fetch_distance_sensor(tof_controller_t *sensor);
void reset_sensor(tof_controller_t *tof,u16_t target_distance);
int range_diff_sensor(tof_controller_t *tof, u16_t target_distance);

#endif /*SENSOR_H_*/
