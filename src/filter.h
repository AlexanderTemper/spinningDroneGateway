#ifndef FILTER_H_
#define FILTER_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;


typedef struct expFilter_s {
    float a;
    float last;
    bool first_messure;
} expFilter_t;

typedef struct filter_s {
    biquadFilter_t *biquadFilter;
    expFilter_t expFilter;
} filter_t;


void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float input);

void expFilterInit(expFilter_t *filter, float a);
void expFilterReset(expFilter_t *filter);
float expFilterApply(expFilter_t *filter,float input);


void filterInit(filter_t *filter);
void filterReset(filter_t *filter);
float filterApply(filter_t *filter,float input);

#endif /*FILTER_H_*/
