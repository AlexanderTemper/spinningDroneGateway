#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <sys/ring_buffer.h>

#define RING_BUF_SIZE (64 * 4)
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/

typedef struct ringbuffer_s {
    struct ring_buf rb;
    u8_t buffer[RING_BUF_SIZE];
} ringbuffer_t;



extern ringbuffer_t  PC_rx;
extern ringbuffer_t  PC_tx;
extern ringbuffer_t  FC_rx;
extern ringbuffer_t  FC_tx;

extern u16_t thrust_alt;
extern u8_t watchdogPC;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#endif /*GLOBALS_H_*/
