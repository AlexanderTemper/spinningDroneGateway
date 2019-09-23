#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <sys/ring_buffer.h>

#define RING_BUF_SIZE (64 * 4)


typedef struct ringbuffer_s {
    struct ring_buf rb;
    u8_t buffer[RING_BUF_SIZE];
} ringbuffer_t;

extern ringbuffer_t  PC_rx;
extern ringbuffer_t  PC_tx;
extern ringbuffer_t  FC_rx;
extern ringbuffer_t  FC_tx;

extern uint16_t distance_mm;


#endif /*GLOBALS_H_*/