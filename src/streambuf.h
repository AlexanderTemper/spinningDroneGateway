#ifndef _STREAMBUF_H_
#define _STREAMBUF_H_

#include <stdbool.h>
#include <stdint.h>

// simple buffer-based serializer/deserializer without implicit size check
// little-endian encoding implemneted now

typedef struct sbuf_s {
    uint8_t *ptr;          // data pointer must be first (sbuff_t* is equivalent to uint8_t **)
    uint8_t *end;
} sbuf_t;

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end);

void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufFill(sbuf_t *dst, uint8_t data, int len);
void sbufWriteData(sbuf_t *dst, const void *data, int len);
bool sbufWriteDataSafe(sbuf_t *dst, const void *data, int len);
void sbufWriteString(sbuf_t *dst, const char *string);
void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string);
void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val);
void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val);

uint8_t sbufReadU8(sbuf_t *src);
uint16_t sbufReadU16(sbuf_t *src);
uint32_t sbufReadU32(sbuf_t *src);
void sbufReadData(const sbuf_t *dst, void *data, int len);

bool sbufReadU8Safe(uint8_t *dst, sbuf_t *src);
bool sbufReadU16Safe(uint16_t *dst, sbuf_t *src);
bool sbufReadU32Safe(uint32_t *dst, sbuf_t *src);
bool sbufReadI8Safe(int8_t *dst, sbuf_t *src);
bool sbufReadI16Safe(int16_t *dst, sbuf_t *src);
bool sbufReadI32Safe(int32_t *dst, sbuf_t *src);
bool sbufReadDataSafe(const sbuf_t *src, void *data, int len);

int sbufBytesRemaining(const sbuf_t *buf);
uint8_t* sbufPtr(sbuf_t *buf);
const uint8_t* sbufConstPtr(const sbuf_t *buf);
void sbufAdvance(sbuf_t *buf, int size);

void sbufSwitchToReader(sbuf_t *buf, uint8_t * base);

#endif
