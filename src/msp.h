#ifndef MSP_H_
#define MSP_H_

#include "globals.h"
#include "streambuf.h"

#define MSP_PORT_INBUF_SIZE 192
#define MSP_PORT_OUTBUF_SIZE 256

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])

typedef enum {
    MSP_V1          = 0,
    MSP_V2_OVER_V1  = 1,
    MSP_V2_NATIVE   = 2,
    MSP_VERSION_COUNT
} mspVersion_e;

typedef struct mspPacket_s {
    sbuf_t buf;
    int16_t cmd;
    uint8_t direction;
} mspPacket_t;

#define MSP_VERSION_MAGIC_INITIALIZER { 'M', 'M', 'X' }

// return positive for ACK, negative on error, zero for no reply
typedef enum {
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0,
    MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
} mspResult_e;

typedef enum {
    MSP_DIRECTION_REPLY = 0,
    MSP_DIRECTION_REQUEST = 1
} mspDirection_e;

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,

    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,

    MSP_COMMAND_RECEIVED
} mspState_e;

typedef enum {
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY,
} mspPacketType_e;

typedef enum {
    MSP_GATEWAY_PACKET_REPLY,
    MSP_GATEWAY_PACKET_MODIFY,
    MSP_GATEWAY_PACKET_MODIFY_AND_REPLY,
    MSP_GATEWAY_PACKET_PASSTHROUGH,
    MSP_GATEWAY_PACKET_CONSUME,
} mspGatewayPacketType_e;

typedef enum {
    MSP_PORT_PC,
    MSP_PORT_FC
} mspPortType_e;

typedef struct __attribute__((packed)) {
    uint8_t size;
    uint8_t cmd;
} mspHeaderV1_t;

typedef struct mspPort_s mspPort_t;
struct mspPort_s {
    mspState_e c_state;
    mspPortType_e portType;
    u32_t lastActivityMs;
    uint16_t cmdMSP;
    ringbuffer_t *rxBuffer;
    ringbuffer_t *txBuffer;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    uint_fast16_t offset;
    uint_fast16_t dataSize;
    uint8_t checksum1;
    mspVersion_e mspVersion;
    mspPacketType_e packetType;
    mspGatewayPacketType_e gatewaypacketType;
    mspPort_t *passthroughPort;
};

extern mspPort_t PC_msp;
extern mspPort_t FC_msp;

#define MSP_FC_VERSION                  3    //out message


void processMSP(void);
void fetchAttitude(void);
#endif /*MSP_H_*/
