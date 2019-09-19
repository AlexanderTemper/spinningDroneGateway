#include <zephyr.h>
#include <init.h>

#include "msp.h"
#include "msp_protocol.h"

mspPort_t PC_msp;
mspPort_t FC_msp;

#include <logging/log.h>
LOG_MODULE_REGISTER( drone_msp, LOG_LEVEL_DBG);

int init_msp(struct device *dev)
{
    ARG_UNUSED(dev);
    PC_msp.rxBuffer = &PC_rx;
    PC_msp.txBuffer = &PC_tx;

    return 0;
}
void mspSerialProcess(mspPort_t *mspPort);
void processMSP(void)
{
    mspSerialProcess(&PC_msp);
    /* if (!ring_buf_is_empty(ringbuf)) {
     char buf[40];
     int len = ring_buf_get(ringbuf, buf, sizeof(buf));
     printk("fromPCtoFC RXBuffer: ");
     for (char e = 0; e < len; e++) {
     printk("0x%X ", buf[e]);
     }
     printk("\n");
     }*/

}
static bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c);
static void mspSerialProcessReceivedCommand(mspPort_t *msp);
static mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply);
void mspSerialProcess(mspPort_t *mspPort)
{
    struct ring_buf *ringbuf = &mspPort->rxBuffer->rb;

    if (!ring_buf_is_empty(ringbuf)) {
        // There are bytes incoming - abort pending request
        mspPort->lastActivityMs = k_uptime_get_32();

        while (!ring_buf_is_empty(ringbuf)) {

            uint8_t c;
            int len = ring_buf_get(ringbuf, &c, 1);
            if (len != 1) {
                LOG_ERR("can not read buffer abort processing");
                return;
            }

            mspSerialProcessReceivedData(mspPort, c);

            /*if (!consumed && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
             mspEvaluateNonMspData(mspPort, c);
             }*/

            if (mspPort->c_state == MSP_COMMAND_RECEIVED) {
                printk("get MSP_COMMAND_RECEIVED\n");
                if (mspPort->packetType == MSP_PACKET_COMMAND) {
                    printk("get MSP_PACKET_COMMAND\n");
                    mspSerialProcessReceivedCommand(mspPort);
                } else if (mspPort->packetType == MSP_PACKET_REPLY) {
                    printk("get MSP_PACKET_COMMAND\n");
                    //mspSerialProcessReceivedReply(mspPort, mspProcessReplyFn);
                }

                mspPort->c_state = MSP_IDLE;
                break; // process one command at a time so as not to block.
            }
        }

        /*if (mspPostProcessFn) {
         waitForSerialPortToFinishTransmitting(mspPort->port);
         mspPostProcessFn(mspPort->port);
         }*/
    }
}
static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}
static uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length)
{
    const uint8_t *p = (const uint8_t *) data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {
        crc = crc8_dvb_s2(crc, *p);
    }
    return crc;
}
static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len)
{
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}
#define JUMBO_FRAME_SIZE_LIMIT 255
static int mspSendFrame(mspPort_t *msp, const uint8_t * hdr, int hdrLen, const uint8_t * data, int dataLen, const uint8_t * crc, int crcLen)
{
    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    const int totalFrameLength = hdrLen + dataLen + crcLen;
    struct ring_buf *ringbuf = &msp->txBuffer->rb;

    if (!ring_buf_is_empty(ringbuf)) {
        size_t gesamt = ring_buf_capacity_get(ringbuf);
        size_t frei = ring_buf_space_get(ringbuf);
        if ((gesamt - frei) < totalFrameLength) {
            return 0;
        }
    }
    // Transmit frame
    int wrote = 0;
    wrote = ring_buf_put(ringbuf, hdr, hdrLen);
    wrote += ring_buf_put(ringbuf, data, dataLen);
    wrote += ring_buf_put(ringbuf, crc, crcLen);
    if (wrote < totalFrameLength) {
        LOG_ERR("Write error msp");
    }
    return totalFrameLength;
}
static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet, mspVersion_e mspVersion)
{
    static const uint8_t mspMagic[MSP_VERSION_COUNT] = MSP_VERSION_MAGIC_INITIALIZER
    ;
    const int dataLen = sbufBytesRemaining(&packet->buf);
    uint8_t hdrBuf[16] = {
            '$',
            mspMagic[mspVersion],
            packet->result == MSP_RESULT_ERROR ? '!' : '>' };
    uint8_t crcBuf[2];
    uint8_t checksum;
    int hdrLen = 3;
    int crcLen = 0;

#define V1_CHECKSUM_STARTPOS 3
    if (mspVersion == MSP_V1) {
        mspHeaderV1_t * hdrV1 = (mspHeaderV1_t *) &hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV1_t);
        hdrV1->cmd = packet->cmd;

        // Add JUMBO-frame header if necessary
        if (dataLen >= JUMBO_FRAME_SIZE_LIMIT) {
            mspHeaderJUMBO_t * hdrJUMBO = (mspHeaderJUMBO_t *) &hdrBuf[hdrLen];
            hdrLen += sizeof(mspHeaderJUMBO_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = dataLen;
        } else {
            hdrV1->size = dataLen;
        }

        // Pre-calculate CRC
        checksum = mspSerialChecksumBuf(0, hdrBuf + V1_CHECKSUM_STARTPOS, hdrLen - V1_CHECKSUM_STARTPOS);
        checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
        crcBuf[crcLen++] = checksum;
    } else if (mspVersion == MSP_V2_OVER_V1) {
        mspHeaderV1_t * hdrV1 = (mspHeaderV1_t *) &hdrBuf[hdrLen];

        hdrLen += sizeof(mspHeaderV1_t);

        mspHeaderV2_t * hdrV2 = (mspHeaderV2_t *) &hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV2_t);

        const int v1PayloadSize = sizeof(mspHeaderV2_t) + dataLen + 1;  // MSPv2 header + data payload + MSPv2 checksum
        hdrV1->cmd = MSP_V2_FRAME_ID;

        // Add JUMBO-frame header if necessary
        if (v1PayloadSize >= JUMBO_FRAME_SIZE_LIMIT) {
            mspHeaderJUMBO_t * hdrJUMBO = (mspHeaderJUMBO_t *) &hdrBuf[hdrLen];
            hdrLen += sizeof(mspHeaderJUMBO_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = v1PayloadSize;
        } else {
            hdrV1->size = v1PayloadSize;
        }

        // Fill V2 header
        hdrV2->flags = packet->flags;
        hdrV2->cmd = packet->cmd;
        hdrV2->size = dataLen;

        // V2 CRC: only V2 header + data payload
        checksum = crc8_dvb_s2_update(0, (uint8_t *) hdrV2, sizeof(mspHeaderV2_t));
        checksum = crc8_dvb_s2_update(checksum, sbufPtr(&packet->buf), dataLen);
        crcBuf[crcLen++] = checksum;

        // V1 CRC: All headers + data payload + V2 CRC byte
        checksum = mspSerialChecksumBuf(0, hdrBuf + V1_CHECKSUM_STARTPOS, hdrLen - V1_CHECKSUM_STARTPOS);
        checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
        checksum = mspSerialChecksumBuf(checksum, crcBuf, crcLen);
        crcBuf[crcLen++] = checksum;
    } else if (mspVersion == MSP_V2_NATIVE) {
        mspHeaderV2_t * hdrV2 = (mspHeaderV2_t *) &hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV2_t);

        hdrV2->flags = packet->flags;
        hdrV2->cmd = packet->cmd;
        hdrV2->size = dataLen;

        checksum = crc8_dvb_s2_update(0, (uint8_t *) hdrV2, sizeof(mspHeaderV2_t));
        checksum = crc8_dvb_s2_update(checksum, sbufPtr(&packet->buf), dataLen);
        crcBuf[crcLen++] = checksum;
    } else {
        // Shouldn't get here
        return 0;
    }

// Send the frame
    return mspSendFrame(msp, hdrBuf, hdrLen, sbufPtr(&packet->buf), dataLen, crcBuf, crcLen);
}

static void mspSerialProcessReceivedCommand(mspPort_t *msp)
{
    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];
    mspPacket_t reply = {
            .buf = {
                    .ptr = outBuf,
                    .end = ARRAYEND(outBuf), },
            .cmd = -1,
            .flags = 0,
            .result = 0,
            .direction = MSP_DIRECTION_REPLY, };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = {
            .buf = {
                    .ptr = msp->inBuf,
                    .end = msp->inBuf + msp->dataSize, },
            .cmd = msp->cmdMSP,
            .flags = msp->cmdFlags,
            .result = 0,
            .direction = MSP_DIRECTION_REQUEST, };

    const mspResult_e status = mspFcProcessCommand(&command, &reply);
    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, &reply, msp->mspVersion);
    }

}

static bool mspCommonProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst);
static mspResult_e mspFcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply)
{
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    //sbuf_t *src = &cmd->buf;
    const uint8_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspCommonProcessOutCommand(cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    } /*else if (mspProcessOutCommand(cmdMSP, dst)) {
     ret = MSP_RESULT_ACK;
     } else if ((ret = mspFcProcessOutCommandWithArg(cmdMSP, src, dst, mspPostProcessFn)) != MSP_RESULT_CMD_UNKNOWN) {
     } else {
     ret = mspCommonProcessInCommand(cmdMSP, src, mspPostProcessFn);
     }*/
    reply->result = ret;
    return ret;
}
static bool mspCommonProcessOutCommand(uint8_t cmdMSP, sbuf_t *dst)
{
    switch (cmdMSP) {
    case MSP_NAME:
        sbufWriteU8(dst, 61);
        sbufWriteU8(dst, 62);
        sbufWriteU8(dst, 63);
        printk("get Name request\n");
        break;
    }
    return true;
}
static bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    switch (mspPort->c_state) {
    default:
    case MSP_IDLE:      // Waiting for '$' character
        if (c == '$') {
            mspPort->c_state = MSP_HEADER_START;
        } else {
            return false;
        }
        break;

    case MSP_HEADER_START:  // Waiting for 'M' (MSPv1 / MSPv2_over_v1) or 'X' (MSPv2 native)
        mspPort->offset = 0;
        mspPort->checksum1 = 0;
        mspPort->checksum2 = 0;
        switch (c) {
        case 'M':
            mspPort->c_state = MSP_HEADER_M;
            mspPort->mspVersion = MSP_V1;
            break;
        case 'X':
            mspPort->c_state = MSP_HEADER_X;
            mspPort->mspVersion = MSP_V2_NATIVE;
            break;
        default:
            mspPort->c_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_M:      // Waiting for '<' / '>'
        mspPort->c_state = MSP_HEADER_V1;
        switch (c) {
        case '<':
            mspPort->packetType = MSP_PACKET_COMMAND;
            break;
        case '>':
            mspPort->packetType = MSP_PACKET_REPLY;
            break;
        default:
            mspPort->c_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_X:
        mspPort->c_state = MSP_HEADER_V2_NATIVE;
        switch (c) {
        case '<':
            mspPort->packetType = MSP_PACKET_COMMAND;
            break;
        case '>':
            mspPort->packetType = MSP_PACKET_REPLY;
            break;
        default:
            mspPort->c_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum1 ^= c;
        if (mspPort->offset == sizeof(mspHeaderV1_t)) {
            mspHeaderV1_t * hdr = (mspHeaderV1_t *) &mspPort->inBuf[0];
            // Check incoming buffer size limit
            if (hdr->size > MSP_PORT_INBUF_SIZE) {
                mspPort->c_state = MSP_IDLE;
            } else if (hdr->cmd == MSP_V2_FRAME_ID) {
                // MSPv1 payload must be big enough to hold V2 header + extra checksum
                if (hdr->size >= sizeof(mspHeaderV2_t) + 1) {
                    mspPort->mspVersion = MSP_V2_OVER_V1;
                    mspPort->c_state = MSP_HEADER_V2_OVER_V1;
                } else {
                    mspPort->c_state = MSP_IDLE;
                }
            } else {
                mspPort->dataSize = hdr->size;
                mspPort->cmdMSP = hdr->cmd;
                mspPort->cmdFlags = 0;
                mspPort->offset = 0;                // re-use buffer
                mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }
        }
        break;

    case MSP_PAYLOAD_V1:
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum1 ^= c;
        if (mspPort->offset == mspPort->dataSize) {
            mspPort->c_state = MSP_CHECKSUM_V1;
        }
        break;

    case MSP_CHECKSUM_V1:
        if (mspPort->checksum1 == c) {
            printk("crc passt");
            mspPort->c_state = MSP_COMMAND_RECEIVED;
        } else {
            printk("crc fehler soll %X ist %X \n",mspPort->checksum1,c);
            mspPort->c_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_OVER_V1:     // V2 header is part of V1 payload - we need to calculate both checksums now
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum1 ^= c;
        mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
        if (mspPort->offset == (sizeof(mspHeaderV2_t) + sizeof(mspHeaderV1_t))) {
            mspHeaderV2_t * hdrv2 = (mspHeaderV2_t *) &mspPort->inBuf[sizeof(mspHeaderV1_t)];
            mspPort->dataSize = hdrv2->size;
            mspPort->cmdMSP = hdrv2->cmd;
            mspPort->cmdFlags = hdrv2->flags;
            mspPort->offset = 0;                // re-use buffer
            mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V2_OVER_V1 : MSP_CHECKSUM_V2_OVER_V1;
        }
        break;

    case MSP_PAYLOAD_V2_OVER_V1:
        mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
        mspPort->checksum1 ^= c;
        mspPort->inBuf[mspPort->offset++] = c;

        if (mspPort->offset == mspPort->dataSize) {
            mspPort->c_state = MSP_CHECKSUM_V2_OVER_V1;
        }
        break;

    case MSP_CHECKSUM_V2_OVER_V1:
        mspPort->checksum1 ^= c;
        if (mspPort->checksum2 == c) {
            mspPort->c_state = MSP_CHECKSUM_V1; // Checksum 2 correct - verify v1 checksum
        } else {
            mspPort->c_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_NATIVE:
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
        if (mspPort->offset == sizeof(mspHeaderV2_t)) {
            mspHeaderV2_t * hdrv2 = (mspHeaderV2_t *) &mspPort->inBuf[0];
            mspPort->dataSize = hdrv2->size;
            mspPort->cmdMSP = hdrv2->cmd;
            mspPort->cmdFlags = hdrv2->flags;
            mspPort->offset = 0;                // re-use buffer
            mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_PAYLOAD_V2_NATIVE:
        mspPort->checksum2 = crc8_dvb_s2(mspPort->checksum2, c);
        mspPort->inBuf[mspPort->offset++] = c;

        if (mspPort->offset == mspPort->dataSize) {
            mspPort->c_state = MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_CHECKSUM_V2_NATIVE:
        if (mspPort->checksum2 == c) {
            mspPort->c_state = MSP_COMMAND_RECEIVED;
        } else {
            mspPort->c_state = MSP_IDLE;
        }
        break;
    }

    return true;
}

SYS_INIT( init_msp, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
