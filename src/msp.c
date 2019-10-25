#include <zephyr.h>
#include <init.h>

#include "msp.h"
#include "msp_protocol.h"
#include "uart.h"
#include "streambuf.h"
#include "controller.h"

mspPort_t PC_msp;
mspPort_t FC_msp;

#include <logging/log.h>
LOG_MODULE_REGISTER( drone_msp, LOG_LEVEL_DBG);

int init_msp(struct device *dev)
{
    ARG_UNUSED(dev);
    PC_msp.rxBuffer = &PC_rx;
    PC_msp.txBuffer = &PC_tx;
    PC_msp.passthroughPort = &FC_msp;
    PC_msp.portType = MSP_PORT_PC;

    FC_msp.rxBuffer = &FC_rx;
    FC_msp.txBuffer = &FC_tx;
    FC_msp.passthroughPort = &PC_msp;
    FC_msp.portType = MSP_PORT_FC;

    return 0;
}

static mspGatewayPacketType_e msp_packet_lookup_table(mspPort_t *mspPort)
{
    if (mspPort->portType == MSP_PORT_PC) {
        switch (mspPort->cmdMSP) {
        case MSP_SET_RAW_RC:
            return MSP_GATEWAY_PACKET_MODIFY_AND_REPLY;
        case MSP_SET_PID:
            return MSP_GATEWAY_PACKET_REPLY;
        case MSP_NAME:
            return MSP_GATEWAY_PACKET_REPLY;
        }
    }

    if (mspPort->portType == MSP_PORT_FC) {
        switch (mspPort->cmdMSP) {
        case MSP_SET_RAW_RC:
            return MSP_GATEWAY_PACKET_CONSUME;
        }
    }
    return MSP_GATEWAY_PACKET_PASSTHROUGH;
}

static bool write_data_to_passthroughPort(mspPort_t *mspPort, u8_t *data, u32_t size)
{
    //get the buffer where the data should be placed
    struct ring_buf *ringbuf = &mspPort->passthroughPort->txBuffer->rb;

    // data has to fit in buffer
    if (!ring_buf_is_empty(ringbuf)) {
        size_t gesamt = ring_buf_capacity_get(ringbuf);
        size_t frei = ring_buf_space_get(ringbuf);
        if ((gesamt - frei) < size) {
            return false;
        }
    }
    u32_t wrote = ring_buf_put(ringbuf, data, size);
    if (wrote < size) {
        // TODO delete all written data out of ring buffer
        return false;
    }

    if (mspPort->passthroughPort->portType == MSP_PORT_FC) {
        uart_start_tx();
    }

    return true;
}

static bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    uint8_t data[5];
    switch (mspPort->c_state) {
    default:
    case MSP_IDLE:      // Waiting for '$' character
        if (c == '$') {
            mspPort->c_state = MSP_HEADER_START;
        } else {
            write_data_to_passthroughPort(mspPort, &c, 1);
            return false;
        }
        break;

    case MSP_HEADER_START:  // Waiting for 'M' (MSPv1 / MSPv2_over_v1) or 'X' (MSPv2 native)
        mspPort->offset = 0;
        mspPort->checksum1 = 0;
        switch (c) {
        case 'M':
            mspPort->c_state = MSP_HEADER_M;
            mspPort->mspVersion = MSP_V1;
            break;
        default:
            data[0] = '$';
            data[1] = c;
            write_data_to_passthroughPort(mspPort, data, 2);
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
            data[0] = '$';
            data[1] = 'M';
            data[2] = c;
            write_data_to_passthroughPort(mspPort, data, 3);
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
            } else {
                mspPort->dataSize = hdr->size;
                mspPort->cmdMSP = hdr->cmd;
                mspPort->offset = 0;                // re-use buffer
                mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }

            mspPort->gatewaypacketType = msp_packet_lookup_table(mspPort);
            switch (mspPort->gatewaypacketType) {
            case MSP_GATEWAY_PACKET_REPLY:
                //printk("MSP_GATEWAY_PACKET_REPLY\n");
                //mspPort->packetType = MSP_PACKET_REPLY;
                break;
            case MSP_GATEWAY_PACKET_MODIFY:
                //printk("MSP_GATEWAY_PACKET_MODIFY\n");
                //mspPort->packetType = MSP_PACKET_COMMAND;
                break;
            case MSP_GATEWAY_PACKET_CONSUME:
                //printk("MSP_GATEWAY_PACKET_CONSUME\n");
                break;
            case MSP_GATEWAY_PACKET_MODIFY_AND_REPLY:
                break;
            default:
            case MSP_GATEWAY_PACKET_PASSTHROUGH:
                printk("MSP_GATEWAY_PACKET_PASSTHROUGH\n");
                data[0] = '$';
                data[1] = 'M';
                data[2] = mspPort->packetType == MSP_PACKET_COMMAND ? '<' : '>';
                data[3] = mspPort->dataSize;
                data[4] = mspPort->cmdMSP;
                write_data_to_passthroughPort(mspPort, data, 5);
                mspPort->c_state = MSP_IDLE;
                break;
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
            mspPort->c_state = MSP_COMMAND_RECEIVED;
        } else {
            printk("crc fehler soll %X ist %X \n", mspPort->checksum1, c);
            mspPort->c_state = MSP_IDLE;
        }
        break;
    }

    return true;
}

static int mspSendFrame(mspPort_t *msp, const uint8_t * hdr, int hdrLen, const uint8_t * data, int dataLen, const uint8_t * crc, int crcLen)
{
    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //  b) Response fits into TX buffer
    const int totalFrameLength = hdrLen + dataLen + crcLen;
    struct ring_buf *ringbuf = &msp->txBuffer->rb;

    if (!ring_buf_is_empty(ringbuf)) {
        if (ring_buf_space_get(ringbuf) < totalFrameLength) {
            printk("msp send buffer has not enough space left need %d get %u \n", totalFrameLength, ring_buf_space_get(ringbuf));
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
        return 0;
    }

    if (msp->portType == MSP_PORT_FC) {
        uart_start_tx();
    }
    return totalFrameLength;
}
static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len)
{
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}
static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet)
{
    const int dataLen = sbufBytesRemaining(&packet->buf);
    uint8_t hdrBuf[16] = {
            '$',
            'M',
            packet->direction };
    uint8_t crcBuf[2];
    uint8_t checksum;
    int hdrLen = 3;
    int crcLen = 0;

#define V1_CHECKSUM_STARTPOS 3
    mspHeaderV1_t * hdrV1 = (mspHeaderV1_t *) &hdrBuf[hdrLen];
    hdrLen += sizeof(mspHeaderV1_t);
    hdrV1->cmd = packet->cmd;
    hdrV1->size = dataLen;
    // Pre-calculate CRC
    checksum = mspSerialChecksumBuf(0, hdrBuf + V1_CHECKSUM_STARTPOS, hdrLen - V1_CHECKSUM_STARTPOS);
    checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
    crcBuf[crcLen++] = checksum;

    // Send the frame
    return mspSendFrame(msp, hdrBuf, hdrLen, sbufPtr(&packet->buf), dataLen, crcBuf, crcLen);
}

#define CHAN 5
#define MODE 5
#define THR 4
union rcdata_u {
    struct {
        u16_t pitch;
        u16_t roll;
        u16_t yaw;
        u16_t thrust;
        u16_t mode;
    };
    u16_t raw[5];
} rcdata;

static bool modify_data(mspPort_t *mspPort, sbuf_t *dst)
{
    sbuf_t srcBuffer;

    sbuf_t *src = sbufInit(&srcBuffer, mspPort->inBuf, mspPort->inBuf + mspPort->dataSize);

    switch (mspPort->cmdMSP) {
    case MSP_SET_RAW_RC:
        ;
        //printk("MSP_SET_RAW_RC request data ");
        uint8_t channelCount = mspPort->dataSize / sizeof(uint16_t);

        u16_t chan[CHAN];
        for (int i = 0; i < channelCount && i < CHAN; i++) {
            chan[i] = sbufReadU16(src);
        }

        for (int i = 0; i < CHAN; i++) {
            uint16_t data = chan[i];

            if (chan[MODE - 1] > 1600 && i == (THR - 1)) { // altHoldMode
                u16_t t = constrain(data + thrust_alt, 1100, 1800);
                rcdata.thrust = t;
                //printk("[%d %i %d]", data, thrust_alt, t);
                sbufWriteU16(dst, t);
            } else {
                rcdata.raw[i] = data;
                //printk("%d ", data);
                sbufWriteU16(dst, data);
            }
        }

        //printk("\n");
        return true;
    }
    printk("modify_data connot find cmd\n");
    return false;
}

static bool reply_data(mspPort_t *mspPort, sbuf_t *dst)
{
    sbuf_t srcBuffer;

    sbuf_t *src = sbufInit(&srcBuffer, mspPort->inBuf, mspPort->inBuf + mspPort->dataSize);
    // Messages Replied to pc
    if (mspPort->portType == MSP_PORT_PC) {
        switch (mspPort->cmdMSP) {
        case MSP_SET_PID:
            ;
            uint8_t channelCount = mspPort->dataSize / sizeof(uint16_t);
            if (channelCount == 3) {
                setPID(sbufReadU16(src), sbufReadU16(src), sbufReadU16(src));
                sbufWriteU16(dst, (u16_t) (altHold.P * 1000));
                sbufWriteU16(dst, (u16_t) (altHold.I * 1000));
                sbufWriteU16(dst, (u16_t) (altHold.D * 1000));
            }
        case MSP_SET_RAW_RC:
            for (int i = 0; i < CHAN; i++) {
                sbufWriteU16(dst, rcdata.raw[i]);
            }
            return true;
        case MSP_NAME:
            sbufWriteU8(dst, 'X');
            sbufWriteU8(dst, 'E');
            sbufWriteU8(dst, 'N');
            sbufWriteU8(dst, 'O');
            sbufWriteU8(dst, 'N');
            return true;
        }
    }
    printk("modify_data connot find cmd\n");
    return false;
}

static void mspModify(mspPort_t *mspPort)
{
    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];
    mspPacket_t modify = {
            .buf = {
                    .ptr = outBuf,
                    .end = ARRAYEND(outBuf), },
            .cmd = mspPort->cmdMSP,
            .direction = mspPort->packetType == MSP_PACKET_COMMAND ? '<' : '>', };
    uint8_t *outBufHead = modify.buf.ptr;
    sbuf_t *dst = &modify.buf;
    if (!modify_data(mspPort, dst)) {
        return;
    }

    sbufSwitchToReader(&modify.buf, outBufHead); // change streambuf direction
    mspSerialEncode(mspPort->passthroughPort, &modify); // put the modify_data on passthroughPort
}

static void mspReply(mspPort_t *mspPort)
{
    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];
    mspPacket_t reply = {
            .buf = {
                    .ptr = outBuf,
                    .end = ARRAYEND(outBuf), },
            .cmd = mspPort->cmdMSP,
            .direction = '>', };
    uint8_t *outBufHead = reply.buf.ptr;
    sbuf_t *dst = &reply.buf;
    if (!reply_data(mspPort, dst)) {
        return;
    }
    sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
    mspSerialEncode(mspPort, &reply);
}

static bool mspConsume(mspPort_t *mspPort)
{
    switch (mspPort->cmdMSP) {
    case MSP_SET_RAW_RC:
        return true;
    }
    return false;
}

static void mspSerialProcess(mspPort_t *mspPort)
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
                //printk("get MSP_COMMAND_RECEIVED\n");
                if (mspPort->gatewaypacketType == MSP_GATEWAY_PACKET_MODIFY) {
                    mspModify(mspPort);
                } else if (mspPort->gatewaypacketType == MSP_GATEWAY_PACKET_REPLY) {
                    mspReply(mspPort);
                } else if (mspPort->gatewaypacketType == MSP_GATEWAY_PACKET_CONSUME) {
                    mspConsume(mspPort);
                } else if (mspPort->gatewaypacketType == MSP_GATEWAY_PACKET_MODIFY_AND_REPLY) {
                    mspModify(mspPort);
                    mspReply(mspPort);
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
void processMSP(void)
{
    mspSerialProcess(&PC_msp);
    mspSerialProcess(&FC_msp);
}

SYS_INIT( init_msp, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
