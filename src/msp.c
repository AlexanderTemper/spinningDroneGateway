#include <zephyr.h>
#include <init.h>

#include "msp.h"
#include "msp_protocol.h"
#include "uart.h"

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
    switch (mspPort->cmdMSP) {
    case MSP_SET_RAW_RC:
        return MSP_GATEWAY_PACKET_MODIFY;

    /*case MSP_NAME:
        return MSP_GATEWAY_PACKET_REPLY;
*/
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

    if(mspPort->passthroughPort->portType == MSP_PORT_FC){
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
                mspPort->cmdFlags = 0;
                mspPort->offset = 0;                // re-use buffer
                mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }

            mspGatewayPacketType_e gatewayPaketType = msp_packet_lookup_table(mspPort);
            switch (gatewayPaketType) {
            case MSP_GATEWAY_PACKET_REPLY:
                printk("MSP_GATEWAY_PACKET_REPLY\n");
                mspPort->packetType = MSP_PACKET_COMMAND;
                break;
            case MSP_GATEWAY_PACKET_MODIFY:
                printk("MSP_GATEWAY_PACKET_MODIFY\n");
                mspPort->packetType = MSP_PACKET_REPLY;
                break;
            default:
            case MSP_GATEWAY_PACKET_PASSTHROUGH:
                //printk("MSP_GATEWAY_PACKET_PASSTHROUGH\n");
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
                if (mspPort->packetType == MSP_PACKET_COMMAND) {
                    //printk("get MSP_PACKET_COMMAND\n");
                    //mspSerialProcessReceivedCommand(mspPort);
                } else if (mspPort->packetType == MSP_PACKET_REPLY) {
                    //printk("get MSP_PACKET_REPLY\n");
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
void processMSP(void)
{
    mspSerialProcess(&PC_msp);
    mspSerialProcess(&FC_msp);
}

SYS_INIT( init_msp, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
