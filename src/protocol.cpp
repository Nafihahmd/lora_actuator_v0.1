#include "protocol.h"

/********************************************
 * Global Variables
 *********************************************/

/********************************************
 * Protocol APIs
 *********************************************/
/**
 * Calculates the CRC16 checksum for the given data.
 *
 * @param data Pointer to the data for which to calculate the CRC.
 * @param len Length of the data in bytes.
 * @return The calculated CRC16 checksum.
 */
uint16_t protocolCrc16(
    const uint8_t* data,
    size_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        crc ^= ((uint16_t)*data++) << 8;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}
/**
 * Builds an acknowledgment packet for the given parameters.
 * 
 * @param channel The channel number for which the acknowledgment is being built.
 * @param counter The counter value to include in the acknowledgment.
 * @param result The result code to include in the acknowledgment.
 * @param buffer Pointer to the buffer where the acknowledgment packet will be stored.
 * @param len Pointer to a variable where the length of the acknowledgment packet will be stored.
 * 
 * @return true if the acknowledgment packet was successfully built, false otherwise.
 */
bool buildAckPacket(
    uint8_t channel,
    uint8_t counter,
    uint8_t result,
    uint8_t* buffer,
    size_t* len)
{
    AckFrame frame;

    frame.header.netId =
        PROTO_NET_ID;

    frame.header.ttl =
        PROTO_TTL_INIT;

    frame.payload.type =
        FRAME_ACK;

    frame.payload.counter =
        counter;

    frame.payload.channel =
        channel;

    frame.payload.result =
        result;

    memcpy(
        buffer,
        &frame,
        sizeof(frame));

    *len = sizeof(frame);

    return true;
}

/**
 * Builds a status packet for the given parameters.
 * 
 * @param channel The channel number for which the status is being built.
 * @param counter The counter value to include in the status.
 * @param state The state code to include in the status.
 * @param battery The battery level to include in the status.
 * @param buffer Pointer to the buffer where the status packet will be stored.
 * @param len Pointer to a variable where the length of the status packet will be stored.
 * 
 * @return true if the status packet was successfully built, false otherwise.
 */
bool buildStatusPacket(
    uint8_t channel,
    uint8_t counter,
    uint8_t state,
    uint8_t battery,
    uint8_t* buffer,
    size_t* len)
{
    StatusFrame frame;

    frame.header.netId =
        PROTO_NET_ID;

    frame.header.ttl =
        PROTO_TTL_INIT;

    frame.payload.type =
        FRAME_STATUS;

    frame.payload.counter =
        counter;

    frame.payload.channel =
        channel;

    frame.payload.state =
        state;

    frame.payload.battery =
        battery;

    memcpy(
        buffer,
        &frame,
        sizeof(frame));

    *len = sizeof(frame);

    return true;
}

bool decodeStatusPacket(
    const uint8_t* buffer,
    size_t len,
    StatusPayload* status)
{
    if (len != sizeof(StatusFrame))
        return false;

    const StatusFrame* frame =
        (const StatusFrame*)buffer;

    if (frame->header.netId != PROTO_NET_ID)
        return false;

    if (frame->payload.type != FRAME_STATUS)
        return false;

    *status = frame->payload;

    return true;
}