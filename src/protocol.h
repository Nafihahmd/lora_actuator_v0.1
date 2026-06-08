#pragma once

#include <Arduino.h>

#define PACKED __attribute__((packed))

static constexpr uint8_t PROTO_NET_ID   = 0x01;
static constexpr uint8_t PROTO_TTL_INIT = 0x03;
static constexpr uint8_t PROTO_VERSION  = 0x01;

enum FrameType : uint8_t {
    FRAME_CMD    = 0x01,
    FRAME_ACK    = 0x02,
    FRAME_STATUS = 0x03,
};

enum CommandCode : uint8_t {
    CMD_TRIGGER = 0x01,
    CMD_STATUS  = 0x02,
};

enum ResultCode : uint8_t {
    RES_ACCEPTED      = 0x01,
    RES_ALREADY_FIRED = 0x02,
    RES_BAD_COUNTER   = 0x03,
    RES_ERROR         = 0x04,
};

enum StateCode : uint8_t {
    STATE_READY         = 0x01,
    STATE_FIRED         = 0x02,
    STATE_NOT_REACHABLE = 0x03,
    STATE_ERROR         = 0x04,
};

struct PACKED ClearHeader {
    uint8_t netId;
    uint8_t ttl;
};

struct PACKED CmdPayload {
    uint8_t type;
    uint8_t counter;
    uint8_t channel;
    uint8_t command;
    uint16_t crc16;
};

struct PACKED AckPayload {
    uint8_t type;
    uint8_t counter;
    uint8_t channel;
    uint8_t result;
};

struct PACKED StatusPayload {
    uint8_t type;
    uint8_t counter;
    uint8_t channel;
    uint8_t state;
    uint8_t battery;
};

struct PACKED CmdFrame {
    ClearHeader header;
    CmdPayload payload;
};

struct PACKED AckFrame {
    ClearHeader header;
    AckPayload payload;
};

struct PACKED StatusFrame {
    ClearHeader header;
    StatusPayload payload;
};

static_assert(sizeof(CmdFrame) == 8);
static_assert(sizeof(AckFrame) == 6);
static_assert(sizeof(StatusFrame) == 7);

uint16_t protocolCrc16(
    const uint8_t* data,
    size_t len);

/********************************************
 * Actuator APIs
 *********************************************/
bool buildAckPacket(
    uint8_t channel,
    uint8_t counter,
    uint8_t result,
    uint8_t* buffer,
    size_t* len);

bool buildStatusPacket(
    uint8_t channel,
    uint8_t counter,
    uint8_t state,
    uint8_t battery,
    uint8_t* buffer,
    size_t* len);

bool decodeStatusPacket(
    const uint8_t* buffer,
    size_t len,
    StatusPayload* status);