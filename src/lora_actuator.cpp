#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include "protocol.h"


// ------------------------------------------------------------
// SX1262 wiring
// ------------------------------------------------------------
// SPI1
static const uint8_t LORA_SCK  = PB3;
static const uint8_t LORA_MISO = PB4;
static const uint8_t LORA_MOSI = PB5;

// LoRa Actuator Pin Definitions
static const uint8_t LORA_NSS = PB8;
static const uint8_t LORA_BUSY = PA8;
static const uint8_t LORA_DIO1 = PA7;
static const uint8_t LORA_RST = PB7;

// LoRa Controller Pin Definitions
// static const uint8_t LORA_NSS  = PB0;
// static const uint8_t LORA_BUSY = PB1;
// static const uint8_t LORA_DIO1 = PA0;   // Shared with Wake Pin
// static const uint8_t LORA_RST  = PB7;

// Create LoRa instance using RadioLib (CS, IRQ/DIO1, RST, BUSY)
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// ------------------------------------------------------------
// Actuator Channels
// ------------------------------------------------------------
static const uint8_t ACC_CHANNEL = 0;


static const uint8_t ACT_LED = PC14;
// ------------------------------------------------------------
// Receive flag
// ------------------------------------------------------------
static bool fired = false;
volatile bool receivedFlag = false;

void setFlag(void)
{
    receivedFlag = true;
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup()
{
    pinMode(ACT_LED, OUTPUT);
    digitalWrite(ACT_LED, LOW);

    Serial.begin(115200);

    Serial.println();
    Serial.println("Actuator starting ");
    SPI.setMOSI(LORA_MOSI);
    SPI.setMISO(LORA_MISO);
    SPI.setSCLK(LORA_SCK);
    SPI.begin();

    ConfigLoRa_t config;
    config.frequency = 865.0;

    int state = radio.begin(865.0, 125.0, 12, 8, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 0, false);

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("Radio init failed: ");
        Serial.println(state);

        while(1)
        {
            delay(1000);
        }
    }

    Serial.println("Radio init OK");

    radio.setPacketReceivedAction(setFlag);

    state = radio.startReceive();

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("startReceive failed: ");
        Serial.println(state);

        while(1)
        {
            delay(1000);
        }
    }

    Serial.println("Listening...");
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop()
{
    if(!receivedFlag)
    {
        return;
    }

    receivedFlag = false;

    uint8_t rxBuf[32];

    int packetLength =
        radio.getPacketLength();

    int state =
        radio.readData(
            rxBuf,
            packetLength);

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("RX error: ");
        Serial.println(state);

        radio.startReceive();
        return;
    }

    Serial.println("Packet received");

    if(packetLength != sizeof(CmdFrame))
    {
        Serial.print("Unexpected length: ");
        Serial.println(packetLength);

        radio.startReceive();
        return;
    }

    CmdFrame* frame =
        (CmdFrame*)rxBuf;

    Serial.printf("[NetID: 0x%02X] [TTL: %d] [Type: 0x%02X]\n", frame->header.netId, frame->header.ttl, frame->payload.type);

    if(frame->header.netId != PROTO_NET_ID)
    {
        Serial.println("Bad NetID");

        radio.startReceive();
        return;
    }

    if(frame->payload.type != FRAME_CMD)
    {
        Serial.println("Not CMD");

        radio.startReceive();
        return;
    }

    uint16_t crc =
        protocolCrc16(
            (uint8_t*)&frame->payload,
            sizeof(CmdPayload) - sizeof(uint16_t));

    if(crc != frame->payload.crc16)
    {
        Serial.println("CRC mismatch");

        radio.startReceive();
        return;
    }
    
    if(frame->payload.channel != ACC_CHANNEL)
    {
        Serial.println("Not my channel");

        radio.startReceive();
        return;
    }

    Serial.print("Counter: ");
    Serial.println(frame->payload.counter);

    Serial.print("Channel: ");
    Serial.println(frame->payload.channel);

    Serial.print("Command: ");
    Serial.println(frame->payload.command);

    if(frame->payload.command == CMD_TRIGGER)
    {
        uint8_t txBuf[16];
        size_t txLen;

        buildAckPacket(
            frame->payload.channel,
            frame->payload.counter,
            fired ?
            RES_ACCEPTED :
            RES_ALREADY_FIRED,
            txBuf,
            &txLen);

        if(!fired)
        {
            Serial.println("TRIGGER RECEIVED");
            fired = true;

            digitalWrite(
                ACT_LED,
                HIGH);
        }
        else
        {
            Serial.println("TRIGGER RECEIVED, BUT ALREADY FIRED");
        }

        state = radio.transmit(txBuf, txLen);
        if(state != RADIOLIB_ERR_NONE)
        {
            Serial.print("TX error: ");
            Serial.println(state);
        }

        radio.finishTransmit();

        radio.startReceive();
    }
    else if(frame->payload.command == CMD_STATUS)
    {
        uint8_t txBuf[16];
        size_t txLen;

        Serial.println("STATUS REQUEST RECEIVED");
        buildStatusPacket(
            frame->payload.channel,
            frame->payload.counter,
            fired ?
                STATE_FIRED :
                STATE_READY,
            100,
            txBuf,
            &txLen);

        radio.transmit(txBuf, txLen);
        radio.finishTransmit();

        radio.startReceive();
    }

    Serial.print("RSSI: ");
    Serial.println(radio.getRSSI());

    Serial.print("SNR: ");
    Serial.println(radio.getSNR());

    radio.startReceive();
}