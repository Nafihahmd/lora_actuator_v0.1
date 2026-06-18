#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include "stm32g0xx_hal.h"
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
static const uint8_t LORA_DIO1 = PA0;
static const uint8_t LORA_RST = PB7;

// LoRa Controller Pin Definitions
// static const uint8_t LORA_NSS  = PB0;
// static const uint8_t LORA_BUSY = PB1;
// static const uint8_t LORA_DIO1 = PA0;   // Shared with Wake Pin
// static const uint8_t LORA_RST  = PB7;

// GPIO Pin Definitions
static const uint8_t ACT_LED = PC14;
static const uint8_t TST_LED = PC15;

// Create LoRa instance using RadioLib (CS, IRQ/DIO1, RST, BUSY)
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// Clock configuration
extern "C" void SystemClock_Config(void);

// ------------------------------------------------------------
// Global Variables
// ------------------------------------------------------------
static const uint8_t ACC_CHANNEL = 0;
#define PULSE_DURATION_MS 1000
volatile bool pulseActive = false;
volatile uint32_t pulseEndMs = 0;

// ------------------------------------------------------------
// Receive flag
// ------------------------------------------------------------
static bool fired = false;
volatile bool receivedFlag = false;

void setFlag(void)
{
    receivedFlag = true;
}

/**
 * @brief Restart the radio receiver in duty cycle mode.
 * @return void
 */
static inline void restartRadioRx()
{
    int state =
        radio.startReceiveDutyCycleAuto();

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("RX restart failed: ");
        Serial.println(state);
    }
}

/**
 * @brief Enter low power mode by stopping the SPI and Serial interfaces, 
 * suspending the system tick, and entering STOP mode.
 * 
 * @return void
 */
static void enterLowPower()
{
    Serial.flush();

    Serial.end();
    SPI.end();

    HAL_SuspendTick();

    HAL_PWR_EnterSTOPMode(
        PWR_LOWPOWERREGULATOR_ON,
        PWR_STOPENTRY_WFI);
    HAL_ResumeTick();
    __WFI();
    
    SystemClock_Config();
    SPI.setMOSI(LORA_MOSI);
    SPI.setMISO(LORA_MISO);
    SPI.setSCLK(LORA_SCK);
    SPI.begin();

    Serial.begin(115200);
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup()
{
    pinMode(ACT_LED, OUTPUT);
    digitalWrite(ACT_LED, LOW);
    pinMode(TST_LED, OUTPUT);
    digitalWrite(TST_LED, LOW);
    pinMode(LORA_DIO1, INPUT_PULLDOWN);

    Serial.begin(115200);

    Serial.println();
    Serial.println("Actuator starting ");
    Serial.println(" Channel: " + String(ACC_CHANNEL));
    SPI.setMOSI(LORA_MOSI);
    SPI.setMISO(LORA_MISO);
    SPI.setSCLK(LORA_SCK);
    SPI.begin();

    ConfigLoRa_t config;
    config.frequency = 865.0;

    int state = radio.begin(865.0, 125.0, 10, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 180, 0, false);

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

    state = radio.startReceiveDutyCycleAuto();

    if(state != RADIOLIB_ERR_NONE)
    {
        Serial.print("startReceive failed: ");
        Serial.println(state);

        while(1)
        {
            delay(1000);
        }
    }
    delay(2000);    // So that I can flash without breaking the reset button at the right time

    Serial.println("Listening...");
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop()
{
    if(!receivedFlag)
    {
        enterLowPower();
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

        restartRadioRx();
        return;
    }

    Serial.println("Packet received");

    if(packetLength != sizeof(CmdFrame))
    {
        Serial.print("Unexpected length: ");
        Serial.println(packetLength);

        restartRadioRx();
        return;
    }

    CmdFrame* frame =
        (CmdFrame*)rxBuf;

    // Serial.printf("[NetID: 0x%02X] [TTL: %d] [Type: 0x%02X]\n", frame->header.netId, frame->header.ttl, frame->payload.type);

    if(frame->header.netId != PROTO_NET_ID)
    {
        Serial.println("Bad NetID");

        restartRadioRx();
        return;
    }

    if(frame->payload.type != FRAME_CMD)
    {
        Serial.println("Not CMD");

        restartRadioRx();
        return;
    }

    uint16_t crc =
        protocolCrc16(
            (uint8_t*)&frame->payload,
            sizeof(CmdPayload) - sizeof(uint16_t));

    if(crc != frame->payload.crc16)
    {
        Serial.println("CRC mismatch");

        restartRadioRx();
        return;
    }
    
    if(frame->payload.channel != ACC_CHANNEL)
    {
        Serial.println("Not my channel");

        restartRadioRx();
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
        /* Send ACK First */
        state = radio.transmit(txBuf, txLen);
        if(state != RADIOLIB_ERR_NONE)
        {
            Serial.print("TX error: ");
            Serial.println(state);
        }

        radio.finishTransmit();

        if(!fired)
        {
            Serial.println("TRIGGER RECEIVED");
            fired = true;

            digitalWrite(
                ACT_LED,
                HIGH);
            digitalWrite(
                TST_LED,
                HIGH);

            delay(PULSE_DURATION_MS);
            digitalWrite(ACT_LED, LOW);
            digitalWrite(TST_LED, LOW);
        }
        else
        {
            Serial.println("TRIGGER RECEIVED, BUT ALREADY FIRED");
        }

        restartRadioRx();
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

        restartRadioRx();
    }

    // Serial.print("RSSI: ");
    // Serial.println(radio.getRSSI());

    // Serial.print("SNR: ");
    // Serial.println(radio.getSNR());

    restartRadioRx();
}