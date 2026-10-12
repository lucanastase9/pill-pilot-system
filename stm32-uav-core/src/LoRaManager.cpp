#include "LoRaManager.hpp"
#include <Arduino.h>

// Flag setat de întreruperea hardware pe pinul DIO1
static volatile bool radioActionDone = false;

static void onDio1Interrupt() {
    radioActionDone = true;
}

LoRaManager::LoRaManager(SPIClass& spi_bus, uint8_t cs, uint8_t dio1, uint8_t rst, uint8_t busy)
    : spi(spi_bus), rxLength(0), hasNewPacket(false), transmitting(false) {
    mod = new Module(cs, dio1, rst, busy, spi_bus);
    radio = new SX1262(mod);
}

LoRaManager::~LoRaManager() {
    delete radio;
    delete mod;
}

bool LoRaManager::init() {
    spi.begin();

    // Freq: 868.0, BW: 500.0, SF: 7, CR: 5 (4/5), Sync: 0x12, Pwr: +22 dBm, Preamble: 8, TCXO: 3.3V, LDO: false
    int state = radio->begin(868.0, 500.0, 7, 5, 0x12, 22, 8, 3.3f, false);

    if (state == RADIOLIB_ERR_NONE) {
        // Activează comutatorul RF pe DIO2 (esențial pentru raza SX1262 / E22)
        radio->setDio2AsRfSwitch(true);

        radioActionDone = false;
        transmitting = false;
        radio->setDio1Action(onDio1Interrupt);
        radio->startReceive();
        return true;
    }
    return false;
}

void LoRaManager::update() {
    if (radioActionDone) {
        radioActionDone = false;

        if (transmitting) {
            // S-a finalizat transmisia pachetului asincron
            transmitting = false;
            // Trecem imediat înapoi pe recepție
            radio->startReceive();
        } else {
            // S-a recepționat un pachet nou
            size_t len = radio->getPacketLength();
            if (len > 0 && len <= sizeof(rxBuffer)) {
                int state = radio->readData(rxBuffer, len);
                if (state == RADIOLIB_ERR_NONE) {
                    rxLength = len;
                    hasNewPacket = true;
                }
            }
            // Continuăm să ascultăm
            radio->startReceive();
        }
    }
}

bool LoRaManager::available() {
    return hasNewPacket;
}

size_t LoRaManager::getPacket(uint8_t* buffer, size_t maxLen) {
    if (!hasNewPacket) return 0;
    hasNewPacket = false;
    size_t copyLen = (rxLength < maxLen) ? rxLength : maxLen;
    memcpy(buffer, rxBuffer, copyLen);
    return copyLen;
}

bool LoRaManager::sendPacket(const uint8_t* payload, size_t len) {
    // Dacă se transmite deja un pachet, nu blocăm bucla de zbor
    if (transmitting) {
        return false;
    }

    // startTransmit este NON-BLOCANT (trimite prin SPI comanda TX și iese instant)
    int state = radio->startTransmit((uint8_t*)payload, len);
    if (state == RADIOLIB_ERR_NONE) {
        transmitting = true;
        return true;
    }

    // În caz de eroare reintrăm în ascultare
    radio->startReceive();
    return false;
}