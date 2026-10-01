#include "LoRaManager.hpp"
#include <Arduino.h>

volatile bool loraReceivedFlag = false;

void setFlag() {
    loraReceivedFlag = true;
}

LoRaManager::LoRaManager(SPIClass& spi_bus, uint8_t cs, uint8_t dio1, uint8_t rst, uint8_t busy)
    : spi(spi_bus), rxLength(0), hasNewPacket(false) {
    mod = new Module(cs, dio1, rst, busy, spi_bus);
    radio = new SX1262(mod);
}

bool LoRaManager::init() {
    spi.begin();
    int state = radio->begin(868.0, 500.0, 7, 7, 0x12, 10);

    if (state == RADIOLIB_ERR_NONE) {
        radio->setDio1Action(setFlag);
        radio->startReceive();
        return true;
    }
    return false;
}

void LoRaManager::update() {
    if (loraReceivedFlag) {
        loraReceivedFlag = false;

        size_t len = radio->getPacketLength();
        if (len > 0 && len <= sizeof(rxBuffer)) {
            int state = radio->readData(rxBuffer, len);
            if (state == RADIOLIB_ERR_NONE) {
                rxLength = len;
                hasNewPacket = true;
            }
        }
        radio->startReceive();
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
    // 1. Oprim temporar ascultarea întreruperilor pe pinul DIO1
    // pentru a nu confunda sfârșitul transmisiei cu un pachet primit.
    radio->clearDio1Action();

    // 2. Folosim transmit() - funcție BLOCANTĂ.
    // Codul așteaptă aici câteva milisecunde până când pachetul e fizic în aer.
    int state = radio->transmit((uint8_t*)payload, len);

    // 3. Reconectăm pinul de întrerupere la funcția setFlag (pentru recepție)
    radio->setDio1Action(setFlag);

    // 4. CRITIC: Punem imediat modulul înapoi în modul de ASCULTARE!
    radio->startReceive();

    return (state == RADIOLIB_ERR_NONE);
}