#ifndef LORA_MANAGER_HPP
#define LORA_MANAGER_HPP

#include <RadioLib.h>
#include <SPI.h>

class LoRaManager {
private:
    Module* mod;
    SX1262* radio;
    SPIClass& spi;

    uint8_t rxBuffer[256];
    size_t rxLength;
    bool hasNewPacket;
    bool transmitting;

public:
    LoRaManager(SPIClass& spi_bus, uint8_t cs, uint8_t dio1, uint8_t rst, uint8_t busy);
    ~LoRaManager();

    bool init();
    void update();

    bool available();
    size_t getPacket(uint8_t* buffer, size_t maxLen);

    // Transmisie non-blocantă
    bool sendPacket(const uint8_t* payload, size_t len);
    bool isTransmitting() const { return transmitting; }
};

#endif