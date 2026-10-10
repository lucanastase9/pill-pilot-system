#ifndef ESC_MANAGER_HPP
#define ESC_MANAGER_HPP

#include <Arduino.h>
#include <HardwareTimer.h>

class ESCManager {
private:
    HardwareTimer* timer;
    uint32_t channels[4];
    uint8_t pins[4];
    bool armed;
    uint16_t throttleIdle;

public:
    ESCManager(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4);
    ~ESCManager();
    void init();
    void setArmed(bool state);
    bool isArmed() const { return armed; }
    void writeOutputs(int m1_us, int m2_us, int m3_us, int m4_us);
    void stopAll();
};

#endif