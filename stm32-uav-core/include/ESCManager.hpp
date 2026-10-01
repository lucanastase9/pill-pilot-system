#ifndef ESC_MANAGER_HPP
#define ESC_MANAGER_HPP

#include <Servo.h>

class ESCManager {
private:
    Servo motors[4];
    uint8_t pins[4];
    bool armed;

public:
    ESCManager(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4);
    void init();
    void setArmed(bool state);
    bool isArmed() const { return armed; }
    void writeOutputs(int m1_us, int m2_us, int m3_us, int m4_us);
    void stopAll();
};

#endif