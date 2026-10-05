#include "ESCManager.hpp"
#include <Arduino.h>

ESCManager::ESCManager(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4)
    : armed(false), throttleIdle(1150) {
    pins[0] = m1;
    pins[1] = m2;
    pins[2] = m3;
    pins[3] = m4;
}

void ESCManager::init() {
    for (int i = 0; i < 4; i++) {
        motors[i].attach(pins[i], 1000, 2000);
    }
    stopAll();
}

void ESCManager::setArmed(bool state) {
    armed = state;
    if (!armed) {
        stopAll();
    }
}

void ESCManager::writeOutputs(int m1_us, int m2_us, int m3_us, int m4_us) {
    if (!armed) {
        stopAll();
        return;
    }

    // Limitare cu relanti pentru stabilitate in zbor (ThrottleIdle)
    motors[0].writeMicroseconds(constrain(m1_us, throttleIdle, 2000));
    motors[1].writeMicroseconds(constrain(m2_us, throttleIdle, 2000));
    motors[2].writeMicroseconds(constrain(m3_us, throttleIdle, 2000));
    motors[3].writeMicroseconds(constrain(m4_us, throttleIdle, 2000));
}

void ESCManager::stopAll() {
    for (int i = 0; i < 4; i++) {
        motors[i].writeMicroseconds(1000);
    }
}