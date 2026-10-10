#include "ESCManager.hpp"
#include <Arduino.h>

ESCManager::ESCManager(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4)
    : timer(nullptr), armed(false), throttleIdle(1150) {
    pins[0] = m1;
    pins[1] = m2;
    pins[2] = m3;
    pins[3] = m4;
    channels[0] = 1;
    channels[1] = 2;
    channels[2] = 3;
    channels[3] = 4;
}

//eliberam memoria alocata
ESCManager::~ESCManager() {
    if (timer) {
        timer->pause();
        delete timer;
        timer = nullptr;
    }
}

void ESCManager::init() {
    //iesire a timerului TIM2 pe pinii 0, 1, 2, 3 pentru a trimite catre escuri valorile in pulsuri
    TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pins[0]), PinMap_PWM);
    if (!instance) {
        instance = TIM2;
    }
    timer = new HardwareTimer(instance);
    for (int i = 0; i < 4; i++) {
        channels[i] = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pins[i]), PinMap_PWM));
        timer->setMode(channels[i], TIMER_OUTPUT_COMPARE_PWM1, pins[i]);
    }
    // setam frecventa la 500Hz
    timer->setOverflow(500, HERTZ_FORMAT);
    //plecam cu motoarele oprite
    stopAll();
    //pornim timerul hardware
    timer->resume();
}

void ESCManager::setArmed(bool state) {
    armed = state;
    if (!armed) {
        stopAll();
    }
}

void ESCManager::writeOutputs(int m1_us, int m2_us, int m3_us, int m4_us) {
    if (!armed || !timer) {
        stopAll();
        return;
    }
    //limitam pulsurile la 1150 cand e armata si 1995 maxim
    int outputs[4];
    outputs[0] = constrain(m1_us, throttleIdle, 1995);
    outputs[1] = constrain(m2_us, throttleIdle, 1995);
    outputs[2] = constrain(m3_us, throttleIdle, 1995);
    outputs[3] = constrain(m4_us, throttleIdle, 1995);
    for (int i = 0; i < 4; i++) {
        timer->setCaptureCompare(channels[i], outputs[i], MICROSEC_COMPARE_FORMAT);
    }
}

void ESCManager::stopAll() {
    if (!timer) return;
    for (int i = 0; i < 4; i++) {
        timer->setCaptureCompare(channels[i], 1000, MICROSEC_COMPARE_FORMAT);
    }
}