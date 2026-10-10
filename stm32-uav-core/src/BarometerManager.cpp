#include "BarometerManager.hpp"
#include <math.h>

bool BarometerManager::init() {
    if (!ms5611.begin()) {
        return false;
    }
    ms5611.setOversampling(OSR_ULTRA_LOW); // conversie rapida de ~1ms in loc de 18ms
    return true;
}

void BarometerManager::setGroundZero() {
    float pressureSum = 0.0f;
    int samples = 100;

    for(int i = 0; i < samples; i++) {
        ms5611.read();
        pressureSum += ms5611.getPressure();
        delay(30);
    }
    groundPressure = pressureSum / samples;
    // Resetăm filtrul la calibrare
    filteredAltitude = 0.0f;
}

void BarometerManager::update() {
    int status = ms5611.read();
    float currentPressure = ms5611.getPressure();

    // Verificăm validitatea citirii
    if (status == 0 && currentPressure > 900.0f && groundPressure > 0.0f) {
        currentTemp = ms5611.getTemperature();

        // Calcul altitudine brută
        rawAltitude = 44330.0f * (1.0f - pow((double)currentPressure / (double)groundPressure, 0.190295));

        // Aplicăm filtrul EMA (filtrează zgomotul)
        filteredAltitude = (alpha * rawAltitude) + ((1.0f - alpha) * filteredAltitude);
    }
}

float BarometerManager::getRelativeAltitude() {
    return filteredAltitude; // Returnăm valoarea curată
}

float BarometerManager::getTemperature() {
    return currentTemp;
}