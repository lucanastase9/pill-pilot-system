#ifndef BAROMETER_MANAGER_HPP
#define BAROMETER_MANAGER_HPP

#include <Wire.h>
#include "MS5611.h"

class BarometerManager {
private:
    MS5611 ms5611;
    TwoWire* i2cBus;
    float groundPressure;

    // Variabilele noi pentru filtrare
    float rawAltitude;
    float filteredAltitude;
    float alpha;
    float currentTemp;

public:
    // Constructorul trebuie să inițializeze și variabilele noi
    BarometerManager(TwoWire* bus = &Wire) : ms5611(0x77),
                                                 i2cBus(bus),
                                                 groundPressure(0.0f),
                                                 rawAltitude(0.0f),
                                                 filteredAltitude(0.0f),
                                                 currentTemp(0.0f),
                                                 alpha(0.1f) {}

    bool init();
    void setGroundZero();
    void update();
    float getRelativeAltitude(); // Va returna filteredAltitude
    float getTemperature();
};

#endif