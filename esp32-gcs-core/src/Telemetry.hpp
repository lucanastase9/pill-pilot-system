#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

struct Telemetry {
    bool  isArmed   = false;
    int   throttle  = 1000; // 1000 - 2000 us
    float roll      = 0.0f; // grade
    float pitch     = 0.0f; // grade
    float altitude  = 0.0f; // metri
    int m1 = 1000;
    int m2 = 1000;
    int m3 = 1000;
    int m4 = 1000;
    int rssi = 0;
};

#endif