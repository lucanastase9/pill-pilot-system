#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <SPI.h>
#include <BMI160Gen.h>
#include "Filter.hpp"

class IMUManager {
private:
    uint8_t csPin;
    float roll;
    float pitch;
    float gyroRateX;
    float gyroRateY;
    float gyroRateZ;
    bool initialized;

    // --- VARIABILE NOI PENTRU CALIBRARE (BOARD ALIGNMENT) ---
    float accelOffsetX = 0.0f;
    float accelOffsetY = 0.0f;
    float accelOffsetZ = 0.0f;

    float gyroOffsetX = 0.0f;
    float gyroOffsetY = 0.0f;
    float gyroOffsetZ = 0.0f;

    // Filtre software pentru giroscop (reducerea zgomotului motoarelor)
    PT1Filter gyroFilterX;
    PT1Filter gyroFilterY;
    PT1Filter gyroFilterZ;

    // Valoarea teoretică pentru 1G (Dacă librăria citește date brute (raw) la +-2G, 1G = 16384)
    // Notă: Dacă folosești funcții care returnează deja G-uri (ex: 1.0, 0.0), poți schimba asta în 1.0f
    const float EXPECTED_1G = 16384.0f;

public:
    explicit IMUManager(uint8_t cs_pin);
    bool init();
    void update(float dt);

    // Permite configurarea frecventei de taiere pentru filtrul giroscopului (Hz)
    void setGyroLPF(float cutoff_hz);

    // --- FUNCȚIA NOUĂ DE CALIBRARE ---
    // Aceasta va fi apelată din main.cpp când apeși tasta 'b'
    void calibrate();

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getGyroX() const { return gyroRateX; }
    float getGyroY() const { return gyroRateY; }
    float getGyroZ() const { return gyroRateZ; }
};

#endif