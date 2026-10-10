#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <SPI.h>
#include <BMI160Gen.h>
#include "Filter.hpp"

class IMUManager {
private:
    // ========================================================
    // 1. HARDWARE & STARE
    // ========================================================
    uint8_t csPin;
    bool    initialized;

    // ========================================================
    // 2. STĂRI ESTIMATE (UNGHIURI ȘI RATE DE ROTAȚIE)
    // ========================================================
    float roll;          // Unghi Roll curent estimat (grade)
    float pitch;         // Unghi Pitch curent estimat (grade)
    float gyroRateX;     // Rata unghiulară Roll filtrată (deg/s)
    float gyroRateY;     // Rata unghiulară Pitch filtrată (deg/s)
    float gyroRateZ;     // Rata unghiulară Yaw filtrată (deg/s)

    // ========================================================
    // 3. ERORI DE ZERO / OFFSETS CALIBRARE (ZERO-BIAS)
    // ========================================================
    float accelOffsetX;  // Offset static accelerometru axa X
    float accelOffsetY;  // Offset static accelerometru axa Y
    float accelOffsetZ;  // Offset static accelerometru axa Z

    float gyroOffsetX;   // Offset static giroscop axa X
    float gyroOffsetY;   // Offset static giroscop axa Y
    float gyroOffsetZ;   // Offset static giroscop axa Z

    // ========================================================
    // 4. OBIECTE PENTRU FILTRARE (GIROSCOP) & FUZIUNE
    // ========================================================
    // Filtre software PT1 pentru giroscop (eliminare vibrații motoare)
    PT1Filter gyroFilterX;
    PT1Filter gyroFilterY;
    PT1Filter gyroFilterZ;

    // Factor de încredere filtru complementar (0.98 = 98% gyro, 2% accel)
    float complementaryAlpha;

    // ========================================================
    // 5. CONSTANTE FIZICE, SENZOR ȘI ALINIERE (135°)
    // ========================================================
    static constexpr float EXPECTED_1G = 16384.0f;     // 1G la sensibilitate ±2g (BMI160)
    static constexpr float GYRO_SCALE  = 131.0f;       // LSB per deg/s la sensibilitate ±250 deg/s
    static constexpr float RAD_TO_DEG  = 57.2957795f;  // Conversie radiani -> grade (180 / PI)
    static constexpr float ALIGNMENT_C = 0.70710678f;  // sqrt(2)/2 pentru rotația senzorului la 135°

public:
    // Constructor
    explicit IMUManager(uint8_t cs_pin);

    // Inițializare și buclă principală
    bool init();
    void calibrate();
    void update(float dt);

    // Configurare filtre
    void setGyroLPF(float cutoff_hz);
    void setComplementaryAlpha(float alpha);
    inline float getComplementaryAlpha() const { return complementaryAlpha; }

    // Getteri
    inline float getRoll() const { return roll; }
    inline float getPitch() const { return pitch; }
    inline float getGyroX() const { return gyroRateX; }
    inline float getGyroY() const { return gyroRateY; }
    inline float getGyroZ() const { return gyroRateZ; }
    inline bool  isInitialized() const { return initialized; }
};

#endif // IMU_MANAGER_HPP