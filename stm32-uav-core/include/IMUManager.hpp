#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <SPI.h>
#include <BMI160Gen.h>
#include "Filter.hpp"

class IMUManager {
private:
    //pini SPI
    uint8_t csPin;
    uint8_t sckPin;
    uint8_t misoPin;
    uint8_t mosiPin;
    bool    initialized;

    //iesirile finale din imu
    float roll;          // unghi roll curent final (grade)
    float pitch;         // unghi pitch curent final (grade)
    float gyroRateX;     // viteza unghiulara finala roll (grade/s)
    float gyroRateY;     // viteza unghiulara finala pitch (grade/s)
    float gyroRateZ;     // viteza unghiulara finala yaw (grade/s)

    //erorile de 0 pentru calibrare
    float accelOffsetX;
    float accelOffsetY;
    float accelOffsetZ;
    float gyroOffsetX;
    float gyroOffsetY;
    float gyroOffsetZ;

    //obiecte pentru filtrarea soft
    PT1Filter gyroFilterX;
    PT1Filter gyroFilterY;
    PT1Filter gyroFilterZ;

    //pentru filtru complementar roll si pitch
    float complementaryAlpha;

    //constante
    static constexpr float EXPECTED_1G = 16384.0f;      //acceleratia gravitationala presupusa
    static constexpr float GYRO_SCALE  = 131.0f;        //valoarea maxima acceptata de filtru
    static constexpr float ALIGNMENT_C = 0.70710678f;   //ungiul de 135greade in cos

public:
    //constructor
    IMUManager(uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t mosi_pin);
    //initializare si calibrare
    bool init();
    void calibrate();
    void update(float dt);
    //configurare filtre
    void setGyroLPF(float cutoff_hz);
    void setComplementaryAlpha(float alpha);
    //functii getter
    inline float getComplementaryAlpha() const { return complementaryAlpha; }
    inline float getRoll() const { return roll; }
    inline float getPitch() const { return pitch; }
    inline float getGyroX() const { return gyroRateX; }
    inline float getGyroY() const { return gyroRateY; }
    inline float getGyroZ() const { return gyroRateZ; }
    inline bool  isInitialized() const { return initialized; }
};

#endif // IMU_MANAGER_HPP
