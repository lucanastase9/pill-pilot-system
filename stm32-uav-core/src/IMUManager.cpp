#include "IMUManager.hpp"
#include <Arduino.h>

// Am adăugat inițializarea la 0 și pentru variabilele de offset, pentru siguranță
IMUManager::IMUManager(uint8_t cs_pin)
    : csPin(cs_pin), roll(0.0f), pitch(0.0f), gyroRateX(0.0f), gyroRateY(0.0f), gyroRateZ(0.0f), initialized(false),
      accelOffsetX(0.0f), accelOffsetY(0.0f), accelOffsetZ(0.0f),
      gyroOffsetX(0.0f), gyroOffsetY(0.0f), gyroOffsetZ(0.0f) {
          // Setăm o frecvență de tăiere implicită la 40 Hz
          setGyroLPF(40.0f);
}

void IMUManager::setGyroLPF(float cutoff_hz) {
    gyroFilterX.setCutoffFreq(cutoff_hz);
    gyroFilterY.setCutoffFreq(cutoff_hz);
    gyroFilterZ.setCutoffFreq(cutoff_hz);
}


bool IMUManager::init() {
    if (BMI160.begin(BMI160GenClass::SPI_MODE, csPin)) {
        // Configurăm filtrul Hardware DLPF în modul NORMAL
        // Aceasta elimină zgomotul de foarte înaltă frecvență direct pe cip
        BMI160.setGyroDLPFMode(BMI160_DLPF_MODE_NORM);
        BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_NORM);

        initialized = true;
        return true;
    }

    return false;
}

// --- FUNCȚIA NOUĂ DE CALIBRARE ---
void IMUManager::calibrate() {
    if (!initialized) {
        Serial.println("Eroare: IMU nu este initializat!");
        return;
    }

    Serial.println("=== INCEPERE CALIBRARE IMU ===");
    Serial.println("Te rog NU misca drona si asigura-te ca e perfect orizontala!");

    // Așteptăm 2 secunde ca drona să se stabilizeze mecanic
    delay(2000);

    // Folosim 'long' pentru sume ca să nu depășim limita variabilei
    // (1000 citiri * max 32768 = ~32.7 milioane, încape perfect în long pe 32-biți al STM32)
    long sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    const int numSamples = 1000;

    Serial.println("Calibrare in curs (aprox 3 secunde)...");

    for (int i = 0; i < numSamples; i++) {
        int gx, gy, gz;
        int ax, ay, az;

        BMI160.readGyro(gx, gy, gz);
        BMI160.readAccelerometer(ax, ay, az);

        sumGyroX += gx;
        sumGyroY += gy;
        sumGyroZ += gz;

        sumAccelX += ax;
        sumAccelY += ay;
        sumAccelZ += az;

        // Pauză de 3ms între citiri pentru a lăsa senzorul să producă date proaspete
        delay(3);
    }

    // Calculăm mediile pentru Giroscop (care ar trebui să fie 0)
    gyroOffsetX = (float)sumGyroX / numSamples;
    gyroOffsetY = (float)sumGyroY / numSamples;
    gyroOffsetZ = (float)sumGyroZ / numSamples;

    // Calculăm mediile pentru Accelerometru
    // Pe Z scădem EXPECTED_1G (ex: 16384) definit în .hpp, deoarece acolo acționează gravitația
    accelOffsetX = (float)sumAccelX / numSamples;
    accelOffsetY = (float)sumAccelY / numSamples;
    accelOffsetZ = ((float)sumAccelZ / numSamples) - EXPECTED_1G;

    Serial.println("=== CALIBRARE COMPLETA ===");
    Serial.print("Gyro Offsets: X="); Serial.print(gyroOffsetX);
    Serial.print(" Y="); Serial.print(gyroOffsetY);
    Serial.print(" Z="); Serial.println(gyroOffsetZ);

    Serial.print("Accel Offsets: X="); Serial.print(accelOffsetX);
    Serial.print(" Y="); Serial.print(accelOffsetY);
    Serial.print(" Z="); Serial.println(accelOffsetZ);
}

// --- ACTUALIZAREA CITIRILOR ÎN ZBOR ---
void IMUManager::update(float dt) {
    if (!initialized) return;

    int gx, gy, gz;
    int ax, ay, az;

    BMI160.readGyro(gx, gy, gz);
    BMI160.readAccelerometer(ax, ay, az);

    // 1. APLICĂM OFFSETS-urile PENTRU GIROSCOP
    float gx_f = (float)gx - gyroOffsetX;
    float gy_f = (float)gy - gyroOffsetY;
    float gz_f = (float)gz - gyroOffsetZ;

    // 2. APLICĂM OFFSETS-urile PENTRU ACCELEROMETRU
    float ax_f = (float)ax - accelOffsetX;
    float ay_f = (float)ay - accelOffsetY;
    float az_f = (float)az - accelOffsetZ;

    // 3. CALCULĂM VITEZELE UNGHIULARE (Folosim gx_f si gy_f corectate!)
    float rawGyroX = gx_f / 131.0f;
    float rawGyroY = gy_f / 131.0f;
    float rawGyroZ = gz_f / 131.0f;

    // APLICĂM FILTRUL SOFTWARE PT1
    gyroRateX = gyroFilterX.apply(rawGyroX, dt);
    gyroRateY = gyroFilterY.apply(rawGyroY, dt);
    gyroRateZ = gyroFilterZ.apply(rawGyroZ, dt);

    // 4. CALCULĂM UNGHIURILE DIN ACCELEROMETRU (Folosim ax_f, ay_f, az_f corectate!)
    float accRoll = atan2(ay_f, az_f) * 57.2957795f;
    float accPitch = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * 57.2957795f;

    // 5. FILTRUL COMPLEMENTAR (Pitch și Roll curate)
    roll  = 0.98f * (roll + gyroRateX * dt) + 0.02f * accRoll;
    pitch = 0.98f * (pitch + gyroRateY * dt) + 0.02f * accPitch;
}