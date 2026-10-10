#include "IMUManager.hpp"
#include <Arduino.h>

// ========================================================
// CONSTRUCTOR: Instanțiere și inițializare explicită
// ========================================================
IMUManager::IMUManager(uint8_t cs_pin)
    : csPin(cs_pin),
      initialized(false),
      roll(0.0f),
      pitch(0.0f),
      gyroRateX(0.0f),
      gyroRateY(0.0f),
      gyroRateZ(0.0f),
      accelOffsetX(0.0f),
      accelOffsetY(0.0f),
      accelOffsetZ(0.0f),
      gyroOffsetX(0.0f),
      gyroOffsetY(0.0f),
      gyroOffsetZ(0.0f),
      gyroFilterX(),
      gyroFilterY(),
      gyroFilterZ(),
      kalmanRoll(),
      kalmanPitch() {

    // Configurare referință inițială orizontală pentru filtrele Kalman
    kalmanRoll.setAngle(0.0f);
    kalmanPitch.setAngle(0.0f);

    // Inițializare frecvență de tăiere pentru filtrele PT1 ale giroscopului (40 Hz default)
    setGyroLPF(40.0f);
}

// ========================================================
// CONFIGURARE FILTRE
// ========================================================
void IMUManager::setGyroLPF(float cutoff_hz) {
    gyroFilterX.setCutoffFreq(cutoff_hz);
    gyroFilterY.setCutoffFreq(cutoff_hz);
    gyroFilterZ.setCutoffFreq(cutoff_hz);
}

void IMUManager::setKalmanR(float r_measure) {
    kalmanRoll.setR(r_measure);
    kalmanPitch.setR(r_measure);
}

void IMUManager::setKalmanParameters(float q_angle, float q_bias, float r_measure) {
    kalmanRoll.setParameters(q_angle, q_bias, r_measure);
    kalmanPitch.setParameters(q_angle, q_bias, r_measure);
}

float IMUManager::getKalmanR() const {
    return kalmanRoll.getR();
}

// ========================================================
// INIȚIALIZARE HARDWARE (BMI160 prin SPI)
// ========================================================
bool IMUManager::init() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    delay(10);

    // Protocolul BMI160: Senzorul porneste implicit in mod I2C la alimentare!
    // Pentru a forta comutarea in SPI, CSB trebuie coborat, trimis un dummy read si ridicat inapoi la HIGH:
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(csPin, LOW);
    delay(2);
    SPI.transfer(0x80); // Read register 0x00 (CHIP_ID)
    uint8_t raw_id = SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();

    Serial.print("[IMU DEBUG] Raw SPI Chip ID: 0x");
    Serial.println(raw_id, HEX);
    delay(10);

    if (BMI160.begin(BMI160GenClass::SPI_MODE, csPin)) {
        // Configurăm filtrul Hardware DLPF în modul NORMAL
        // Aceasta elimină zgomotul de foarte înaltă frecvență direct pe cip
        BMI160.setGyroDLPFMode(BMI160_DLPF_MODE_NORM);
        BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_NORM);

        // Presupunem că drona pornește pe o suprafață plană (orizontală = 0.0°)
        kalmanRoll.setAngle(0.0f);
        kalmanPitch.setAngle(0.0f);
        roll = 0.0f;
        pitch = 0.0f;

        initialized = true;
        return true;
    }

    // In caz de eroare trimite ID-ul de debug
    Serial.print("[IMU DEBUG] Chip ID citit de librarie: 0x");
    Serial.println(BMI160.getDeviceID(), HEX);
    return false;
}

// ========================================================
// FUNCȚIA DE CALIBRARE (ZERO-OFFSETS)
// ========================================================
void IMUManager::calibrate() {
    if (!initialized) {
        Serial.println("Eroare: IMU nu este initializat!");
        return;
    }

    Serial.println("=== INCEPERE CALIBRARE IMU ===");
    Serial.println("Te rog NU misca drona si asigura-te ca e perfect orizontala!");

    delay(2000);

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

        delay(3);
    }

    // Calculăm mediile pentru Giroscop (care ar trebui să fie 0)
    gyroOffsetX = (float)sumGyroX / numSamples;
    gyroOffsetY = (float)sumGyroY / numSamples;
    gyroOffsetZ = (float)sumGyroZ / numSamples;

    // Calculăm mediile pentru Accelerometru
    // Pe Z scădem EXPECTED_1G deoarece acolo acționează gravitația
    accelOffsetX = (float)sumAccelX / numSamples;
    accelOffsetY = (float)sumAccelY / numSamples;
    accelOffsetZ = ((float)sumAccelZ / numSamples) - EXPECTED_1G;

    // Resetăm filtrele Kalman pe 0.0f (orizontală de referință)
    kalmanRoll.setAngle(0.0f);
    kalmanPitch.setAngle(0.0f);
    roll = 0.0f;
    pitch = 0.0f;

    Serial.println("=== CALIBRARE COMPLETA ===");
    Serial.print("Gyro Offsets: X="); Serial.print(gyroOffsetX);
    Serial.print(" Y="); Serial.print(gyroOffsetY);
    Serial.print(" Z="); Serial.println(gyroOffsetZ);

    Serial.print("Accel Offsets: X="); Serial.print(accelOffsetX);
    Serial.print(" Y="); Serial.print(accelOffsetY);
    Serial.print(" Z="); Serial.println(accelOffsetZ);
}

// ========================================================
// ACTUALIZAREA CITIRILOR ÎN ZBOR (FAST LOOP)
// ========================================================
void IMUManager::update(float dt) {
    if (!initialized) return;

    int gx, gy, gz;
    int ax, ay, az;

    BMI160.readGyro(gx, gy, gz);
    BMI160.readAccelerometer(ax, ay, az);

    // 1. APLICĂM OFFSETS-urile (ZERO-BIAS) PENTRU GIROSCOP
    float gx_f = (float)gx - gyroOffsetX;
    float gy_f = (float)gy - gyroOffsetY;
    float gz_f = (float)gz - gyroOffsetZ;

    // 2. APLICĂM OFFSETS-urile (ZERO-BIAS) PENTRU ACCELEROMETRU
    float ax_f = (float)ax - accelOffsetX;
    float ay_f = (float)ay - accelOffsetY;
    float az_f = (float)az - accelOffsetZ;

    // 3. APLICĂM TRANSFORMAREA DE 135 GRADE PENTRU ALINIEREA SENZORULUI (BOARD ALIGNMENT)
    // cos(135°) = -ALIGNMENT_C, sin(135°) = ALIGNMENT_C
    float ax_drone = -ALIGNMENT_C * (ax_f + ay_f);
    float ay_drone =  ALIGNMENT_C * (ay_f - ax_f);
    float az_drone =  az_f;

    float gx_drone = -ALIGNMENT_C * (gx_f + gy_f);
    float gy_drone =  ALIGNMENT_C * (gy_f - gx_f);
    float gz_drone =  gz_f;

    // 4. CALCULĂM VITEZELE UNGHIULARE REALE ÎN DEG/S
    float rawGyroX = gx_drone / GYRO_SCALE;
    float rawGyroY = gy_drone / GYRO_SCALE;
    float rawGyroZ = gz_drone / GYRO_SCALE;

    // 5. APLICĂM FILTRUL SOFTWARE PT1 PENTRU GIROSCOP (ATENUARE VIBRAȚII)
    gyroRateX = gyroFilterX.apply(rawGyroX, dt);
    gyroRateY = gyroFilterY.apply(rawGyroY, dt);
    gyroRateZ = gyroFilterZ.apply(rawGyroZ, dt);

    // 6. CALCULĂM UNGHIURILE DIN ACCELEROMETRU (GRADE)
    float accRoll  = atan2(ay_drone, az_drone) * RAD_TO_DEG;
    float accPitch = atan2(-ax_drone, sqrt(ay_drone * ay_drone + az_drone * az_drone)) * RAD_TO_DEG;

    // 7. FILTRU KALMAN (FUZIUNE SENZORI PENTRU ESTIMARE UNGHI ROLL ȘI PITCH)
    roll  = kalmanRoll.update(accRoll, gyroRateX, dt);
    pitch = kalmanPitch.update(accPitch, gyroRateY, dt);
}