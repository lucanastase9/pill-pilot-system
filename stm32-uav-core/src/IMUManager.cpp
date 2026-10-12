#include "IMUManager.hpp"
#include <Arduino.h>

//initializarea senzorului bmi160 ca IMUManager
IMUManager::IMUManager(uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t mosi_pin)
    : csPin(cs_pin),
      sckPin(sck_pin),
      misoPin(miso_pin),
      mosiPin(mosi_pin),
      initialized(false),
      calibrated(false),
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
      filterTau(1.0f) {
    setGyroLPF(40.0f);
}

//configurarea filtrului PT1 la frecventa de taiere
void IMUManager::setGyroLPF(float cutoff_hz) {
    gyroFilterX.setCutoffFreq(cutoff_hz);
    gyroFilterY.setCutoffFreq(cutoff_hz);
    gyroFilterZ.setCutoffFreq(cutoff_hz);
}

//initializeaza hardware senzorul bmi160 in modul SPI
bool IMUManager::init() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    delay(10);
    SPI.setSCLK(sckPin);
    SPI.setMISO(misoPin);
    SPI.setMOSI(mosiPin);
    SPI.begin();
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
        // configuram hardware DLPF pentru eliminarea frecventelor foarte inalte
        BMI160.setGyroDLPFMode(BMI160_DLPF_MODE_NORM);
        BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_NORM);
        BMI160.setGyroRate(800);
        BMI160.setAccelerometerRate(800);
        roll = 0.0f;
        pitch = 0.0f;
        initialized = true;
        return true;
    }
    Serial.print("[IMU DEBUG] Chip ID citit de librarie: 0x");
    Serial.println(BMI160.getDeviceID(), HEX);
    return false;
}

//functia de calibrare a valorii de zero
void IMUManager::calibrate() {
    if (!initialized) {
        return;
    }
    delay(2000);
    long sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    const int numSamples = 1000;
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
    gyroOffsetX = (float)sumGyroX / numSamples;
    gyroOffsetY = (float)sumGyroY / numSamples;
    gyroOffsetZ = (float)sumGyroZ / numSamples;
    accelOffsetX = (float)sumAccelX / numSamples;
    accelOffsetY = (float)sumAccelY / numSamples;
    accelOffsetZ = ((float)sumAccelZ / numSamples) - EXPECTED_1G;
    // asezam roll si pitchul la 0
    roll = 0.0f;
    pitch = 0.0f;
    calibrated = true;
}

//actualizarea citirilor in zbor(inner loop)
void IMUManager::update(float dt) {
    if (!initialized || dt <= 0.0f) return;
    int gx, gy, gz;
    int ax, ay, az;
    BMI160.readGyro(gx, gy, gz);
    BMI160.readAccelerometer(ax, ay, az);

    //aplicam offseturile fata de 0
    float gx_f = (float)gx - gyroOffsetX;
    float gy_f = (float)gy - gyroOffsetY;
    float gz_f = (float)gz - gyroOffsetZ;
    float ax_f = (float)ax - accelOffsetX;
    float ay_f = (float)ay - accelOffsetY;
    float az_f = (float)az - accelOffsetZ;

    //axa x a senzorul e la -135grade fata de axa x a dronei
    float ax_drone = -ALIGNMENT_C * (ax_f + ay_f);
    float ay_drone =  ALIGNMENT_C * (ay_f - ax_f);
    float az_drone =  az_f;
    float gx_drone = -ALIGNMENT_C * (gx_f + gy_f);
    float gy_drone =  ALIGNMENT_C * (gy_f - gx_f);
    float gz_drone =  gz_f;

    //calculam acceleratia unghiulara
    float rawGyroX = gx_drone / GYRO_SCALE;
    float rawGyroY = gy_drone / GYRO_SCALE;
    float rawGyroZ = gz_drone / GYRO_SCALE;
    //filtru pe aceleratia unghiulara
    gyroRateX = gyroFilterX.apply(rawGyroX, dt);
    gyroRateY = gyroFilterY.apply(rawGyroY, dt);
    gyroRateZ = gyroFilterZ.apply(rawGyroZ, dt);

    //calcul de ungiuri pentru pitch si roll
    float accRoll  = atan2(ay_drone, az_drone) * RAD_TO_DEG;
    float accPitch = atan2(-ax_drone, sqrt(ay_drone * ay_drone + az_drone * az_drone)) * RAD_TO_DEG;
    //filtru pe unghiurile de pitch si roll
    float alpha = filterTau / (filterTau + dt);
    roll  = alpha * (roll + gyroRateX * dt) + (1.0f - alpha) * accRoll;
    pitch = alpha * (pitch + gyroRateY * dt) + (1.0f - alpha) * accPitch;
}