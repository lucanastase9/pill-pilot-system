#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MAVLink.h>

#include "IMUManager.hpp"
#include "ESCManager.hpp"
#include "PIDController.hpp"
#include "MotorMixer.hpp"
#include "LoRaManager.hpp"
#include "BarometerManager.hpp"
#include "Filter.hpp"


// ==========================================
// 1. INSTANȚIEREA OBIECTELOR (MODULELOR)
// ==========================================
IMUManager imu(PA4);                       // BMI160 pe SPI1
ESCManager esc(PA0, PA1, PA2, PA3);        // PWM ESC
SPIClass SPI_2(PB15, PB14, PB13);
LoRaManager lora(SPI_2, PB12, PA8, PA10, PA9); // LoRa pe SPI2
BarometerManager baro;                     // MS5611 pe I2C

// ==========================================
// CONTROLERE PID CASCADATE
// ==========================================
// Outer Loop: Angle Controllers (Doar P-term activ, ieșire în °/s)
PIDController rollAnglePID(3.5f, 0.0f, 0.0f, 0.0f, -180.0f, 180.0f);
PIDController pitchAnglePID(3.5f, 0.0f, 0.0f, 0.0f, -180.0f, 180.0f);

// Inner Loop: Rate Controllers (PID complet, ieșire în unități PWM)
PIDController rollRatePID(1.2f, 0.05f, 0.02f, 100.0f, -350.0f, 350.0f);
PIDController pitchRatePID(1.2f, 0.05f, 0.02f, 100.0f, -350.0f, 350.0f);
PIDController yawRatePID(2.0f, 0.1f, 0.0f, 100.0f, -300.0f, 300.0f);

// Filtre pentru Setpoint Smoothing (Netezirea comenzilor de pe butoane)
PT1Filter targetRollFilter;
PT1Filter targetPitchFilter;
PT1Filter targetYawFilter;

// ==========================================
// 2. VARIABILE DE ZBOR ȘI SCHEDULING
// ==========================================
int currentThrottle = 1000;
float targetRollAngle = 0.0f;
float targetPitchAngle = 0.0f;
float targetYawRate = 0.0f;

// ---- Variabile pentru turația fiecărui motor (PWM) ----
int m1_pwm = 1000;
int m2_pwm = 1000;
int m3_pwm = 1000;
int m4_pwm = 1000;
char lastUpdatedParamId[17] = {0};
float lastUpdatedParamVal = 0.0f;
int lastUpdatedParamIdx = -1;

// ---- Timing (Loop Scheduling) ----
unsigned long lastFastLoop = 0;
unsigned long lastMediumLoop = 0;
unsigned long lastSlowLoop = 0;
const unsigned long FAST_LOOP_MICROS = 4000; // 250 Hz

const uint8_t system_id = 1;
const uint8_t component_id = 1;

// Helper funcție pentru a trimite o singură valoare parametru prin MAVLink
void sendSingleParam(const char* param_id, float value, uint16_t param_index) {
    mavlink_message_t txMsg;
    uint8_t txBuf[128];
    mavlink_param_value_t param_val;
    memset(&param_val, 0, sizeof(param_val));
    
    strncpy(param_val.param_id, param_id, 16);
    param_val.param_value = value;
    param_val.param_type = MAV_PARAM_TYPE_REAL32;
    param_val.param_count = 11;
    param_val.param_index = param_index;

    mavlink_msg_param_value_encode(system_id, component_id, &txMsg, &param_val);
    uint16_t len = mavlink_msg_to_send_buffer(txBuf, &txMsg);
    lora.sendPacket(txBuf, len);
}

// Funcție pentru a trimite toți parametrii de PID la cererea aplicației
void sendAllParams() {
    sendSingleParam("RATE_ROLL_P", rollRatePID.getP(), 0);
    sendSingleParam("RATE_ROLL_I", rollRatePID.getI(), 1);
    sendSingleParam("RATE_ROLL_D", rollRatePID.getD(), 2);
    
    sendSingleParam("RATE_PITCH_P", pitchRatePID.getP(), 3);
    sendSingleParam("RATE_PITCH_I", pitchRatePID.getI(), 4);
    sendSingleParam("RATE_PITCH_D", pitchRatePID.getD(), 5);
    
    sendSingleParam("RATE_YAW_P", yawRatePID.getP(), 6);
    sendSingleParam("RATE_YAW_I", yawRatePID.getI(), 7);
    sendSingleParam("RATE_YAW_D", yawRatePID.getD(), 8);

    sendSingleParam("ANG_ROLL_P", rollAnglePID.getP(), 9);
    sendSingleParam("ANG_PITCH_P", pitchAnglePID.getP(), 10);
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== DRONA STM32 OOP: PREGATITA ===");

    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    Wire.setClock(400000);

    // Initializare setpoint smoothing
    targetRollFilter.setCutoffFreq(5.0f);  
    targetPitchFilter.setCutoffFreq(5.0f);
    targetYawFilter.setCutoffFreq(5.0f);

    rollRatePID.setFeedforward(0.1f);
    pitchRatePID.setFeedforward(0.1f);
    yawRatePID.setFeedforward(0.0f);

    rollAnglePID.setDeadband(0.2f);
    pitchAnglePID.setDeadband(0.2f);
    rollRatePID.setDeadband(0.5f);
    pitchRatePID.setDeadband(0.5f);

    Serial.println("Initializare ESC...");
    esc.init();
    
    Serial.println("Initializare SPI1 (Fortare pini hardware pentru IMU)...");
    SPI.setSCLK(PA5);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    // NU chemam SPI.begin() aici, libraria o va face!
    
    Serial.println("Initializare IMU...");
    if (imu.init()) {
        Serial.println("[OK] Senzor BMI160 conectat.");
    } else {
        Serial.println("[EROARE] Senzor BMI160 NU a fost gasit sau a esuat initializarea!");
    }

    Serial.println("Initializare Barometru...");
    if (baro.init()) {
        Serial.println("[OK] Barometru MS5611 conectat. Calibrare...");
        delay(200);
        baro.setGroundZero();
    } else {
        Serial.println("[EROARE] Barometru MS5611 NU a fost gasit!");
    }

    Serial.println("Initializare LoRa...");
    if (lora.init()) {
        Serial.println("[OK] Modul LoRa SX1262 online.");
    } else {
        Serial.println("[EROARE] Modul LoRa SX1262 a esuat! Verifica firele (SPI/CS/DIO1/RST)!");
    }

    lastFastLoop = micros();
    lastMediumLoop = millis();
    lastSlowLoop = millis();
}

void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMillis = millis();

    // ========================================================
    // BUCLA RAPIDA: 250 Hz (IMU + PID CASCADAT + MOTOARE)
    // ========================================================
    if (currentMicros - lastFastLoop >= FAST_LOOP_MICROS) {
        float dt = (currentMicros - lastFastLoop) / 1000000.0f;
        lastFastLoop = currentMicros;

        imu.update(dt);

        if (esc.isArmed() && currentThrottle > 1050) {
            // 1. Setpoint Smoothing
            float smoothedRollAngle = targetRollFilter.apply(targetRollAngle, dt);
            float smoothedPitchAngle = targetPitchFilter.apply(targetPitchAngle, dt);
            float smoothedYawRate = targetYawFilter.apply(targetYawRate, dt);

            // 2. Outer Loop (Angle PID) -> scoate Target Rate
            float targetRollRate = rollAnglePID.compute(smoothedRollAngle, imu.getRoll(), dt);
            float targetPitchRate = pitchAnglePID.compute(smoothedPitchAngle, imu.getPitch(), dt);

            // 3. Inner Loop (Rate PID) -> scoate Corectie PWM
            float rollCorr = rollRatePID.compute(targetRollRate, imu.getGyroX(), dt);
            float pitchCorr = pitchRatePID.compute(targetPitchRate, imu.getGyroY(), dt);
            float yawCorr = yawRatePID.compute(smoothedYawRate, imu.getGyroZ(), dt);

            // 4. Mixaj Quad-X si comanda ESC
            MotorOutput motors = MotorMixer::mixQuadX(currentThrottle, rollCorr, pitchCorr, yawCorr);

            m1_pwm = motors.m1;
            m2_pwm = motors.m2;
            m3_pwm = motors.m3;
            m4_pwm = motors.m4;

            esc.writeOutputs(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
        } else {
            m1_pwm = 1000;
            m2_pwm = 1000;
            m3_pwm = 1000;
            m4_pwm = 1000;

            esc.writeOutputs(1000, 1000, 1000, 1000);
            
            // Resetam toate controllerele la dezarmare
            rollAnglePID.reset();
            pitchAnglePID.reset();
            rollRatePID.reset();
            pitchRatePID.reset();
            yawRatePID.reset();

            targetRollFilter.reset(imu.getRoll());
            targetPitchFilter.reset(imu.getPitch());
            targetYawFilter.reset(imu.getGyroZ());
        }
    }

    // ========================================================
    // BUCLA MEDIE: 50 Hz (BAROMETRU)
    // ========================================================
    if (currentMillis - lastMediumLoop >= 20) {
        lastMediumLoop = currentMillis;
        baro.update();
    }

    // ========================================================
    // RECEPTIE LORA (Non-blocking) - SLAVE MODE
    // ========================================================
    lora.update();

    if (lora.available()) {
        uint8_t rxBuf[256];
        size_t rxLen = lora.getPacket(rxBuf, sizeof(rxBuf));
        bool shouldReply = false;

        for (size_t i = 0; i < rxLen; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, rxBuf[i], &msg, &status)) {
                shouldReply = true; // Am primit cel puțin un mesaj valid
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                        mavlink_manual_control_t manual;
                        mavlink_msg_manual_control_decode(&msg, &manual);

                        currentThrottle = manual.z;
                        if(currentThrottle < 1000) currentThrottle = 1000;
                        if(currentThrottle > 2000) currentThrottle = 2000;

                        targetPitchAngle = (manual.x / 1000.0f) * 30.0f;
                        targetRollAngle  = (manual.y / 1000.0f) * 30.0f;
                        targetYawRate    = (manual.r / 1000.0f) * 150.0f; 

                        uint16_t buttons = manual.buttons;

                        if (buttons & (1 << 1)) {
                            if (esc.isArmed()) esc.setArmed(false);
                        } else if (buttons & (1 << 0)) {
                            if (!esc.isArmed()) esc.setArmed(true);
                        }

                        if (buttons & (1 << 2)) {
                            if (!esc.isArmed()) {
                                // În acest mod am putea trimite direct textul, deși e mai curat sa punem în buffer
                                imu.calibrate();
                                baro.setGroundZero();
                            }
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                        if (!esc.isArmed()) sendAllParams();
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_SET: {
                        // Securitate critica de zbor: Nu permitem modificarea PID-urilor daca drona este armata!
                        if (esc.isArmed()) {
                            break;
                        }

                        mavlink_param_set_t param_set;
                        mavlink_msg_param_set_decode(&msg, &param_set);

                        char param_id[17];
                        strncpy(param_id, param_set.param_id, 16);
                        param_id[16] = '\0';
                        
                        float val = param_set.param_value;
                        int idx = -1;

                        if (strcmp(param_id, "RATE_ROLL_P") == 0) { rollRatePID.setGains(val, rollRatePID.getI(), rollRatePID.getD()); idx = 0; }
                        else if (strcmp(param_id, "RATE_ROLL_I") == 0) { rollRatePID.setGains(rollRatePID.getP(), val, rollRatePID.getD()); idx = 1; }
                        else if (strcmp(param_id, "RATE_ROLL_D") == 0) { rollRatePID.setGains(rollRatePID.getP(), rollRatePID.getI(), val); idx = 2; }
                        else if (strcmp(param_id, "RATE_PITCH_P") == 0) { pitchRatePID.setGains(val, pitchRatePID.getI(), pitchRatePID.getD()); idx = 3; }
                        else if (strcmp(param_id, "RATE_PITCH_I") == 0) { pitchRatePID.setGains(pitchRatePID.getP(), val, pitchRatePID.getD()); idx = 4; }
                        else if (strcmp(param_id, "RATE_PITCH_D") == 0) { pitchRatePID.setGains(pitchRatePID.getP(), pitchRatePID.getI(), val); idx = 5; }
                        else if (strcmp(param_id, "RATE_YAW_P") == 0) { yawRatePID.setGains(val, yawRatePID.getI(), yawRatePID.getD()); idx = 6; }
                        else if (strcmp(param_id, "RATE_YAW_I") == 0) { yawRatePID.setGains(yawRatePID.getP(), val, yawRatePID.getD()); idx = 7; }
                        else if (strcmp(param_id, "RATE_YAW_D") == 0) { yawRatePID.setGains(yawRatePID.getP(), yawRatePID.getI(), val); idx = 8; }
                        else if (strcmp(param_id, "ANG_ROLL_P") == 0) { rollAnglePID.setGains(val, rollAnglePID.getI(), rollAnglePID.getD()); idx = 9; }
                        else if (strcmp(param_id, "ANG_PITCH_P") == 0) { pitchAnglePID.setGains(val, pitchAnglePID.getI(), pitchAnglePID.getD()); idx = 10; }

                        if (idx != -1) {
                            strncpy(lastUpdatedParamId, param_id, 16);
                            lastUpdatedParamVal = val;
                            lastUpdatedParamIdx = idx;
                        }
                        break;
                    }
                }
            }
        }

        // ========================================================
        // DACA AM PRIMIT UN PACHET GCS, RĂSPUNDEM IMEDIAT CU TELEMETRIE
        // ========================================================
        if (shouldReply) {
            uint8_t txBuf[256];
            uint16_t offset = 0;
            mavlink_message_t txMsg;

            uint8_t base_mode = esc.isArmed() ? 128 : 1;
            mavlink_msg_heartbeat_pack(system_id, component_id, &txMsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, base_mode, 0, MAV_STATE_ACTIVE);
            offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

            float roll_rad  = imu.getRoll() * (PI / 180.0f);
            float pitch_rad = imu.getPitch() * (PI / 180.0f);
            mavlink_msg_attitude_pack(system_id, component_id, &txMsg, millis(), roll_rad, pitch_rad, 0.0f, 0, 0, 0);
            offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

            float currentAlt = baro.getRelativeAltitude();
            mavlink_msg_vfr_hud_pack(system_id, component_id, &txMsg, 0.0f, 0.0f, 0, currentThrottle, currentAlt, 0.0f);
            offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

            mavlink_msg_servo_output_raw_pack(system_id, component_id, &txMsg, micros(), 0, m1_pwm, m2_pwm, m3_pwm, m4_pwm,
                                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

            // Daca s-a actualizat un parametru, il includem direct in raspunsul unic de telemetrie
            if (lastUpdatedParamIdx != -1) {
                mavlink_param_value_t param_val;
                memset(&param_val, 0, sizeof(param_val));
                strncpy(param_val.param_id, lastUpdatedParamId, 16);
                param_val.param_value = lastUpdatedParamVal;
                param_val.param_type = MAV_PARAM_TYPE_REAL32;
                param_val.param_count = 11;
                param_val.param_index = lastUpdatedParamIdx;

                mavlink_msg_param_value_encode(system_id, component_id, &txMsg, &param_val);
                offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);
                lastUpdatedParamIdx = -1;
            }

            lora.sendPacket(txBuf, offset);
        }
    }
}