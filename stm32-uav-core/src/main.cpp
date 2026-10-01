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

// PID Controllers
PIDController rollPID(1.2f, 0.05f, 0.3f, 100.0f, -400.0f, 400.0f);
PIDController pitchPID(1.2f, 0.05f, 0.3f, 100.0f, -400.0f, 400.0f);
PIDController yawPID(2.0f, 0.0f, 0.1f, 100.0f, -300.0f, 300.0f);

// Filtre pentru Setpoint Smoothing (Netezirea comenzilor de pe butoane)
PT1Filter targetRollFilter;
PT1Filter targetPitchFilter;
PT1Filter targetYawFilter;


// ==========================================
// 2. VARIABILE DE ZBOR
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
    param_val.param_count = 9;
    param_val.param_index = param_index;

    mavlink_msg_param_value_encode(system_id, component_id, &txMsg, &param_val);
    uint16_t len = mavlink_msg_to_send_buffer(txBuf, &txMsg);
    lora.sendPacket(txBuf, len);
}

// Funcție pentru a trimite toți parametrii de PID la cererea aplicației
void sendAllParams() {
    sendSingleParam("ROLL_P", rollPID.getP(), 0);
    sendSingleParam("ROLL_I", rollPID.getI(), 1);
    sendSingleParam("ROLL_D", rollPID.getD(), 2);
    
    sendSingleParam("PITCH_P", pitchPID.getP(), 3);
    sendSingleParam("PITCH_I", pitchPID.getI(), 4);
    sendSingleParam("PITCH_D", pitchPID.getD(), 5);
    
    sendSingleParam("YAW_P", yawPID.getP(), 6);
    sendSingleParam("YAW_I", yawPID.getI(), 7);
    sendSingleParam("YAW_D", yawPID.getD(), 8);
    
    // Serial.println("[MAVLINK] Au fost trimisi cei 9 parametri PID.");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== DRONA STM32 OOP: PREGATITA ===");

    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    Wire.setClock(400000);

    // Initializare setpoint smoothing si deadband/feedforward
    targetRollFilter.setCutoffFreq(5.0f);  // O valoare mică pentru a crea o rampă exponențială
    targetPitchFilter.setCutoffFreq(5.0f);
    targetYawFilter.setCutoffFreq(5.0f);

    rollPID.setFeedforward(0.1f);
    pitchPID.setFeedforward(0.1f);
    yawPID.setFeedforward(0.0f); // Yaw are mai putina nevoie de FF

    rollPID.setDeadband(0.5f);
    pitchPID.setDeadband(0.5f);


    esc.init();
    if (imu.init()) Serial.println("[OK] Senzor BMI160 conectat.");

    if (baro.init()) {
        Serial.println("[OK] Barometru MS5611 conectat. Calibrare...");
        delay(200);
        baro.setGroundZero();
    }

    if (lora.init()) Serial.println("[OK] Modul LoRa SX1262 online.");
}

void loop() {
    // ------------------------------------------------
    // 1. CALCUL TIMP (dt) ȘI CITIRE SENZORI
    // ------------------------------------------------
    static unsigned long lastLoopTime = 0;
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastLoopTime) / 1000000.0f;
    lastLoopTime = currentMicros;

    imu.update(dt);
    baro.update();

    // ------------------------------------------------
    // 2. SIGURANȚĂ ȘI CONTROL MOTOARE
    // ------------------------------------------------
    unsigned long currentMillis = millis();

    // --- DEBUG: Afișare date IMU pe Serial ---
    // (Oprit pentru a nu polua Serial Monitorul)
    // static unsigned long lastPrintTime = 0;
    // if (currentMillis - lastPrintTime >= 200) { ... }

    if (esc.isArmed() && currentThrottle > 1050) {
        // --- Setpoint Smoothing ---
        // Netezim săriturile bruste ale comenzilor date de butoanele joystick-ului
        float smoothedRoll = targetRollFilter.apply(targetRollAngle, dt);
        float smoothedPitch = targetPitchFilter.apply(targetPitchAngle, dt);
        float smoothedYaw = targetYawFilter.apply(targetYawRate, dt);

        float rollCorr = rollPID.compute(smoothedRoll, imu.getRoll(), dt);
        float pitchCorr = pitchPID.compute(smoothedPitch, imu.getPitch(), dt);
        float yawCorr = yawPID.compute(smoothedYaw, imu.getGyroZ(), dt);


        MotorOutput motors = MotorMixer::mixQuadX(currentThrottle, rollCorr, pitchCorr, yawCorr);

        // Salvăm turațiile pentru a le trimite prin telemetrie
        m1_pwm = motors.m1;
        m2_pwm = motors.m2;
        m3_pwm = motors.m3;
        m4_pwm = motors.m4;

        esc.writeOutputs(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
    } else {
        // Resetăm turațiile la nivelul minim
        m1_pwm = 1000;
        m2_pwm = 1000;
        m3_pwm = 1000;
        m4_pwm = 1000;

        esc.writeOutputs(1000, 1000, 1000, 1000);
        rollPID.reset();
        pitchPID.reset();
        yawPID.reset();

        targetRollFilter.reset(imu.getRoll());
        targetPitchFilter.reset(imu.getPitch());
        targetYawFilter.reset(imu.getGyroZ());
    }


    // ------------------------------------------------
    // 3. RECEPȚIE LORA (ASCULTĂM MASTER-UL)
    // ------------------------------------------------
    lora.update();

    bool shouldSendTelemetry = false;

    if (lora.available()) {
        uint8_t rxBuf[256];
        size_t rxLen = lora.getPacket(rxBuf, sizeof(rxBuf));

        for (size_t i = 0; i < rxLen; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, rxBuf[i], &msg, &status)) {
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_MANUAL_CONTROL: {
                        mavlink_manual_control_t manual;
                        mavlink_msg_manual_control_decode(&msg, &manual);

                        currentThrottle = manual.z;
                        if(currentThrottle < 1000) currentThrottle = 1000;
                        if(currentThrottle > 2000) currentThrottle = 2000;

                        targetPitchAngle = (manual.x / 1000.0f) * 30.0f;
                        targetRollAngle  = (manual.y / 1000.0f) * 30.0f;
                        targetYawRate    = (manual.r / 1000.0f) * 50.0f;

                        uint16_t buttons = manual.buttons;

                        if (buttons & (1 << 1)) {
                            if (esc.isArmed()) {
                                esc.setArmed(false);
                                // Serial.println("[DRONA RX] KILL SWITCH ACTIVAT! Drona DEZARMATĂ!");
                            }
                        } else if (buttons & (1 << 0)) {
                            if (!esc.isArmed()) {
                                esc.setArmed(true);
                                // Serial.println("[DRONA RX] Comanda ARM primită -> Drona ARMATĂ!");
                            }
                        }

                        if (buttons & (1 << 2)) {
                            if (!esc.isArmed()) {
                                // TRIMITEM UN MESAJ RAPID CĂTRE PC CĂ INTRĂM ÎN CALIBRARE
                                mavlink_message_t txtMsg;
                                uint8_t txtBuf[128];
                                mavlink_msg_statustext_pack(system_id, component_id, &txtMsg, MAV_SEVERITY_NOTICE, "CALIBRATING IMU...", 0, 0);
                                uint16_t txtLen = mavlink_msg_to_send_buffer(txtBuf, &txtMsg);
                                lora.sendPacket(txtBuf, txtLen);

                                // Blocăm execuția pentru calibrare
                                imu.calibrate();
                                baro.setGroundZero();
                                // Serial.println("[DRONA RX] Comanda CALIBRARE EXECUTATĂ!");
                            } else {
                                // Serial.println("[DRONA RX] Ignorat: Nu pot calibra o dronă armată/în aer.");
                            }
                        }

                        shouldSendTelemetry = true;
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                        if (!esc.isArmed()) {
                            // Serial.println("[DRONA RX] PARAM_REQUEST_LIST primit. Trimit parametri...");
                            sendAllParams();
                        } else {
                            // Serial.println("[DRONA RX] Ignorat PARAM_REQUEST_LIST: Drona este armata!");
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_PARAM_SET: {
                        mavlink_param_set_t param_set;
                        mavlink_msg_param_set_decode(&msg, &param_set);

                        char param_id[17];
                        strncpy(param_id, param_set.param_id, 16);
                        param_id[16] = '\0';
                        
                        float val = param_set.param_value;
                        int idx = -1;

                        if (strcmp(param_id, "ROLL_P") == 0) { rollPID.setGains(val, rollPID.getI(), rollPID.getD()); idx = 0; }
                        else if (strcmp(param_id, "ROLL_I") == 0) { rollPID.setGains(rollPID.getP(), val, rollPID.getD()); idx = 1; }
                        else if (strcmp(param_id, "ROLL_D") == 0) { rollPID.setGains(rollPID.getP(), rollPID.getI(), val); idx = 2; }
                        else if (strcmp(param_id, "PITCH_P") == 0) { pitchPID.setGains(val, pitchPID.getI(), pitchPID.getD()); idx = 3; }
                        else if (strcmp(param_id, "PITCH_I") == 0) { pitchPID.setGains(pitchPID.getP(), val, pitchPID.getD()); idx = 4; }
                        else if (strcmp(param_id, "PITCH_D") == 0) { pitchPID.setGains(pitchPID.getP(), pitchPID.getI(), val); idx = 5; }
                        else if (strcmp(param_id, "YAW_P") == 0) { yawPID.setGains(val, yawPID.getI(), yawPID.getD()); idx = 6; }
                        else if (strcmp(param_id, "YAW_I") == 0) { yawPID.setGains(yawPID.getP(), val, yawPID.getD()); idx = 7; }
                        else if (strcmp(param_id, "YAW_D") == 0) { yawPID.setGains(yawPID.getP(), yawPID.getI(), val); idx = 8; }

                        if (idx != -1) {
                            // Serial.print("[DRONA RX] Parametru actualizat: ");
                            // Serial.print(param_id);
                            // Serial.print(" = ");
                            // Serial.println(val);
                            // Confirmare rapida trimitand valoarea inapoi
                            sendSingleParam(param_id, val, idx);
                        }
                        break;
                    }
                }
            }
        }
    }

    // ------------------------------------------------
    // 4. TRIMITERE TELEMETRIE (PING-PONG)
    // ------------------------------------------------
    if (shouldSendTelemetry) {
        uint8_t txBuf[256];
        uint16_t offset = 0;
        mavlink_message_t txMsg;

        // PACHET 1: HEARTBEAT (Starea de ARMARE este trimisă automat aici în base_mode)
        uint8_t base_mode = esc.isArmed() ? 128 : 1;
        mavlink_msg_heartbeat_pack(system_id, component_id, &txMsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, base_mode, 0, MAV_STATE_ACTIVE);
        offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

        // PACHET 2: ATTITUDE (Roll, Pitch)
        float roll_rad  = imu.getRoll() * (PI / 180.0f);
        float pitch_rad = imu.getPitch() * (PI / 180.0f);
        mavlink_msg_attitude_pack(system_id, component_id, &txMsg, millis(), roll_rad, pitch_rad, 0.0f, 0, 0, 0);
        offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

        // PACHET 3: VFR_HUD (Altitudine, Throttle de pe manșă)
        float currentAlt = baro.getRelativeAltitude();
        mavlink_msg_vfr_hud_pack(system_id, component_id, &txMsg, 0.0f, 0.0f, 0, currentThrottle, currentAlt, 0.0f);
        offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

        // PACHET 4: SERVO_OUTPUT_RAW (Nivelul fizic PWM la care sunt cele 4 motoare!!!)
        mavlink_msg_servo_output_raw_pack(system_id, component_id, &txMsg, micros(), 0, m1_pwm, m2_pwm, m3_pwm, m4_pwm,
                                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        offset += mavlink_msg_to_send_buffer(txBuf + offset, &txMsg);

        lora.sendPacket(txBuf, offset);
    }
}