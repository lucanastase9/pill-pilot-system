#include "PIDController.hpp"
#include <Arduino.h>

PIDController::PIDController(float p, float i, float d, float i_limit, float min_out, float max_out)
    : kp(p), ki(i), kd(d), kf(0.0f), integral(0.0f), previousMeasured(0.0f), previousError(0.0f),
      iMax(i_limit), integralLimit(0.0f), outputMin(min_out), outputMax(max_out), deadband(0.0f) {
    if (ki > 0.0f) {
        integralLimit = iMax / ki;
    }
}


float PIDController::compute(float setpoint, float measuredValue, float dt) {
    if (dt < 0.001f) return 0.0f;

    float error = setpoint - measuredValue;
    
    // Deadband
    if (abs(error) < deadband) {
        error = 0.0f;
    }

    // 1. Termenul Proporțional (P)
    float pTerm = kp * error;

    // 2. Termenul Integrator (I) cu protecție Anti-Windup optimizată (regula trapezului)
    integral += (error + previousError) * 0.5f * dt;
    previousError = error;
    if (ki > 0.0f) {
        integral = constrain(integral, -integralLimit, integralLimit);
    }
    float iTerm = ki * integral;

    // 3. Termenul Derivativ (D) calculat pe măsurătoare pentru a evita Derivative Kick
    float derivative = -(measuredValue - previousMeasured) / dt;
    float dTerm = kd * derivative;
    previousMeasured = measuredValue;
    
    // 4. Termenul Feedforward (KF)
    float ffTerm = kf * setpoint;

    // Suma componentelor și limitarea finală a comenzii
    float output = pTerm + iTerm + dTerm + ffTerm;
    return constrain(output, outputMin, outputMax);
}

void PIDController::reset() {
    integral = 0.0f;
    previousMeasured = 0.0f;
    previousError = 0.0f;
}

float PIDController::getP() const { return kp; }
float PIDController::getI() const { return ki; }
float PIDController::getD() const { return kd; }


void PIDController::setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    if (ki > 0.0f) {
        integralLimit = iMax / ki;
    }
}

void PIDController::setFeedforward(float f) {
    kf = f;
}

void PIDController::setDeadband(float d) {
    deadband = d;
}