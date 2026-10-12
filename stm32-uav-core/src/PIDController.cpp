#include "PIDController.hpp"
#include <Arduino.h>

PIDController::PIDController(float p, float i, float d, float min_out, float max_out, float d_cutoff_hz)
    : kp(p), ki(i), kd(d), integral(0.0f), previousMeasured(0.0f), previousError(0.0f),
      outputMin(min_out), outputMax(max_out), firstSample(true) {
    dFilter.setCutoffFreq(d_cutoff_hz);
}

float PIDController::compute(float setpoint, float measuredValue, float dt) {
    if (dt < 0.0001f) return 0.0f;
    //calculul erorii
    float error = setpoint - measuredValue;
    //termenul proportional(P)
    float pTerm = kp * error;
    //termenul integral(I)
    float delta_integral = (error + previousError) * 0.5f * dt;
    float tentative_integral = integral + delta_integral;
    float iTerm = ki * tentative_integral;
    previousError = error;
    //termenul derivativ(D)
    float rawDerivative = 0.0f;
    if (!firstSample) {
        rawDerivative = -(measuredValue - previousMeasured) / dt;
    } else {
        firstSample = false;
    }
    float filteredDerivative = dFilter.apply(rawDerivative, dt);
    float dTerm = kd * filteredDerivative;
    previousMeasured = measuredValue;
    //suma iersirii totale
    float output = pTerm + iTerm + dTerm;
    //aplicarea Anti-Windup
    if (output > outputMax){
        if (error < 0.0f) integral = tentative_integral;
        output = outputMax;
    }else if (output < outputMin){
        if (error >= 0.0f) integral = tentative_integral;
        output = outputMin;
    }else{
        integral = tentative_integral;
    }
    return output;
}

void PIDController::reset() {
    integral = 0.0f;
    previousMeasured = 0.0f;
    previousError = 0.0f;
    firstSample = true;
    dFilter.reset(0.0f);
}

float PIDController::getP() const { return kp; }
float PIDController::getI() const { return ki; }
float PIDController::getD() const { return kd; }


void PIDController::setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}