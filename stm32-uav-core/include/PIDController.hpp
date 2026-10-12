#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include "Filter.hpp"

class PIDController {
private:
    float kp, ki, kd;
    float integral;
    float previousMeasured;
    float previousError;
    float outputMin;     // Limita minima de ieșire
    float outputMax;     // Limita maxima de ieșire
    bool  firstSample;
    PT1Filter dFilter;

public:
    PIDController(float p, float i, float d, float min_out, float max_out, float d_cutoff_hz = 35.0f);

    // Calculează ieșirea PID în funcție de Setpoint (țintă) și starea reală
    float compute(float setpoint, float measuredValue, float dt);

    // Resetează acumularea Integratorului (util la dezarmare)
    void reset();

    // Gettere pentru telemetria MAVLink
    float getP() const;
    float getI() const;
    float getD() const;

    // Permite ajustarea dinamică a parametrilor
    void setGains(float p, float i, float d);
};

#endif