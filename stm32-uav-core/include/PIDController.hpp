#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
private:
    float kp, ki, kd, kf;
    float integral;
    float previousMeasured;
    float previousError;
    float iMax;          // Limita Anti-Windup teoretică
    float integralLimit; // Limita precalculată (iMax / ki)
    float outputMin;     // Limita minima de ieșire
    float outputMax;     // Limita maxima de ieșire
    float deadband;      // Zona moartă pentru eroare


public:
    PIDController(float p, float i, float d, float i_limit, float min_out, float max_out);

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
    
    // Setează termenul Feedforward
    void setFeedforward(float f);

    // Setează zona moartă (deadband)
    void setDeadband(float d);

};

#endif