#ifndef MOTOR_MIXER_HPP
#define MOTOR_MIXER_HPP

#include <math.h>

struct MotorOutput {
    int m1; // Front Right
    int m2; // Rear Right
    int m3; // Rear Left
    int m4; // Front Left
};

class MotorMixer {
public:
    static MotorOutput mixQuadX(int throttle, float rollCorr, float pitchCorr, float yawCorr) {
        MotorOutput out;
        // geometria Quad-X:
        out.m1 = (int)roundf((float)throttle + pitchCorr - rollCorr - yawCorr); // Front Right
        out.m2 = (int)roundf((float)throttle - pitchCorr - rollCorr + yawCorr); // Rear Right
        out.m3 = (int)roundf((float)throttle - pitchCorr + rollCorr - yawCorr); // Rear Left
        out.m4 = (int)roundf((float)throttle + pitchCorr + rollCorr + yawCorr); // Front Left

        return out;
    }
};

#endif