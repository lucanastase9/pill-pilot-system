#ifndef MOTOR_MIXER_HPP
#define MOTOR_MIXER_HPP

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

        // Geometria standard Quad-X
        out.m1 = throttle - (int)pitchCorr - (int)rollCorr - (int)yawCorr;
        out.m2 = throttle + (int)pitchCorr - (int)rollCorr + (int)yawCorr;
        out.m3 = throttle + (int)pitchCorr + (int)rollCorr - (int)yawCorr;
        out.m4 = throttle - (int)pitchCorr + (int)rollCorr + (int)yawCorr;

        return out;
    }
};

#endif