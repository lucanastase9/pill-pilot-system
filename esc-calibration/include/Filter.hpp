#ifndef FILTER_HPP
#define FILTER_HPP

#include <math.h>

// Filtru trece-jos de ordinul 1 (PT1 / Low-Pass Filter)
class PT1Filter {
private:
    float state;
    float RC;

public:
    PT1Filter() : state(0.0f), RC(0.0f) {}

    // Inițializare cu frecvența de tăiere în Hz
    void setCutoffFreq(float cutoff_freq) {
        if (cutoff_freq > 0.0f) {
            RC = 1.0f / (2.0f * M_PI * cutoff_freq);
        } else {
            RC = 0.0f;
        }
    }

    // Aplicare filtru la un nou eșantion. dt este timpul în secunde de la ultimul eșantion.
    float apply(float sample, float dt) {
        if (RC == 0.0f || dt <= 0.0f) {
            return sample; // Filtrul este dezactivat sau dt invalid
        }

        float alpha = dt / (RC + dt);
        state = state + alpha * (sample - state);
        return state;
    }

    // Resetare stare filtru
    void reset(float val = 0.0f) {
        state = val;
    }

    float getState() const {
        return state;
    }
};

#endif // FILTER_HPP
