#ifndef FILTER_HPP
#define FILTER_HPP

#include <math.h>

class PT1Filter {
private:
    float state;
    float RC;

public:
    PT1Filter() : state(0.0f), RC(0.0f) {}

    // Initialize with a cutoff frequency in Hz
    void setCutoffFreq(float cutoff_freq) {
        if (cutoff_freq > 0.0f) {
            RC = 1.0f / (2.0f * M_PI * cutoff_freq);
        } else {
            RC = 0.0f;
        }
    }

    // Apply the filter to a new sample. dt is the time since the last sample in seconds.
    float apply(float sample, float dt) {
        if (RC == 0.0f || dt <= 0.0f) {
            return sample; // Filter is disabled or invalid dt
        }
        
        float alpha = dt / (RC + dt);
        state = state + alpha * (sample - state);
        return state;
    }

    // Reset the filter state (e.g. on arming or initialization)
    void reset(float val = 0.0f) {
        state = val;
    }
    
    float getState() const {
        return state;
    }
};

#endif // FILTER_HPP
