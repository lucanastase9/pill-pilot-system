#ifndef INPUT_STATE_HPP
#define INPUT_STATE_HPP

#include <string>

struct InputState {
    bool quit = false;

    // Axe
    int pitch = 0; // Axa X (Înainte / Înapoi)
    int roll = 0;  // Axa Y (Stânga / Dreapta)

    // Butoane
    bool arm = false;          // Index 9
    bool calibrate = false;    // Index 8
    bool yawRight = false;     // Index 5
    bool yawLeft = false;      // Index 4
    bool throttleUp = false;   // Index 7
    bool throttleDown = false; // Index 6
    bool kill = false;         // Index 0
    
    // UI - PID Tuning
    bool debugMode = false;
    
    // Mouse
    int mouseX = 0;
    int mouseY = 0;
    bool mouseDown = false;
    bool mouseReleased = false;

    // Text Input
    std::string inputText = "";
    bool backspacePressed = false;
    bool enterPressed = false;
    
    // PID Actions
    bool sendPids = false;
    bool requestPids = false;
};

#endif