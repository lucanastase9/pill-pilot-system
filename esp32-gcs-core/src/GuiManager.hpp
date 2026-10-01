#ifndef GUI_MANAGER_HPP
#define GUI_MANAGER_HPP

#include <SDL.h>
#include <string>
#include "Telemetry.hpp"
#include "InputState.hpp"
#include "PIDConfig.hpp"

enum class UserAction {
    NONE,
    TOGGLE_ARM,
    THROTTLE_UP,
    THROTTLE_DOWN,
    CALIBRATE,
    QUIT
};

class GuiManager {
private:
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    int windowWidth;
    int windowHeight;
    SDL_Joystick* joystick = nullptr;

    // Timer pentru a menține butonul de calibrare roșu 4 secunde
    Uint32 calibrateEndTime = 0;

public:
    GuiManager(int width = 900, int height = 650);
    ~GuiManager();

    bool init(const char* title);
    InputState processEvents(PIDConfig& pidConfig);

    void drawRect(int x, int y, int w, int h, Uint8 r, Uint8 g, Uint8 b, bool filled = true);
    void drawChar(char c, int x, int y, int scale, Uint8 r, Uint8 g, Uint8 b);
    void drawString(const std::string& text, int x, int y, int scale, Uint8 r, Uint8 g, Uint8 b);
    void drawHeader(bool isArmed, bool isConnected);
    void drawPrimaryFlightDisplay(float roll, float pitch, float altitude);

    // Funcție actualizată pentru a desena toate cele 5 bare
    void drawThrottleAndMotors(const Telemetry& tele);

    void drawTelemetryCard(const Telemetry& tele, bool isConnected);
    void drawKeyGuide();
    void drawPIDTuningCard(PIDConfig& pidConfig, const InputState& input);
    
    void render(const Telemetry& tele, bool isConnected, PIDConfig& pidConfig, const InputState& input);
    
    int activeTextBox = -1; // -1 none, 0-8 for P, I, D of Roll, Pitch, Yaw
};

#endif