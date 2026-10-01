#define SDL_MAIN_HANDLED
#include <iostream>
#include <chrono>
#include <cmath>
#include "SerialPort.hpp"
#include "Telemetry.hpp"
#include "MavlinkManager.hpp"
#include "GuiManager.hpp"
#include "PIDConfig.hpp"

int main(int argc, char* argv[]) {
    std::cout << "=== DRONE GROUND STATION START ===" << std::endl;

    const char* defaultPort = "COM4";
#ifndef _WIN32
    defaultPort = "/dev/ttyUSB0";
#endif
    const char* portName = (argc > 1) ? argv[1] : defaultPort;

    SerialPort serial(portName);

    if (!serial.isConnected()) {
        std::cout << "[AVERTISMENT] Portul " << portName << " nu s-a deschis. Offline Mode." << std::endl;
    } else {
        std::cout << "[OK] Conectat cu succes pe " << portName << "!" << std::endl;
    }

    MavlinkManager mavlink(serial);
    GuiManager gui(900, 650);

    if (!gui.init("GCS Master Control")) return -1;

    Telemetry tele;
    PIDConfig pidConfig;
    bool running = true;

    int currentThrottle = 1000;
    int currentYaw = 0;

    auto lastSendTime = std::chrono::steady_clock::now();
    InputState prevInput;

    const int HEARTBEAT_INTERVAL_MS = 100; // Trimitem un singur pachet exact la 1 secundă

    int lastSentThrottle = -1;
    int lastSentYaw = -9999;
    int lastSentPitch = -9999;
    int lastSentRoll = -9999;
    const int CHANGE_THRESHOLD = 5;

    // -------- FLAG PENTRU RECEPTIE --------
    bool waitingForTelemetry = false;

    // -------- MASCA PENTRU MEMORAREA BUTOANELOR (LATCHING) --------
    uint16_t latchedButtonsMask = 0;

    while (running) {
        InputState input = gui.processEvents(pidConfig);
        if (input.quit) running = false;

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastSendTime).count();

        // =========================================================
        // 1. AȘTEPTARE ȘI RECEPȚIE (ASCULTĂM DRONA)
        // =========================================================
        bool gotTelemetry = mavlink.update(tele, pidConfig);

        if (gotTelemetry) {
            if (waitingForTelemetry) {
                std::cout << "[GCS RX] <- Răspuns telemetrie primit!" << std::endl;
                waitingForTelemetry = false;
            }
        }

        // =========================================================
        // 2. CALCUL VALORI MANȘE (INPUT CONTINUU)
        // =========================================================
        if (input.throttleUp)   currentThrottle += 5;
        if (input.throttleDown) currentThrottle -= 5;
        if (input.yawRight)     currentYaw += 20;
        if (input.yawLeft)      currentYaw -= 20;

        if (currentThrottle > 2000) currentThrottle = 2000;
        if (currentThrottle < 1000) currentThrottle = 1000;

        if (!input.yawLeft && !input.yawRight) {
            if (currentYaw > 0) currentYaw = std::max(0, currentYaw - 25);
            if (currentYaw < 0) currentYaw = std::min(0, currentYaw + 25);
        }
        if (currentYaw > 1000) currentYaw = 1000;
        if (currentYaw < -1000) currentYaw = -1000;

        int pitchOut = 0;
        int rollOut = 0;
        if (std::abs(input.pitch) > 3000) pitchOut = (input.pitch * 1000) / 32767;
        if (std::abs(input.roll) > 3000)  rollOut = (input.roll * 1000) / 32767;

        // =========================================================
        // 3. MEMORAREA APĂSĂRILOR (EDGE DETECTION + SMART TOGGLE)
        // =========================================================
        // Tranziție: înregistrăm comanda DOAR când butonul trece din neatins în apăsat
        bool armJustPressed   = (input.arm && !prevInput.arm);
        bool killJustPressed  = (input.kill && !prevInput.kill);
        bool calibJustPressed = (input.calibrate && !prevInput.calibrate);

        if (armJustPressed) {
            if (tele.isArmed) {
                // Dacă drona e deja armată, butonul acționează ca DEZARMARE
                // Trimitem bitul 1 (care pe dronă dezactivează motoarele)
                latchedButtonsMask |= (1 << 1);
                std::cout << "[GCS] Buton ARM apăsat -> Solicitare DEZARMARE trimisă în coadă." << std::endl;
            } else {
                // Dacă drona e dezarmată, trimitem comanda normală de ARMARE
                latchedButtonsMask |= (1 << 0);
                std::cout << "[GCS] Buton ARM apăsat -> Soldeicitare ARMARE trimisă în coadă." << std::endl;
            }
        }

        // Butonul fizic dedicat pentru KILL / Urgență își păstrează funcția
        if (killJustPressed) {
            latchedButtonsMask |= (1 << 1);
        }

        if (calibJustPressed) {
            latchedButtonsMask |= (1 << 2);
        }
        
        // =========================================================
        // 3.5. PROCESARE CERERI PID
        // =========================================================
        if (input.requestPids) {
            mavlink.sendParamRequestList();
        }
        
        if (input.sendPids) {
            mavlink.sendParamSet("ROLL_P", pidConfig.rollP);
            mavlink.sendParamSet("ROLL_I", pidConfig.rollI);
            mavlink.sendParamSet("ROLL_D", pidConfig.rollD);
            mavlink.sendParamSet("PITCH_P", pidConfig.pitchP);
            mavlink.sendParamSet("PITCH_I", pidConfig.pitchI);
            mavlink.sendParamSet("PITCH_D", pidConfig.pitchD);
            mavlink.sendParamSet("YAW_P", pidConfig.yawP);
            mavlink.sendParamSet("YAW_I", pidConfig.yawI);
            mavlink.sendParamSet("YAW_D", pidConfig.yawD);
        }

        // =========================================================
        // 4. TRANSMISIE STRICTĂ (1 SINGUR PACHET / SECUNDĂ)
        // =========================================================
        if (elapsedTime >= HEARTBEAT_INTERVAL_MS) {

            if (waitingForTelemetry) {
                std::cout << "[AVERTISMENT] Timeout! Pachet pierdut pe LoRa în ultima secundă." << std::endl;
            }

            // Trimitem pachetul folosind masca memoriată pe parcursul secundei
            mavlink.sendManualControl(pitchOut, rollOut, currentThrottle, currentYaw, latchedButtonsMask);
            std::cout << "[GCS TX] -> Comandă trimisă. Butoane salvate: " << latchedButtonsMask << ". Trec în modul ASCULTARE..." << std::endl;

            // CRITIC: Resetăm masca de butoane DUPĂ ce am trimis-o!
            // Altfel, am trimite comanda de calibrare sau armare la infinit.
            latchedButtonsMask = 0;

            waitingForTelemetry = true;
            lastSendTime = currentTime;
        }

        prevInput = input;
        gui.render(tele, serial.isConnected(), pidConfig, input);

        SDL_Delay(10);
    }
    
    // Salvare PID la final (după apăsarea pe ESC)
    pidConfig.saveToFile("D:/DroneGroundStation/PID Output/pid_config.txt");

    return 0;
}