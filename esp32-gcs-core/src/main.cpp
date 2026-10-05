#define SDL_MAIN_HANDLED
#include <iostream>
#include <chrono>
#include <cmath>
#include "SerialPort.hpp"
#include "Telemetry.hpp"
#include "MavlinkManager.hpp"
#include "GuiManager.hpp"
#include "PIDConfig.hpp"
#include <vector>
#include <string>
#include <utility>

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
    bool pendingRequestPids = false;
    std::vector<std::pair<std::string, float>> pidSendQueue;

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

        if (input.requestPids) {
            pendingRequestPids = true;
        }

        if (input.sendPids) {
            if (tele.isArmed) {
                std::cout << "\n[BLOCAT] SIGURANTA: Drona este ARMATA! Nu se pot modifica parametrii PID in zbor." << std::endl;
                std::cout << "[BLOCAT] Dezarmeaza drona pentru a putea trimite noii parametri.\n" << std::endl;
            } else {
                pidSendQueue.clear();
                pidSendQueue.push_back({"RATE_ROLL_P", pidConfig.rollP});
                pidSendQueue.push_back({"RATE_ROLL_I", pidConfig.rollI});
                pidSendQueue.push_back({"RATE_ROLL_D", pidConfig.rollD});
                pidSendQueue.push_back({"RATE_PITCH_P", pidConfig.pitchP});
                pidSendQueue.push_back({"RATE_PITCH_I", pidConfig.pitchI});
                pidSendQueue.push_back({"RATE_PITCH_D", pidConfig.pitchD});
                pidSendQueue.push_back({"RATE_YAW_P", pidConfig.yawP});
                pidSendQueue.push_back({"RATE_YAW_I", pidConfig.yawI});
                pidSendQueue.push_back({"RATE_YAW_D", pidConfig.yawD});
                std::cout << "[GCS] PID-urile au fost adaugate in coada de transmisie (" << pidSendQueue.size() << " parametri)." << std::endl;
            }
        }

        // Daca drona s-a armat intre timp, curatam orice comanda pendinte de PID
        if (tele.isArmed && !pidSendQueue.empty()) {
            pidSendQueue.clear();
            std::cout << "[GCS] Coada de transmitere PID a fost anulata automat deoarece drona s-a armat!" << std::endl;
        }
        
        // =========================================================
        // 3.5 & 4. MASTER POLLING (GCS TRIMITE COMENZI LA INTERVAL FIX)
        // =========================================================
        if (elapsedTime >= 200) { // Polling la fiecare 200ms
            // Trimitem comanda de control ca pachet "master"
            mavlink.sendManualControl(pitchOut, rollOut, currentThrottle, currentYaw, latchedButtonsMask);
            
            // Resetăm butoanele apăsate
            latchedButtonsMask = 0;
            
            // Trimitem cerere de parametri dacă s-a solicitat (doar daca drona este dezarmata)
            if (pendingRequestPids) {
                if (!tele.isArmed) {
                    mavlink.sendParamRequestList();
                } else {
                    std::cout << "[GCS] Ignorat REQUEST PIDS: drona este armata!" << std::endl;
                }
                pendingRequestPids = false;
            }

            // Trimitem maxim 2 parametri pe ciclu DOAR daca drona este DEZARMATA
            int sentCount = 0;
            while (!tele.isArmed && !pidSendQueue.empty() && sentCount < 2) {
                auto& p = pidSendQueue.front();
                mavlink.sendParamSet(p.first.c_str(), p.second);
                pidSendQueue.erase(pidSendQueue.begin());
                sentCount++;
            }
            
            waitingForTelemetry = true;
            lastSendTime = currentTime;
        }

        // =========================================================
        // 5. VERIFICARE TIMEOUT
        // =========================================================
        if (elapsedTime >= 1000) {
            if (waitingForTelemetry) {
                std::cout << "[AVERTISMENT] Conexiune pierdută! Drona nu a răspuns la polling." << std::endl;
                waitingForTelemetry = false;
            }
        }

        prevInput = input;
        gui.render(tele, serial.isConnected(), pidConfig, input);

        SDL_Delay(10);
    }
    
    // Salvare PID la final (după apăsarea pe ESC)
    pidConfig.saveToFile("D:/DroneGroundStation/PID Output/pid_config.txt");

    return 0;
}