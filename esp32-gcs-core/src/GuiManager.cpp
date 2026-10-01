#include "GuiManager.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>

GuiManager::GuiManager(int width, int height)
    : windowWidth(width), windowHeight(height) {}

GuiManager::~GuiManager() {
    if (joystick && SDL_JoystickGetAttached(joystick)) {
        SDL_JoystickClose(joystick);
        joystick = nullptr;
    }
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
}

bool GuiManager::init(const char* title) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0) {
        std::cout << "Eroare la initializarea SDL: " << SDL_GetError() << std::endl;
        return false;
    }

    window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                 windowWidth, windowHeight, SDL_WINDOW_SHOWN);
    if (!window) return false;

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) return false;

    if (SDL_NumJoysticks() > 0) {
        joystick = SDL_JoystickOpen(0);
        if (joystick) {
            std::cout << "Controller conectat cu succes: " << SDL_JoystickName(joystick) << std::endl;
        } else {
            std::cout << "Eroare la deschiderea controller-ului: " << SDL_GetError() << std::endl;
        }
    } else {
        std::cout << "Atentie: Niciun controller/joystick nu a fost detectat!" << std::endl;
    }

    return true;
}

void GuiManager::drawRect(int x, int y, int w, int h, Uint8 r, Uint8 g, Uint8 b, bool filled) {
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
    SDL_Rect rect = {x, y, w, h};
    if (filled) {
        SDL_RenderFillRect(renderer, &rect);
    } else {
        SDL_RenderDrawRect(renderer, &rect);
    }
}

void GuiManager::drawChar(char c, int x, int y, int scale, Uint8 r, Uint8 g, Uint8 b) {
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);

    static uint8_t font5x7[128][7] = {0};
    static bool fontInitialized = false;

    if (!fontInitialized) {
        const uint8_t n0[] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}; std::copy(std::begin(n0), std::end(n0), font5x7['0']);
        const uint8_t n1[] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}; std::copy(std::begin(n1), std::end(n1), font5x7['1']);
        const uint8_t n2[] = {0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F}; std::copy(std::begin(n2), std::end(n2), font5x7['2']);
        const uint8_t n3[] = {0x1F, 0x02, 0x04, 0x06, 0x01, 0x11, 0x0E}; std::copy(std::begin(n3), std::end(n3), font5x7['3']);
        const uint8_t n4[] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}; std::copy(std::begin(n4), std::end(n4), font5x7['4']);
        const uint8_t n5[] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}; std::copy(std::begin(n5), std::end(n5), font5x7['5']);
        const uint8_t n6[] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}; std::copy(std::begin(n6), std::end(n6), font5x7['6']);
        const uint8_t n7[] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}; std::copy(std::begin(n7), std::end(n7), font5x7['7']);
        const uint8_t n8[] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}; std::copy(std::begin(n8), std::end(n8), font5x7['8']);
        const uint8_t n9[] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}; std::copy(std::begin(n9), std::end(n9), font5x7['9']);

        const uint8_t lA[] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}; std::copy(std::begin(lA), std::end(lA), font5x7['A']);
        const uint8_t lB[] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}; std::copy(std::begin(lB), std::end(lB), font5x7['B']);
        const uint8_t lC[] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}; std::copy(std::begin(lC), std::end(lC), font5x7['C']);
        const uint8_t lD[] = {0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C}; std::copy(std::begin(lD), std::end(lD), font5x7['D']);
        const uint8_t lE[] = {0x1F, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x1F}; std::copy(std::begin(lE), std::end(lE), font5x7['E']);
        const uint8_t lF[] = {0x1F, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10}; std::copy(std::begin(lF), std::end(lF), font5x7['F']);
        const uint8_t lG[] = {0x0E, 0x11, 0x10, 0x13, 0x11, 0x11, 0x0F}; std::copy(std::begin(lG), std::end(lG), font5x7['G']);
        const uint8_t lH[] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}; std::copy(std::begin(lH), std::end(lH), font5x7['H']);
        const uint8_t lI[] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}; std::copy(std::begin(lI), std::end(lI), font5x7['I']);
        const uint8_t lK[] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}; std::copy(std::begin(lK), std::end(lK), font5x7['K']);
        const uint8_t lL[] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}; std::copy(std::begin(lL), std::end(lL), font5x7['L']);
        const uint8_t lM[] = {0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11}; std::copy(std::begin(lM), std::end(lM), font5x7['M']);
        const uint8_t lN[] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}; std::copy(std::begin(lN), std::end(lN), font5x7['N']);
        const uint8_t lO[] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}; std::copy(std::begin(lO), std::end(lO), font5x7['O']);
        const uint8_t lP[] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}; std::copy(std::begin(lP), std::end(lP), font5x7['P']);
        const uint8_t lR[] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}; std::copy(std::begin(lR), std::end(lR), font5x7['R']);
        const uint8_t lS[] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}; std::copy(std::begin(lS), std::end(lS), font5x7['S']);
        const uint8_t lT[] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}; std::copy(std::begin(lT), std::end(lT), font5x7['T']);
        const uint8_t lU[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}; std::copy(std::begin(lU), std::end(lU), font5x7['U']);
        const uint8_t lV[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}; std::copy(std::begin(lV), std::end(lV), font5x7['V']);
        const uint8_t lW[] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}; std::copy(std::begin(lW), std::end(lW), font5x7['W']);
        const uint8_t lX[] = {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}; std::copy(std::begin(lX), std::end(lX), font5x7['X']);
        const uint8_t lY[] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}; std::copy(std::begin(lY), std::end(lY), font5x7['Y']);
        const uint8_t lZ[] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}; std::copy(std::begin(lZ), std::end(lZ), font5x7['Z']);

        const uint8_t sMin[] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}; std::copy(std::begin(sMin), std::end(sMin), font5x7['-']);
        const uint8_t sPlu[] = {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}; std::copy(std::begin(sPlu), std::end(sPlu), font5x7['+']);
        const uint8_t sCol[] = {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}; std::copy(std::begin(sCol), std::end(sCol), font5x7[':']);
        const uint8_t sDot[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}; std::copy(std::begin(sDot), std::end(sDot), font5x7['.']);
        const uint8_t sLBr[] = {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E}; std::copy(std::begin(sLBr), std::end(sLBr), font5x7['[']);
        const uint8_t sRBr[] = {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E}; std::copy(std::begin(sRBr), std::end(sRBr), font5x7[']']);
        const uint8_t sSla[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00}; std::copy(std::begin(sSla), std::end(sSla), font5x7['/']);
        const uint8_t sSpc[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; std::copy(std::begin(sSpc), std::end(sSpc), font5x7[' ']);

        fontInitialized = true;
    }

    if (c < 0 || c > 127) return;

    if (c >= 'a' && c <= 'z') c -= 32;

    const uint8_t* rows = font5x7[(uint8_t)c];

    for (int r_idx = 0; r_idx < 7; ++r_idx) {
        for (int col = 0; col < 5; ++col) {
            if (rows[r_idx] & (1 << (4 - col))) {
                SDL_Rect p = { x + col * scale, y + r_idx * scale, scale, scale };
                SDL_RenderFillRect(renderer, &p);
            }
        }
    }
}

void GuiManager::drawString(const std::string& text, int x, int y, int scale, Uint8 r, Uint8 g, Uint8 b) {
    int curX = x;
    for (char c : text) {
        drawChar(c, curX, y, scale, r, g, b);
        curX += 6 * scale;
    }
}

void GuiManager::drawHeader(bool isArmed, bool isConnected) {
    drawRect(15, 15, windowWidth - 30, 50, 25, 28, 36);
    drawRect(15, 15, windowWidth - 30, 50, 45, 52, 65, false);

    drawString("GROUND STATION", 30, 31, 2, 0, 210, 255);

    if (isConnected) {
        drawRect(windowWidth - 340, 25, 140, 30, 20, 80, 45);
        drawRect(windowWidth - 340, 25, 140, 30, 40, 180, 80, false);
        drawString("COM: ONLINE", windowWidth - 330, 34, 1, 100, 255, 140);
    } else {
        drawRect(windowWidth - 340, 25, 140, 30, 80, 30, 30);
        drawRect(windowWidth - 340, 25, 140, 30, 200, 60, 60, false);
        drawString("COM: OFFLINE", windowWidth - 335, 34, 1, 255, 100, 100);
    }

    // Buton Calibrate - ROSU CAND E ACTIV TIMP DE 10 SECUNDE
    bool isCalibrating = SDL_GetTicks() < calibrateEndTime;
    if (isCalibrating) {
        drawRect(windowWidth - 500, 25, 140, 30, 200, 40, 40); // ROSU
        drawRect(windowWidth - 500, 25, 140, 30, 255, 80, 80, false);
        drawString("CALIBRATING..", windowWidth - 490, 34, 1, 255, 255, 255);
    } else {
        drawRect(windowWidth - 500, 25, 140, 30, 40, 50, 60); // INACTIV
        drawRect(windowWidth - 500, 25, 140, 30, 80, 90, 100, false);
        drawString("CALIBRATE", windowWidth - 475, 34, 1, 150, 150, 150);
    }

    // Status Armed - VERDE CAND E ARMAT
    if (isArmed) {
        drawRect(windowWidth - 180, 25, 140, 30, 40, 180, 60); // VERDE
        drawRect(windowWidth - 180, 25, 140, 30, 60, 255, 80, false);
        drawString("STATE: ARMED", windowWidth - 173, 34, 1, 255, 255, 255);
    } else {
        drawRect(windowWidth - 180, 25, 140, 30, 30, 40, 50); // INACTIV
        drawRect(windowWidth - 180, 25, 140, 30, 80, 100, 120, false);
        drawString("STATE: DISARM", windowWidth - 175, 34, 1, 150, 170, 190);
    }
}

void GuiManager::drawPrimaryFlightDisplay(float roll, float pitch, float altitude) {
    int pfdX = 180;
    int pfdY = 85;
    int pfdW = 420;
    int pfdH = 420;
    int cx = pfdX + pfdW / 2;
    int cy = pfdY + pfdH / 2;

    SDL_Rect pfdClip = { pfdX, pfdY, pfdW, pfdH };
    SDL_RenderSetClipRect(renderer, &pfdClip);

    // Fundal complet pentru CER (Sky) #13224f (RGB: 19, 34, 79)
    drawRect(pfdX, pfdY, pfdW, pfdH, 19, 34, 79);

    float radRoll = roll * (3.14159265f / 180.0f);
    int pitchPixels = static_cast<int>(pitch * 4.0f);

    // Calculăm poligonul pentru Pământ (Ground) #18330d (RGB: 24, 51, 13)
    float py = static_cast<float>(pitchPixels);
    float cosR = cos(radRoll);
    float sinR = sin(radRoll);

    SDL_Vertex verts[4];
    SDL_Color groundCol = {24, 51, 13, 255};
    
    // Coordonate relative la centru
    float px1 = -1000.0f, py1 = py;
    float px2 =  1000.0f, py2 = py;
    float px3 =  1000.0f, py3 = py + 2000.0f;
    float px4 = -1000.0f, py4 = py + 2000.0f;

    verts[0].position.x = cx + (px1 * cosR - py1 * sinR);
    verts[0].position.y = cy + (px1 * sinR + py1 * cosR);
    verts[0].color = groundCol;

    verts[1].position.x = cx + (px2 * cosR - py2 * sinR);
    verts[1].position.y = cy + (px2 * sinR + py2 * cosR);
    verts[1].color = groundCol;

    verts[2].position.x = cx + (px3 * cosR - py3 * sinR);
    verts[2].position.y = cy + (px3 * sinR + py3 * cosR);
    verts[2].color = groundCol;

    verts[3].position.x = cx + (px4 * cosR - py4 * sinR);
    verts[3].position.y = cy + (px4 * sinR + py4 * cosR);
    verts[3].color = groundCol;

    int indices[6] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(renderer, nullptr, verts, 4, indices, 6);

    // Linia principală de orizont
    int rotX1 = static_cast<int>(verts[0].position.x);
    int rotY1 = static_cast<int>(verts[0].position.y);
    int rotX2 = static_cast<int>(verts[1].position.x);
    int rotY2 = static_cast<int>(verts[1].position.y);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderDrawLine(renderer, rotX1, rotY1, rotX2, rotY2);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 200);
    for (int p = -30; p <= 30; p += 10) {
        if (p == 0) continue;
        int lineY = cy + pitchPixels - static_cast<int>(p * 4.0f);
        int lineWidth = (p % 20 == 0) ? 80 : 40;

        int lx1 = cx - lineWidth / 2;
        int lx2 = cx + lineWidth / 2;

        int rlx1 = cx + static_cast<int>((lx1 - cx) * cos(radRoll) - (lineY - cy) * sin(radRoll));
        int rly1 = cy + static_cast<int>((lx1 - cx) * sin(radRoll) + (lineY - cy) * cos(radRoll));
        int rlx2 = cx + static_cast<int>((lx2 - cx) * cos(radRoll) - (lineY - cy) * sin(radRoll));
        int rly2 = cy + static_cast<int>((lx2 - cx) * sin(radRoll) + (lineY - cy) * cos(radRoll));

        SDL_RenderDrawLine(renderer, rlx1, rly1, rlx2, rly2);
    }

    SDL_RenderSetClipRect(renderer, NULL);

    drawRect(pfdX, pfdY, pfdW, pfdH, 0, 180, 220, false);
    drawRect(pfdX - 1, pfdY - 1, pfdW + 2, pfdH + 2, 40, 50, 65, false);

    drawRect(cx - 50, cy - 2, 35, 4, 255, 140, 0);
    drawRect(cx + 15, cy - 2, 35, 4, 255, 140, 0);
    drawRect(cx - 3, cy - 3, 6, 6, 255, 140, 0);
    
    // === ALTITUDE TAPE PFD RIGHT SIDE (OUTSIDE PFD) ===
    int tapeW = 50;
    int tapeX = pfdX + pfdW + 2; // Mutat in exterior
    int tapeY = pfdY;
    int tapeH = pfdH;

    // Background for altitude tape
    drawRect(tapeX, tapeY, tapeW, tapeH, 20, 25, 30, true);
    drawRect(tapeX, tapeY, tapeW, tapeH, 40, 50, 65, false);

    // Setăm clip doar pentru tape ca să nu iasă numerele afară pe verticală
    SDL_Rect tapeClip = {tapeX, tapeY, tapeW, tapeH};
    SDL_RenderSetClipRect(renderer, &tapeClip);

    // Green vertical line for tape
    SDL_SetRenderDrawColor(renderer, 100, 255, 100, 255);
    SDL_RenderDrawLine(renderer, tapeX, tapeY, tapeX, tapeY + tapeH);

    drawString("ALT", tapeX + 5, tapeY + 5, 1, 100, 255, 100);

    // Ticks and numbers
    int pixelsPerMeter = 5;
    int centerAlt = static_cast<int>(altitude);
    int startAlt = centerAlt - (tapeH / 2) / pixelsPerMeter - 1;
    int endAlt = centerAlt + (tapeH / 2) / pixelsPerMeter + 1;

    for (int alt = startAlt; alt <= endAlt; alt++) {
        int y = cy - static_cast<int>((alt - altitude) * pixelsPerMeter);
        if (alt % 50 == 0) { // Major tick every 50 meters
            SDL_RenderDrawLine(renderer, tapeX, y, tapeX + 10, y);
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", alt);
            drawString(buf, tapeX + 15, y - 5, 1, 255, 255, 255);
        } else if (alt % 10 == 0) { // Minor tick every 10 meters
            SDL_RenderDrawLine(renderer, tapeX, y, tapeX + 5, y);
        }
    }

    // Current altitude box in the middle of tape
    int boxW = 45;
    int boxH = 30;
    int boxX = tapeX + 2;
    int boxY = cy - boxH / 2;
    drawRect(boxX, boxY, boxW, boxH, 10, 25, 40, true);
    drawRect(boxX, boxY, boxW, boxH, 100, 255, 255, false); // Cyan border
    char altBuf[16];
    snprintf(altBuf, sizeof(altBuf), "%.1f", altitude);
    drawString(altBuf, boxX + 5, cy - 5, 1, 255, 255, 255);

    SDL_RenderSetClipRect(renderer, NULL);
}

// ==== PANOU COMPLET NOU PENTRU MANȘĂ ȘI CELE 4 MOTOARE ====
void GuiManager::drawThrottleAndMotors(const Telemetry& tele) {
    int gX = 15; // Mutat putin la stanga pentru a incapea toate 5 barele
    int gY = 85;
    int gW = 155;
    int gH = 420;

    // Card Container
    drawRect(gX, gY, gW, gH, 25, 28, 36);
    drawRect(gX, gY, gW, gH, 45, 52, 65, false);

    drawString("THR & MOTORS", gX + 15, gY + 15, 1, 0, 200, 255);

    // Parametrii barelor
    int barY = gY + 45;
    int barH = 310;
    int barW = 16;
    int gap = 8;
    int startX = gX + 12;

    // Functie lambda rapida pentru a desena o singura bara
    auto drawBar = [&](int x, int val, const char* label, bool isTarget) {
        drawRect(x, barY, barW, barH, 15, 18, 22);
        drawRect(x, barY, barW, barH, 50, 60, 70, false);

        float pct = (val - 1000) / 1000.0f;
        pct = std::clamp(pct, 0.0f, 1.0f);
        int fillH = static_cast<int>(pct * (barH - 4));

        Uint8 r, g, b;
        if (isTarget) {
            r = 100; g = 150; b = 255; // Albastru pt bara principala GCS
        } else {
            r = static_cast<Uint8>(pct * 255);
            g = static_cast<Uint8>((1.0f - pct) * 200 + 55);
            b = 40;
        }

        drawRect(x + 2, barY + barH - 2 - fillH, barW - 4, fillH, r, g, b);
        drawString(label, x + 2, barY + barH + 8, 1, 200, 200, 200);
    };

    // Desenăm cele 4 Motoare Hardware (am eliminat sliderul T)
    startX += 20; // mutăm puțin mai la dreapta pentru a centra
    drawBar(startX + (barW + gap) * 0, tele.m1, "1", false);
    drawBar(startX + (barW + gap) * 1, tele.m2, "2", false);
    drawBar(startX + (barW + gap) * 2, tele.m3, "3", false);
    drawBar(startX + (barW + gap) * 3, tele.m4, "4", false);

    // === LINIUȚA VERDE PENTRU TARGET THROTTLE ===
    float targetPct = (tele.throttle - 1000) / 1000.0f;
    targetPct = std::clamp(targetPct, 0.0f, 1.0f);
    int lineY = barY + barH - 2 - static_cast<int>(targetPct * (barH - 4));

    int endX = startX + (barW + gap) * 3 + barW;

    SDL_SetRenderDrawColor(renderer, 50, 255, 50, 255); // Verde pur aprins
    SDL_RenderDrawLine(renderer, startX - 4, lineY, endX + 4, lineY);
    SDL_RenderDrawLine(renderer, startX - 4, lineY - 1, endX + 4, lineY - 1); // Grosime 2px

    // Gradatii in partea dreapta a grupului de bare
    for (int i = 0; i <= 4; ++i) {
        int markY = barY + barH - (i * (barH / 4));
        SDL_SetRenderDrawColor(renderer, 100, 120, 140, 255);
        SDL_RenderDrawLine(renderer, endX, markY, endX + 4, markY);

        char valStr[10];
        snprintf(valStr, sizeof(valStr), "%d", 1000 + i * 250);
        drawString(valStr, endX + 6, markY - 3, 1, 140, 160, 180);
    }

    // Afisaj Valoare Throttle Principal Jos
    char thrBuf[16];
    snprintf(thrBuf, sizeof(thrBuf), "%d US", tele.throttle);
    drawString(thrBuf, gX + 35, gY + 380, 1, 255, 255, 255);
}

void GuiManager::drawTelemetryCard(const Telemetry& tele, bool isConnected) {
    int cX = 655; // Mutat la dreapta pt a face loc benzii de altitudine
    int cY = 85;
    int cW = windowWidth - cX - 15;
    int cH = 420;

    // Card Container
    drawRect(cX, cY, cW, cH, 25, 28, 36);
    drawRect(cX, cY, cW, cH, 45, 52, 65, false);

    drawString("TELEMETRY DATA", cX + 25, cY + 18, 1, 0, 200, 255);

    // Box Roll
    drawRect(cX + 15, cY + 50, cW - 30, 65, 18, 22, 28);
    drawRect(cX + 15, cY + 50, cW - 30, 65, 40, 48, 60, false);
    drawString("ROLL ANGLE", cX + 25, cY + 60, 1, 120, 140, 160);

    char rollBuf[20];
    snprintf(rollBuf, sizeof(rollBuf), "%+.2f DEG", tele.roll);
    drawString(rollBuf, cX + 25, cY + 82, 2, 255, 255, 255);

    // Box Pitch
    drawRect(cX + 15, cY + 130, cW - 30, 65, 18, 22, 28);
    drawRect(cX + 15, cY + 130, cW - 30, 65, 40, 48, 60, false);
    drawString("PITCH ANGLE", cX + 25, cY + 140, 1, 120, 140, 160);

    char pitchBuf[20];
    snprintf(pitchBuf, sizeof(pitchBuf), "%+.2f DEG", tele.pitch);
    drawString(pitchBuf, cX + 25, cY + 162, 2, 255, 255, 255);

    // Box Altitude
    drawRect(cX + 15, cY + 210, cW - 30, 65, 18, 22, 28);
    drawRect(cX + 15, cY + 210, cW - 30, 65, 40, 48, 60, false);
    drawString("ALTITUDE", cX + 25, cY + 220, 1, 120, 140, 160);

    char altBuf[20];
    snprintf(altBuf, sizeof(altBuf), "%.2f M", tele.altitude);
    drawString(altBuf, cX + 25, cY + 242, 2, 0, 220, 180);

    // Barometru Status Box
    drawRect(cX + 15, cY + 295, cW - 30, 105, 18, 22, 28);
    drawRect(cX + 15, cY + 295, cW - 30, 105, 40, 48, 60, false);

    drawString("SYSTEM STATUS", cX + 25, cY + 308, 1, 120, 140, 160);
    drawString("MAVLINK V2: ACTIVE", cX + 25, cY + 330, 1, 0, 200, 120);
    
    if (isConnected) {
        drawString("RADIO: LORA 868MHZ", cX + 25, cY + 350, 1, 50, 255, 50);
    } else {
        drawString("RADIO: LORA 868MHZ", cX + 25, cY + 350, 1, 255, 50, 50);
    }

    char rssiBuf[32];
    snprintf(rssiBuf, sizeof(rssiBuf), "SIGNAL: %d %%", tele.rssi);

    Uint8 sigR = 255, sigG = 50, sigB = 50; // default red
    if (tele.rssi > 60) {
        sigR = 50; sigG = 255; sigB = 50; // green
    } else if (tele.rssi > 25) {
        sigR = 255; sigG = 200; sigB = 0; // yellow
    }
    
    drawString(rssiBuf, cX + 25, cY + 370, 1, sigR, sigG, sigB);

    // Draw 4 signal bars next to the text
    int barsX = cX + 160;
    int barsY = cY + 377; // aligned with text bottom
    int barSpacing = 8;
    int barWidth = 4;
    
    for (int b = 0; b < 4; ++b) {
        int barHeight = 6 + b * 4; // 6, 10, 14, 18
        bool active = false;
        if (b == 0 && tele.rssi > 5) active = true;
        if (b == 1 && tele.rssi > 25) active = true;
        if (b == 2 && tele.rssi > 50) active = true;
        if (b == 3 && tele.rssi > 75) active = true;
        
        if (active) {
            drawRect(barsX + b * barSpacing, barsY - barHeight, barWidth, barHeight, sigR, sigG, sigB, true);
        } else {
            drawRect(barsX + b * barSpacing, barsY - barHeight, barWidth, barHeight, 40, 50, 60, true);
        }
    }
}

void GuiManager::drawKeyGuide() {
    int gY = windowHeight - 110;

    drawRect(15, gY, windowWidth - 30, 95, 25, 28, 36);
    drawRect(15, gY, windowWidth - 30, 95, 45, 52, 65, false);

    drawString("CONTROLS & KEYBOARD SHORTCUTS", 30, gY + 12, 1, 0, 200, 255);

    drawString("[D]   OPEN DEBUG / PID TUNING", 30, gY + 38, 1, 255, 200, 50);
    drawString("[ESC] EXIT GROUND STATION & SAVE", 30, gY + 60, 1, 220, 100, 100);
}

// ==== RANDAREA PRINCIPALA SCHIMBATA ====
void GuiManager::drawPIDTuningCard(PIDConfig& pidConfig, const InputState& input) {
    int pX = 50, pY = 50, pW = 800, pH = 550;
    drawRect(pX, pY, pW, pH, 30, 35, 45, true);
    drawRect(pX, pY, pW, pH, 60, 150, 255, false);
    
    drawString("PID TUNING DASHBOARD", pX + 20, pY + 20, 2, 255, 200, 50);
    
    // Buton X (Close) in coltul dreapta-sus
    drawRect(pX + pW - 40, pY + 10, 30, 30, 200, 50, 50, true);
    drawRect(pX + pW - 40, pY + 10, 30, 30, 255, 100, 100, false);
    drawChar('X', pX + pW - 32, pY + 17, 2, 255, 255, 255);
    
    // Nume axe
    const char* axes[] = {"ROLL", "PITCH", "YAW"};
    float* pids[] = {&pidConfig.rollP, &pidConfig.rollI, &pidConfig.rollD,
                     &pidConfig.pitchP, &pidConfig.pitchI, &pidConfig.pitchD,
                     &pidConfig.yawP, &pidConfig.yawI, &pidConfig.yawD};
                     
    for(int i = 0; i < 3; i++) {
        int axY = pY + 80 + i * 130;
        drawString(axes[i], pX + 20, axY, 1, 200, 200, 200);
        
        for(int j = 0; j < 3; j++) {
            int idx = i * 3 + j;
            int boxX = pX + 100 + j * 200;
            
            // Labels P, I, D
            char pidLabel = "PID"[j];
            drawChar(pidLabel, boxX, axY, 1, 150, 150, 150);
            
            // Text box
            bool isActive = (activeTextBox == idx);
            drawRect(boxX + 20, axY - 5, 80, 25, isActive ? 60 : 40, isActive ? 60 : 40, isActive ? 80 : 50, true);
            drawRect(boxX + 20, axY - 5, 80, 25, isActive ? 255 : 100, isActive ? 255 : 100, isActive ? 255 : 100, false);
            
            char valStr[16];
            snprintf(valStr, sizeof(valStr), "%.3f", *pids[idx]);
            
            // If active and typing, show input text instead of real value
            if (isActive && input.inputText.length() > 0) {
                drawString(input.inputText, boxX + 25, axY, 1, 255, 255, 0);
            } else if (isActive) {
                drawString(valStr, boxX + 25, axY, 1, 255, 255, 0); // typing cursor style
            } else {
                drawString(valStr, boxX + 25, axY, 1, 255, 255, 255);
            }
            
            // Slider bar
            int sX = boxX;
            int sY = axY + 30;
            int sW = 150;
            int sH = 10;
            drawRect(sX, sY, sW, sH, 20, 20, 20, true);
            drawRect(sX, sY, sW, sH, 100, 100, 100, false);
            
            // Slider knob
            float pct = (*pids[idx]) / 5.0f; // max 5.0
            if(pct > 1.0f) pct = 1.0f;
            if(pct < 0.0f) pct = 0.0f;
            
            int knobX = sX + (int)(pct * sW) - 5;
            drawRect(knobX, sY - 5, 10, 20, 200, 200, 200, true);
        }
    }
    
    // Buttons
    // Request Button
    drawRect(pX + 20, pY + 480, 200, 40, 50, 100, 150, true);
    drawRect(pX + 20, pY + 480, 200, 40, 100, 150, 200, false);
    drawString("REQUEST PIDS (R)", pX + 35, pY + 495, 1, 255, 255, 255);
    
    // Send Button
    drawRect(pX + 240, pY + 480, 200, 40, 150, 50, 50, true);
    drawRect(pX + 240, pY + 480, 200, 40, 200, 100, 100, false);
    drawString("SEND PIDS (ENTER)", pX + 255, pY + 495, 1, 255, 255, 255);
}

void GuiManager::render(const Telemetry& tele, bool isConnected, PIDConfig& pidConfig, const InputState& input) {
    SDL_SetRenderDrawColor(renderer, 15, 17, 22, 255);
    SDL_RenderClear(renderer);

    drawHeader(tele.isArmed, isConnected);
    drawThrottleAndMotors(tele);
    drawPrimaryFlightDisplay(tele.roll, tele.pitch, tele.altitude);
    drawTelemetryCard(tele, isConnected);
    drawKeyGuide();

    if (input.debugMode) {
        drawPIDTuningCard(pidConfig, input);
    }

    SDL_RenderPresent(renderer);
}

InputState GuiManager::processEvents(PIDConfig& pidConfig) {
    static bool debugModeState = false;
    static std::string currentInputText = "";
    
    InputState input;
    input.debugMode = debugModeState;
    SDL_Event event;

    int mx, my;
    Uint32 mouseButtons = SDL_GetMouseState(&mx, &my);
    input.mouseX = mx;
    input.mouseY = my;
    bool isLeftHeld = (mouseButtons & SDL_BUTTON(SDL_BUTTON_LEFT)) != 0;

    // Check sliders drag
    if (debugModeState && isLeftHeld) {
        float* pids[] = {&pidConfig.rollP, &pidConfig.rollI, &pidConfig.rollD,
                         &pidConfig.pitchP, &pidConfig.pitchI, &pidConfig.pitchD,
                         &pidConfig.yawP, &pidConfig.yawI, &pidConfig.yawD};
        int pX = 50, pY = 50;
        for(int i = 0; i < 3; i++) {
            int axY = pY + 80 + i * 130;
            for(int j = 0; j < 3; j++) {
                int idx = i * 3 + j;
                int boxX = pX + 100 + j * 200;
                int sX = boxX;
                int sY = axY + 30;
                int sW = 150;
                
                // If mouse in slider area
                if (mx >= sX && mx <= sX + sW && my >= sY - 10 && my <= sY + 20) {
                    float pct = (float)(mx - sX) / (float)sW;
                    if(pct < 0) pct = 0;
                    if(pct > 1) pct = 1;
                    *pids[idx] = pct * 5.0f; // range 0..5
                }
            }
        }
    }

    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            input.quit = true;
        }
        else if (event.type == SDL_KEYDOWN) {
            if (event.key.keysym.sym == SDLK_ESCAPE) {
                input.quit = true;
            } else if (event.key.keysym.sym == SDLK_d) {
                debugModeState = !debugModeState;
                input.debugMode = debugModeState;
                if(debugModeState) SDL_StartTextInput();
                else SDL_StopTextInput();
                activeTextBox = -1;
            } else if (event.key.keysym.sym == SDLK_r && debugModeState) {
                input.requestPids = true;
            } else if (event.key.keysym.sym == SDLK_BACKSPACE && debugModeState) {
                input.backspacePressed = true;
                if (activeTextBox >= 0 && currentInputText.length() > 0) {
                    currentInputText.pop_back();
                    try {
                        float* pids[] = {&pidConfig.rollP, &pidConfig.rollI, &pidConfig.rollD,
                                         &pidConfig.pitchP, &pidConfig.pitchI, &pidConfig.pitchD,
                                         &pidConfig.yawP, &pidConfig.yawI, &pidConfig.yawD};
                        if (currentInputText.empty() || currentInputText == ".") *pids[activeTextBox] = 0.0f;
                        else *pids[activeTextBox] = std::stof(currentInputText);
                    } catch (...) {}
                }
            } else if ((event.key.keysym.sym == SDLK_RETURN || event.key.keysym.sym == SDLK_KP_ENTER) && debugModeState) {
                input.enterPressed = true;
                if (activeTextBox >= 0) {
                    // Deselectează căsuța curentă
                    activeTextBox = -1;
                    currentInputText = "";
                } else {
                    // Enter without textbox means Send PIDs
                    input.sendPids = true;
                }
            }
        }
        else if (event.type == SDL_TEXTINPUT && debugModeState) {
            if (activeTextBox >= 0) {
                // allow numbers, dot, and comma (convert comma to dot for RO layouts)
                char c = event.text.text[0];
                if (c == ',') c = '.';
                
                if ((c >= '0' && c <= '9') || c == '.') {
                    // Evităm mai multe puncte
                    if (c == '.' && currentInputText.find('.') != std::string::npos) {
                        // ignore
                    } else {
                        currentInputText += c;
                        try {
                            float* pids[] = {&pidConfig.rollP, &pidConfig.rollI, &pidConfig.rollD,
                                             &pidConfig.pitchP, &pidConfig.pitchI, &pidConfig.pitchD,
                                             &pidConfig.yawP, &pidConfig.yawI, &pidConfig.yawD};
                            if (currentInputText.empty() || currentInputText == ".") *pids[activeTextBox] = 0.0f;
                            else *pids[activeTextBox] = std::stof(currentInputText);
                        } catch (...) {}
                    }
                }
            }
        }
        else if (event.type == SDL_MOUSEBUTTONDOWN) {
            if (event.button.button == SDL_BUTTON_LEFT) {
                input.mouseDown = true;
                if (debugModeState) {
                    // Check text boxes
                    int pX = 50, pY = 50;
                    bool clickedBox = false;
                    for(int i = 0; i < 3; i++) {
                        int axY = pY + 80 + i * 130;
                        for(int j = 0; j < 3; j++) {
                            int idx = i * 3 + j;
                            int boxX = pX + 100 + j * 200 + 20;
                            int boxY = axY - 5;
                            if (mx >= boxX && mx <= boxX + 80 && my >= boxY && my <= boxY + 25) {
                                if (activeTextBox != idx) {
                                    activeTextBox = idx;
                                    float* pids[] = {&pidConfig.rollP, &pidConfig.rollI, &pidConfig.rollD,
                                                     &pidConfig.pitchP, &pidConfig.pitchI, &pidConfig.pitchD,
                                                     &pidConfig.yawP, &pidConfig.yawI, &pidConfig.yawD};
                                    char valStr[16];
                                    snprintf(valStr, sizeof(valStr), "%.3f", *pids[idx]);
                                    currentInputText = valStr;
                                }
                                if (event.button.clicks >= 2) {
                                    currentInputText = ""; // Clear pe dublu click
                                }
                                clickedBox = true;
                            }
                        }
                    }
                    if (!clickedBox) {
                        activeTextBox = -1;
                        currentInputText = "";
                    }
                    
                    // Check Buttons
                    // Request Button
                    if (mx >= pX + 20 && mx <= pX + 220 && my >= pY + 480 && my <= pY + 520) {
                        input.requestPids = true;
                    }
                    // Send Button
                    if (mx >= pX + 240 && mx <= pX + 440 && my >= pY + 480 && my <= pY + 520) {
                        input.sendPids = true;
                    }
                    // Close X Button
                    if (mx >= pX + 800 - 40 && mx <= pX + 800 - 10 && my >= pY + 10 && my <= pY + 40) {
                        debugModeState = false;
                        input.debugMode = false;
                        SDL_StopTextInput();
                        activeTextBox = -1;
                    }
                }
            }
        }
        else if (event.type == SDL_MOUSEBUTTONUP) {
            if (event.button.button == SDL_BUTTON_LEFT) {
                input.mouseReleased = true;
            }
        }
        else if (event.type == SDL_JOYDEVICEADDED) {
            if (!joystick) {
                joystick = SDL_JoystickOpen(event.jdevice.which);
                if (joystick) {
                    std::cout << "Controller conectat: " << SDL_JoystickName(joystick) << std::endl;
                }
            }
        }
        else if (event.type == SDL_JOYDEVICEREMOVED) {
            if (joystick && event.jdevice.which == SDL_JoystickInstanceID(joystick)) {
                SDL_JoystickClose(joystick);
                joystick = nullptr;
                std::cout << "Controller deconectat!" << std::endl;
            }
        }
    }

    if (joystick) {
        SDL_JoystickUpdate();

        input.pitch = SDL_JoystickGetAxis(joystick, 0);
        input.roll  = SDL_JoystickGetAxis(joystick, 1);

        input.kill         = SDL_JoystickGetButton(joystick, 0) == 1;
        input.yawLeft      = SDL_JoystickGetButton(joystick, 4) == 1;
        input.yawRight     = SDL_JoystickGetButton(joystick, 5) == 1;
        input.throttleDown = SDL_JoystickGetButton(joystick, 6) == 1;
        input.throttleUp   = SDL_JoystickGetButton(joystick, 7) == 1;
        input.calibrate    = SDL_JoystickGetButton(joystick, 8) == 1;
        input.arm          = SDL_JoystickGetButton(joystick, 9) == 1;
    }

    const Uint8* keyState = SDL_GetKeyboardState(NULL);
    if (keyState[SDL_SCANCODE_A]) input.arm = true;
    if (keyState[SDL_SCANCODE_B]) input.calibrate = true;
    if (keyState[SDL_SCANCODE_W]) input.throttleUp = true;
    if (keyState[SDL_SCANCODE_S]) input.throttleDown = true;

    if (input.calibrate) {
        calibrateEndTime = SDL_GetTicks() + 10000;
    }

    input.inputText = currentInputText; // Propagăm variabila statică către randare
    return input;
}