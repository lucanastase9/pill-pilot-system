#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include "IMUManager.hpp"

// ============================================================================
// CONFIGURARE PINI SI CONSTANTE
// ============================================================================
// M1 -> PA0 (Față-Dreapta) - CCW (↺)
// M2 -> PA1 (Spate-Dreapta) - CW (↻)
// M3 -> PA2 (Spate-Stânga)  - CCW (↺)
// M4 -> PA3 (Față-Stânga)   - CW (↻)
const uint8_t MOTOR_PINS[4] = {PA0, PA1, PA2, PA3};
const char* MOTOR_NAMES[4] = {
    "Motor 1 (PA0 - Fata Dreapta | CCW ↺)",
    "Motor 2 (PA1 - Spate Dreapta | CW ↻)",
    "Motor 3 (PA2 - Spate Stanga | CCW ↺)",
    "Motor 4 (PA3 - Fata Stanga | CW ↻)"
};

// Pin CS pentru senzorul BMI160 (SPI1)
#define IMU_CS_PIN PA4

// LED Onboard BlackPill F411CE (PC13 activ pe LOW)
#define LED_PIN PC13
#define LED_ON  LOW
#define LED_OFF HIGH

#define PWM_MAX 2000
#define PWM_MIN 1000

// Turație fixă pentru modul de testare individuală
#define PWM_TEST 1120 

// Turație maximă permisă în testul combinat IMU + Motoare
#define PWM_IMU_TEST_MAX 1500

Servo escMotors[4];
IMUManager imu(IMU_CS_PIN);

// Stările posibile ale programului
enum ProgramMode {
    MODE_SELECT_MENU,
    MODE_MOTOR_TEST,
    MODE_IMU_TEST
};

ProgramMode currentMode = MODE_SELECT_MENU;

// ============================================================================
// FUNCTII HELPER PENTRU MOTOARE SI SERIAL
// ============================================================================
void setAllMotors(int pulse_us) {
    if (pulse_us < PWM_MIN) pulse_us = PWM_MIN;
    if (pulse_us > PWM_MAX) pulse_us = PWM_MAX;
    for (int i = 0; i < 4; i++) {
        escMotors[i].writeMicroseconds(pulse_us);
    }
}

void setSingleMotor(int motorIndex, int pulse_us) {
    for (int i = 0; i < 4; i++) {
        if (i == motorIndex) {
            escMotors[i].writeMicroseconds(pulse_us);
        } else {
            escMotors[i].writeMicroseconds(PWM_MIN);
        }
    }
}

void flushSerial() {
    while (Serial.available()) {
        Serial.read();
        delay(5);
    }
}

void waitForSerialInput() {
    flushSerial();
    while (!Serial.available()) {
        delay(30);
    }
    flushSerial();
}

// Afiseaza un grafic tip bara orizontala cu axa de zero (|) in centru (pozitia pe masa)
void printBarGraph(const char* label, float angle, float maxAngle = 20.0f) {
    const int HALF_WIDTH = 15; // 15 caractere spre stanga (-), 15 spre dreapta (+)
    char bar[HALF_WIDTH * 2 + 2];
    
    // Umplem bara cu spatii/puncte
    for (int i = 0; i < HALF_WIDTH * 2 + 1; i++) {
        bar[i] = '.';
    }
    bar[HALF_WIDTH * 2 + 1] = '\0';
    bar[HALF_WIDTH] = '|'; // Axa X: Punctul de 0.0° (reperul mesei)

    float clamped = angle;
    if (clamped < -maxAngle) clamped = -maxAngle;
    if (clamped > maxAngle) clamped = maxAngle;

    int offset = (int)round((clamped / maxAngle) * HALF_WIDTH);
    if (offset > 0) {
        for (int i = 1; i <= offset && (HALF_WIDTH + i) <= HALF_WIDTH * 2; i++) {
            bar[HALF_WIDTH + i] = '#';
        }
    } else if (offset < 0) {
        for (int i = -1; i >= offset && (HALF_WIDTH + i) >= 0; i--) {
            bar[HALF_WIDTH + i] = '#';
        }
    }

    Serial.print(label);
    Serial.print(" [-20° ");
    Serial.print(bar);
    Serial.print(" +20°] ");
    if (angle >= 0.0f) Serial.print("+");
    Serial.print(angle, 1);
    Serial.println("°");
}

// ============================================================================
// AFISARE MENIURI
// ============================================================================
void printMainMenu() {
    Serial.println("\n========================================================");
    Serial.println("           PILL-PILOT: CONTROL, CALIBRARE & IMU         ");
    Serial.println("========================================================");
    Serial.println("[ATENTIE CRITICA] ELICELE TREBUIE SA FIE DEMONTATE!");
    Serial.println("--------------------------------------------------------");
    Serial.println("ALEGE MODUL DE LUCRU:");
    Serial.println("  [1] -> MODUL DE TEST MOTOARE (1, 2, 3, 4)");
    Serial.println("         (daca ESC-urile sunt deja calibrate si bateria e cuplata)");
    Serial.println("  [2] -> MODUL DE CALIBRARE ESC");
    Serial.println("         (procedura de calibrare max 2000us / min 1000us)");
    Serial.println("  [3] -> MODUL TEST IMU (GRAFICE KALMAN ROLL & PITCH)");
    Serial.println("         (grafice vizuale cu axa 0 pe masa + turatie motoare 1000-1500us)");
    Serial.println("--------------------------------------------------------");
    Serial.println(">>> Trimite '1', '2' sau '3' in Serial...");
}

void printTestMenu() {
    Serial.println("\n--------------------------------------------------------");
    Serial.println(">>> MOD TEST MOTOARE ACTIV (Trimite tasta in Serial):");
    Serial.println("  [1] -> Invartire Motor 1 (PA0 - Fata Dreapta | CCW ↺)");
    Serial.println("  [2] -> Invartire Motor 2 (PA1 - Spate Dreapta | CW ↻)");
    Serial.println("  [3] -> Invartire Motor 3 (PA2 - Spate Stanga | CCW ↺)");
    Serial.println("  [4] -> Invartire Motor 4 (PA3 - Fata Stanga | CW ↻)");
    Serial.println("  [a] -> Invartire TOATE cele 4 motoare simultan");
    Serial.println("  [s] -> STOP urgent (toate la 1000us)");
    Serial.println("  [m] -> Inapoi la MENIUL PRINCIPAL");
    Serial.println("--------------------------------------------------------");
}

// ============================================================================
// PROCEDURA DE CALIBRARE ESC (OPTIONALUL 2)
// ============================================================================
void executeCalibration() {
    Serial.println("\n========================================================");
    Serial.println("             MODUL 2: CALIBRARE ESC SELECTAT            ");
    Serial.println("========================================================");
    Serial.println("[PAS 0] BATERIA TREBUIE SA FIE DECONECTATA DE LA ESC-URI ACUM!");
    Serial.println(">>> Apasa ENTER (sau orice tasta) cand esti gata sa activez 2000us...");
    waitForSerialInput();

    // 1. Trimite semnalul MAX (2000us)
    setAllMotors(PWM_MAX);
    digitalWrite(LED_PIN, LED_ON);

    Serial.println("\n[PAS 1] Semnal MAX (2000us) ACTIVAT pe PA0, PA1, PA2, PA3!");
    Serial.println("  1. CONECTEAZA BATERIA ACUM.");
    Serial.println("  2. Asculta ESC-urile: melodia de start + 2 BIPURI SCURTE.");
    Serial.println(">>> DUPA CE AI AUZIT CELE 2 BIPURI SCURTE, apasa ENTER...");
    waitForSerialInput();

    // 2. Trimite semnalul MIN (1000us)
    Serial.println("\n[PAS 2] Trimit semnal MINIM (1000us)...");
    setAllMotors(PWM_MIN);

    Serial.println("Asculta ESC-urile: confirma salvarea minimului si se armeaza.");
    Serial.println("Astept 3 secunde pentru stabilizare...");
    for (int sec = 3; sec > 0; sec--) {
        Serial.print("  ... ");
        Serial.print(sec);
        Serial.println("s");
        digitalWrite(LED_PIN, LED_OFF);
        delay(500);
        digitalWrite(LED_PIN, LED_ON);
        delay(500);
    }

    Serial.println("\n========================================================");
    Serial.println("         CALIBRARE FINALIZATA CU SUCCES!                ");
    Serial.println("========================================================");
    Serial.println("Trecem automat in Modul de Test pentru verificare!");
    
    currentMode = MODE_MOTOR_TEST;
    printTestMenu();
}

// ============================================================================
// MODUL 3: TEST IMU (GRAFICE KALMAN ROLL & PITCH CU AXA 0 PE MASA)
// ============================================================================
void runIMUTestLoop() {
    setAllMotors(PWM_MIN); // Siguranta: motoarele oprite la intrare

    // Initializare IMU daca nu a fost initializat
    if (!imu.isInitialized()) {
        Serial.println("\n[IMU] Initializare senzor BMI160 pe SPI...");
        if (!imu.init()) {
            Serial.println("[EROARE] Senzorul BMI160 nu a putut fi initializat!");
            currentMode = MODE_SELECT_MENU;
            printMainMenu();
            return;
        }
        Serial.println("[OK] Senzor BMI160 initializat pe SPI!");
    }

    Serial.println("\n========================================================");
    Serial.println(" MODUL 3: GRAFICE ROLL & PITCH (KALMAN) & TEST MOTOARE  ");
    Serial.println("========================================================");
    Serial.println("[ATENTIE] ASIGURA-TE CA ELICELE SUNT DEMONTATE!");
    Serial.println("--------------------------------------------------------");
    Serial.println("REPERE GRAFICE:");
    Serial.println("  '|' reprezinta 0.0° (axa de referinta pe masa)");
    Serial.println("  '#' reprezinta abaterea unghiului (la stanga sau la dreapta)");
    Serial.println("--------------------------------------------------------");
    Serial.println("COMENZI DISPONIBILE DIN SERIAL:");
    Serial.println("  [e] / [d]   : Pas FIN de +/-1us (ex: 1001, 1002... prag pornire)");
    Serial.println("  [w] / [s]   : Pas MARE de +/-25us (sau [+] / [-])");
    Serial.println("  [x] sau [SPACE]: STOP URGENT motoare (revine la 1000us)");
    Serial.println("  [c]         : Calibreaza offset-ul IMU la orizontala (0.0°)");
    Serial.println("  [r] / [f]   : Mareste / Micsoreaza parametrul Kalman R (+/-0.001)");
    Serial.println("  [p]         : Comuta intre Mod Grafic ASCII si Mod Serial Plotter");
    Serial.println("  [m]         : STOP motoare si IESIRE la meniul principal");
    Serial.println("--------------------------------------------------------");
    Serial.println("Pornim cu motoarele OPRITE (1000us). Trimite comenzi oricand:\n");

    int currentThrottle = PWM_MIN;
    bool plotterMode = false;
    unsigned long lastTimeMicros = micros();
    unsigned long lastPrintMillis = 0;

    while (true) {
        // 1. Calcul dt si update IMU
        unsigned long nowMicros = micros();
        float dt = (nowMicros - lastTimeMicros) / 1000000.0f;
        if (dt <= 0.0f || dt > 0.05f) dt = 0.004f;
        lastTimeMicros = nowMicros;

        imu.update(dt);

        // 2. Procesare comenzi Serial non-blocking
        if (Serial.available()) {
            char cmd = Serial.read();

            if (cmd == 'e' || cmd == 'E') {
                currentThrottle += 1;
                if (currentThrottle > PWM_IMU_TEST_MAX) currentThrottle = PWM_IMU_TEST_MAX;
                setAllMotors(currentThrottle);
                Serial.print(">>> [PWM +1us] -> "); Serial.print(currentThrottle); Serial.println("us");
            } else if (cmd == 'd' || cmd == 'D') {
                currentThrottle -= 1;
                if (currentThrottle < PWM_MIN) currentThrottle = PWM_MIN;
                setAllMotors(currentThrottle);
                Serial.print(">>> [PWM -1us] -> "); Serial.print(currentThrottle); Serial.println("us");
            } else if (cmd == 'w' || cmd == 'W' || cmd == '+') {
                currentThrottle += 25;
                if (currentThrottle > PWM_IMU_TEST_MAX) currentThrottle = PWM_IMU_TEST_MAX;
                setAllMotors(currentThrottle);
                Serial.print(">>> [PWM +25us] -> "); Serial.print(currentThrottle); Serial.println("us");
            } else if (cmd == 's' || cmd == 'S' || cmd == '-') {
                currentThrottle -= 25;
                if (currentThrottle < PWM_MIN) currentThrottle = PWM_MIN;
                setAllMotors(currentThrottle);
                Serial.print(">>> [PWM -25us] -> "); Serial.print(currentThrottle); Serial.println("us");
            } else if (cmd == 'x' || cmd == 'X' || cmd == ' ') {
                currentThrottle = PWM_MIN;
                setAllMotors(PWM_MIN);
                Serial.println("\n>>> [STOP URGENT] Motoare oprite (1000us)!\n");
            } else if (cmd == 'c' || cmd == 'C') {
                if (currentThrottle > PWM_MIN) {
                    Serial.println("\n[AVERTISMENT] Opreste mai intai motoarele (la 1000us) pentru a calibra!");
                } else {
                    Serial.println("\n[IMU] Calibrare la orizontala in curs...");
                    imu.calibrate();
                    lastTimeMicros = micros();
                }
            } else if (cmd == 'r' || cmd == 'R') {
                float newR = imu.getKalmanR() + 0.001f;
                imu.setKalmanR(newR);
                Serial.print("\n[KALMAN] R marit la: ");
                Serial.println(newR, 3);
            } else if (cmd == 'f' || cmd == 'F') {
                float newR = imu.getKalmanR() - 0.001f;
                if (newR < 0.001f) newR = 0.001f;
                imu.setKalmanR(newR);
                Serial.print("\n[KALMAN] R micsorat la: ");
                Serial.println(newR, 3);
            } else if (cmd == 'p' || cmd == 'P') {
                plotterMode = !plotterMode;
                if (plotterMode) {
                    Serial.println("\n>>> Activat MOD SERIAL PLOTTER (Format: Roll, Pitch, Zero, Throttle)");
                } else {
                    Serial.println("\n>>> Activat MOD GRAFIC BARA ASCII (Pentru Serial Monitor)");
                }
            } else if (cmd == 'm' || cmd == 'M') {
                currentThrottle = PWM_MIN;
                setAllMotors(PWM_MIN);
                Serial.println("\n>>> Oprit Modul 3. Revenire la MENIUL PRINCIPAL...");
                break;
            }
        }

        // 3. Afisare la fiecare 120 ms
        if (millis() - lastPrintMillis >= 120) {
            lastPrintMillis = millis();

            if (plotterMode) {
                // Format compatibil cu Arduino IDE / VSCode Serial Plotter:
                // Traseaza curbele Roll, Pitch, linia de 0 (Zero) si nivelul de turatie scalat
                Serial.print("Roll:");
                Serial.print(imu.getRoll(), 2);
                Serial.print(",Pitch:");
                Serial.print(imu.getPitch(), 2);
                Serial.print(",Zero:0.00");
                Serial.print(",PWM_Scaled:");
                Serial.println((currentThrottle - 1000) / 25.0f); // 0 la 1000us, 20 la 1500us
            } else {
                // Formatul grafic vizual cu axa 0 in mijloc
                Serial.print("[PWM: ");
                Serial.print(currentThrottle);
                Serial.print("us] | Gyro(X,Y): (");
                Serial.print(imu.getGyroX(), 1);
                Serial.print(", ");
                Serial.print(imu.getGyroY(), 1);
                Serial.print(") dps | Kalman R: ");
                Serial.println(imu.getKalmanR(), 3);

                printBarGraph("ROLL  ", imu.getRoll(), 20.0f);
                printBarGraph("PITCH ", imu.getPitch(), 20.0f);
                Serial.println(); // Linie goala pentru claritate vizuala
            }

            digitalWrite(LED_PIN, (currentThrottle > PWM_MIN) ? LED_ON : ((millis() / 300) % 2 == 0 ? LED_ON : LED_OFF));
        }

        delay(4); // Permite o bucla de aproximativ 250 Hz
    }

    setAllMotors(PWM_MIN);
    digitalWrite(LED_PIN, LED_OFF);
    flushSerial();
    currentMode = MODE_SELECT_MENU;
    printMainMenu();
}

// ============================================================================
// TESTEAZA MOTOARE INDIVIDUALE SAU TOATE
// ============================================================================
void runMotorTest(int index) {
    Serial.print("\n>>> Pornesc ");
    Serial.print(MOTOR_NAMES[index]);
    Serial.print(" la ");
    Serial.print(PWM_TEST);
    Serial.println("us timp de 2 secunde...");

    digitalWrite(LED_PIN, LED_ON);
    setSingleMotor(index, PWM_TEST);
    delay(2000);
    setAllMotors(PWM_MIN);
    digitalWrite(LED_PIN, LED_OFF);

    Serial.println("--> Motor oprit (1000us).");
    printTestMenu();
}

void runAllMotorsTest() {
    Serial.print("\n>>> Pornesc TOATE cele 4 motoare la ");
    Serial.print(PWM_TEST);
    Serial.println("us timp de 2 secunde...");

    digitalWrite(LED_PIN, LED_ON);
    setAllMotors(PWM_TEST);
    delay(2000);
    setAllMotors(PWM_MIN);
    digitalWrite(LED_PIN, LED_OFF);

    Serial.println("--> Toate motoarele oprite (1000us).");
    printTestMenu();
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    Serial.begin(115200);

    // Initializare pini SPI1 pentru IMU (BlackPill F411CE)
    SPI.setSCLK(PA5);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);

    // Initializare pini motoare in stare de siguranta (1000us)
    for (int i = 0; i < 4; i++) {
        escMotors[i].attach(MOTOR_PINS[i], PWM_MIN, PWM_MAX);
    }
    setAllMotors(PWM_MIN);

    // Asteptam deschiderea Serial Monitorului si afisam meniul
    unsigned long lastPrompt = 0;
    while (!Serial.available()) {
        if (millis() - lastPrompt >= 3000) {
            printMainMenu();
            lastPrompt = millis();
        }
        digitalWrite(LED_PIN, (millis() / 300) % 2 == 0 ? LED_ON : LED_OFF);
        delay(50);
    }
}

void loop() {
    // Siguranta: motoarele raman oprite cand nu este cerut un test
    setAllMotors(PWM_MIN);

    if (Serial.available()) {
        char c = Serial.read();
        flushSerial();

        // Ignoram caractere goale
        if (c == '\r' || c == '\n' || c == ' ') return;

        // GESTIUNE MENIU PRINCIPAL
        if (currentMode == MODE_SELECT_MENU) {
            if (c == '1') {
                Serial.println("\n>>> AI SELECTAT: [1] MODUL DE TEST MOTOARE");
                Serial.println("Asigura-te ca bateria este conectata (ESC-urile armate la 1000us).");
                currentMode = MODE_MOTOR_TEST;
                printTestMenu();
            } else if (c == '2') {
                executeCalibration();
            } else if (c == '3') {
                currentMode = MODE_IMU_TEST;
                runIMUTestLoop();
            } else {
                Serial.print("\nOptiune invalida: '");
                Serial.print(c);
                Serial.println("'. Alege '1' (Motoare), '2' (Calibrare) sau '3' (Test IMU + Motoare).");
                printMainMenu();
            }
        }
        // GESTIUNE MOD TEST MOTOARE
        else if (currentMode == MODE_MOTOR_TEST) {
            switch (c) {
                case '1':
                    runMotorTest(0); // PA0
                    break;
                case '2':
                    runMotorTest(1); // PA1
                    break;
                case '3':
                    runMotorTest(2); // PA2
                    break;
                case '4':
                    runMotorTest(3); // PA3
                    break;
                case 'a':
                case 'A':
                    runAllMotorsTest();
                    break;
                case 's':
                case 'S':
                    setAllMotors(PWM_MIN);
                    Serial.println("\n[STOP] Comanda urgenta: toate motoarele la 1000us.");
                    printTestMenu();
                    break;
                case 'm':
                case 'M':
                    Serial.println("\n>>> Revenire la MENIUL PRINCIPAL...");
                    currentMode = MODE_SELECT_MENU;
                    printMainMenu();
                    break;
                default:
                    Serial.print("\nComanda invalida in modul de test: '");
                    Serial.print(c);
                    Serial.println("'. Tasteaza 1, 2, 3, 4, a, s sau m.");
                    printTestMenu();
                    break;
            }
        }
    }

    delay(20);
}
