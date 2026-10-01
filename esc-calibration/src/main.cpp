#include <Arduino.h>
#include <Servo.h>

// ============================================================================
// CONFIGURARE PINI SI CONSTANTE
// ============================================================================
// M1 -> PA0 (Față-Dreapta)
// M2 -> PA1 (Spate-Dreapta)
// M3 -> PA2 (Spate-Stânga)
// M4 -> PA3 (Față-Stânga)
const uint8_t MOTOR_PINS[4] = {PA0, PA1, PA2, PA3};
const char* MOTOR_NAMES[4] = {
    "Motor 1 (PA0 - Fata Dreapta)",
    "Motor 2 (PA1 - Spate Dreapta)",
    "Motor 3 (PA2 - Spate Stanga)",
    "Motor 4 (PA3 - Fata Stanga)"
};

// LED Onboard BlackPill F411CE (PC13 activ pe LOW)
#define LED_PIN PC13
#define LED_ON  LOW
#define LED_OFF HIGH

#define PWM_MAX 2000
#define PWM_MIN 1000

// Turatie sigura pentru test (depaseste zona de deadband a ESC-ului fara riscuri)
#define PWM_TEST 1120 

Servo escMotors[4];

void setAllMotors(int pulse_us) {
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

void printMenu() {
    Serial.println("\n--------------------------------------------------------");
    Serial.println(">>> MENIU TESTARE MOTOARE (Trimite cifra in Serial):");
    Serial.println("  [1] -> Invartire Motor 1 (PA0 - Fata Dreapta)");
    Serial.println("  [2] -> Invartire Motor 2 (PA1 - Spate Dreapta)");
    Serial.println("  [3] -> Invartire Motor 3 (PA2 - Spate Stanga)");
    Serial.println("  [4] -> Invartire Motor 4 (PA3 - Fata Stanga)");
    Serial.println("  [a] -> Invartire TOATE cele 4 motoare simultan");
    Serial.println("  [s] -> STOP de urgenta (toate la 1000us)");
    Serial.println("--------------------------------------------------------");
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    Serial.begin(115200);

    // Initializam pinii servo
    for (int i = 0; i < 4; i++) {
        escMotors[i].attach(MOTOR_PINS[i], PWM_MIN, PWM_MAX);
    }

    // PASUL 0: Asteptam conectarea utilizatorului la Serial Monitor
    while (!Serial.available()) {
        Serial.println("\n========================================================");
        Serial.println("         PILL-PILOT: CALIBRARE ESC (INTERACTIV)         ");
        Serial.println("========================================================");
        Serial.println("[ATENTIE CRITICA] Asigura-te ca ELICELE SUNT DEMONTATE!");
        Serial.println("[STARE] Bateria trebuie sa fie DECONECTATA in acest moment.");
        Serial.println("--------------------------------------------------------");
        Serial.println(">>> Trimite ENTER (sau orice tasta) pentru a incepe!");

        // Clipire LED de asteptare
        for (int i = 0; i < 20 && !Serial.available(); i++) {
            digitalWrite(LED_PIN, (i % 2 == 0) ? LED_ON : LED_OFF);
            delay(100);
        }
    }
    flushSerial();

    // PASUL 1: Activam semnalul MAXIM (2000us)
    setAllMotors(PWM_MAX);
    digitalWrite(LED_PIN, LED_ON);

    Serial.println("\n========================================================");
    Serial.println("[PAS 1] Semnal MAXIM (2000us) ACTIVAT pe PA0, PA1, PA2, PA3!");
    Serial.println("========================================================");
    Serial.println("  1. CONECTEAZA BATERIA LA DRONA ACUM.");
    Serial.println("  2. Asculta ESC-urile: vor canta melodia de pornire,");
    Serial.println("     apoi vor emite 2 BIPURI SCURTE (confirmare MAX).");
    Serial.println("--------------------------------------------------------");
    Serial.println(">>> DUPA CE AI CONECTAT BATERIA SI AI AUZIT CELE 2 BIPURI,");
    Serial.println(">>> trimite ENTER (sau orice caracter) in Serial...");

    // Asteptam ca utilizatorul sa confirme ca a auzit bipurile de MAX
    waitForSerialInput();

    // PASUL 2: Comutam la semnalul MINIM (1000us)
    Serial.println("\n========================================================");
    Serial.println("[PAS 2] Confirmare primita! Trimit semnal MINIM (1000us)...");
    Serial.println("========================================================");
    setAllMotors(PWM_MIN);

    Serial.println("Asculta ESC-urile: vor canta sunetul de armare/confirmare.");
    Serial.println("(Acel 'sunet de ON' pe care il auzi este chiar confirmarea");
    Serial.println("ca ESC-ul a acceptat minimul si este acum ARMAT si gata!)");
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

    // PASUL 3: Finalizare si afisare meniu teste motoare
    Serial.println("\n========================================================");
    Serial.println("         CALIBRARE FINALIZATA CU SUCCES!                ");
    Serial.println("========================================================");
    Serial.println("Toate ESC-urile sunt calibrate si armate la 1000us.");

    printMenu();
}

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
    printMenu();
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
    printMenu();
}

void loop() {
    // Siguranta: motoarele raman oprite implicit
    setAllMotors(PWM_MIN);

    // Citim comenzile din Serial
    if (Serial.available()) {
        char c = Serial.read();
        flushSerial();

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
            case 't':
            case 'T':
                runAllMotorsTest();
                break;
            case 's':
            case 'S':
                setAllMotors(PWM_MIN);
                Serial.println("\n[STOP] Toate motoarele au fost fortate la 1000us.");
                printMenu();
                break;
            case '\r':
            case '\n':
                // Ignoram tastele goale de newline
                break;
            default:
                Serial.print("\nComanda necunoscuta: '");
                Serial.print(c);
                Serial.println("'. Trimite 1, 2, 3, 4, a sau s.");
                printMenu();
                break;
        }
    }

    delay(20);
}
