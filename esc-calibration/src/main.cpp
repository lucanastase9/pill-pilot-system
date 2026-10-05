#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <BMI160Gen.h>

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

// Turație sigură de test pe banc (fără elice montate)
#define PWM_TEST 1120 

Servo escMotors[4];

// Stările posibile ale programului
enum ProgramMode {
    MODE_SELECT_MENU,
    MODE_MOTOR_TEST,
    MODE_IMU_TEST
};

ProgramMode currentMode = MODE_SELECT_MENU;

// Variabile de stare pentru IMU BMI160
bool imuInitialized = false;
float accelOffsetX = 0.0f;
float accelOffsetY = 0.0f;
float accelOffsetZ = 0.0f;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

const float EXPECTED_1G = 16384.0f; // 1G la sensibilitate +/- 2G

// ============================================================================
// FUNCTII HELPER PENTRU MOTOARE SI SERIAL
// ============================================================================
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
    Serial.println("  [3] -> MODUL DE TEST ALINIERE IMU (135 GRADE)");
    Serial.println("         (afisare in timp real Roll/Pitch cu transformarea aplicata)");
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
// PROCEDURA TEST IMU CU TRANSFORMARE 135 GRADE (OPTIONALUL 3)
// ============================================================================
bool initAndCalibrateIMU() {
    if (!imuInitialized) {
        Serial.println("\n[IMU] Conectare la BMI160 pe pinul PA4 (SPI)...");
        pinMode(IMU_CS_PIN, OUTPUT);
        digitalWrite(IMU_CS_PIN, HIGH);
        delay(10);
        
        // Configurare explicita a pinilor SPI1 (pentru STM32F411 BlackPill)
        SPI.setSCLK(PA5);
        SPI.setMISO(PA6);
        SPI.setMOSI(PA7);

        // RAW SPI TEST to see what's failing
        SPI.begin();
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        digitalWrite(IMU_CS_PIN, LOW);
        delay(1);
        SPI.transfer(0x80); // Read register 0x00 (CHIP_ID)
        uint8_t raw_id = SPI.transfer(0x00);
        digitalWrite(IMU_CS_PIN, HIGH);
        SPI.endTransaction();

        Serial.print("[DEBUG] RAW CHIP ID: 0x");
        Serial.println(raw_id, HEX);
        delay(2000); // Pauza ca utilizatorul sa vada mesajul

        if (!BMI160.begin(BMI160GenClass::SPI_MODE, IMU_CS_PIN)) {
            Serial.print("[EROARE] Nu am putut initializa BMI160 pe SPI! CHIP ID raportat de librarie: 0x");
            Serial.println(BMI160.getDeviceID(), HEX);
            Serial.println("  -> Verifica daca senzorul este alimentat (LED-ul modulului aprins/tensiune VCC).");
            Serial.println("  -> Daca senzorul este alimentat din bateria dronei (BEC 5V), conecteaza bateria!");
            Serial.println("  -> Verifica cablajul SPI: CS=PA4, SCK=PA5, MISO=PA6, MOSI=PA7.");
            delay(4000); // Pauza lunga sa ramana mesajul pe ecran
            return false;
        }
        BMI160.setGyroDLPFMode(BMI160_DLPF_MODE_NORM);
        BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_NORM);
        imuInitialized = true;
        Serial.println("[OK] BMI160 initializat cu succes!");
        delay(1000);
    }

    Serial.println("[IMU] Aseaza drona pe o suprafata orizontala si NU o misca!");
    Serial.println("[IMU] Calibrez offset-urile de zero (aprox 2 secunde)...");
    delay(1000);

    long sumAX = 0, sumAY = 0, sumAZ = 0;
    long sumGX = 0, sumGY = 0, sumGZ = 0;
    const int samples = 500;

    for (int i = 0; i < samples; i++) {
        int gx, gy, gz, ax, ay, az;
        BMI160.readGyro(gx, gy, gz);
        BMI160.readAccelerometer(ax, ay, az);

        sumGX += gx; sumGY += gy; sumGZ += gz;
        sumAX += ax; sumAY += ay; sumAZ += az;
        delay(3);
    }

    gyroOffsetX = (float)sumGX / samples;
    gyroOffsetY = (float)sumGY / samples;
    gyroOffsetZ = (float)sumGZ / samples;

    accelOffsetX = (float)sumAX / samples;
    accelOffsetY = (float)sumAY / samples;
    accelOffsetZ = ((float)sumAZ / samples) - EXPECTED_1G;

    Serial.println("[IMU] Calibrare offset reusita!");
    return true;
}

void runIMUTestLoop() {
    setAllMotors(PWM_MIN); // Siguranta: motoare oprite

    if (!initAndCalibrateIMU()) {
        currentMode = MODE_SELECT_MENU;
        printMainMenu();
        return;
    }

    Serial.println("\n========================================================");
    Serial.println("      MODUL 3: MONITORIZARE IN TIMP REAL IMU (135°)     ");
    Serial.println("========================================================");
    Serial.println("Cum verifici alinierea:");
    Serial.println("  1. Ridica BOTUL dronei in sus     --> PITCH creste cu PLUS (+)");
    Serial.println("  2. Inclina drona spre DREAPTA     --> ROLL creste cu PLUS (+)");
    Serial.println("  * Observa: cand misti doar botul, ROLL-ul ar trebui sa ramana ~0!");
    Serial.println("--------------------------------------------------------");
    Serial.println(">>> Trimite 'm' (sau orice tasta) pentru revenire la MENIU.");
    Serial.println("========================================================\n");

    float rollEstimate = 0.0f;
    float pitchEstimate = 0.0f;
    unsigned long lastTimeMicros = micros();
    unsigned long lastPrintMillis = 0;

    const float C = 0.70710678f; // sqrt(2)/2 pentru rotirea la 135 grade

    while (!Serial.available()) {
        unsigned long nowMicros = micros();
        float dt = (nowMicros - lastTimeMicros) / 1000000.0f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
        lastTimeMicros = nowMicros;

        int raw_gx, raw_gy, raw_gz;
        int raw_ax, raw_ay, raw_az;

        BMI160.readGyro(raw_gx, raw_gy, raw_gz);
        BMI160.readAccelerometer(raw_ax, raw_ay, raw_az);

        // 1. Scadem offset-urile de calibrare
        float gx_f = (float)raw_gx - gyroOffsetX;
        float gy_f = (float)raw_gy - gyroOffsetY;
        float gz_f = (float)raw_gz - gyroOffsetZ;

        float ax_f = (float)raw_ax - accelOffsetX;
        float ay_f = (float)raw_ay - accelOffsetY;
        float az_f = (float)raw_az - accelOffsetZ;

        // 2. APLICAM TRANSFORMAREA DE 135 GRADE CATRE COORDONATELE DRONEI
        // X senzor spre M3, Y senzor spre M2, Z in sus
        float ax_drone = -C * (ax_f + ay_f);
        float ay_drone =  C * (ay_f - ax_f);
        float az_drone =  az_f;

        float gx_drone = -C * (gx_f + gy_f);
        float gy_drone =  C * (gy_f - gx_f);
        float gz_drone =  gz_f;

        // 3. Calculam vitezele unghiulare in grade/secunda (131 LSB/(deg/s))
        float gyroRateX = gx_drone / 131.0f;
        float gyroRateY = gy_drone / 131.0f;
        float gyroRateZ = gz_drone / 131.0f;

        // 4. Calculam unghiurile din accelerometru
        float accRoll  = atan2(ay_drone, az_drone) * 57.2957795f;
        float accPitch = atan2(-ax_drone, sqrt(ay_drone * ay_drone + az_drone * az_drone)) * 57.2957795f;

        // 5. Filtru complementar
        rollEstimate  = 0.98f * (rollEstimate  + gyroRateX * dt) + 0.02f * accRoll;
        pitchEstimate = 0.98f * (pitchEstimate + gyroRateY * dt) + 0.02f * accPitch;

        // 6. Afisare la fiecare 150 ms
        if (millis() - lastPrintMillis >= 150) {
            lastPrintMillis = millis();

            Serial.print("[DRONA 135°] Roll: ");
            if (rollEstimate >= 0.0f) Serial.print("+");
            Serial.print(rollEstimate, 1);
            Serial.print("°  |  Pitch: ");
            if (pitchEstimate >= 0.0f) Serial.print("+");
            Serial.print(pitchEstimate, 1);
            Serial.print("°  |  Gyro(X,Y): (");
            Serial.print(gyroRateX, 1);
            Serial.print(", ");
            Serial.print(gyroRateY, 1);
            Serial.println(") dps");

            digitalWrite(LED_PIN, (millis() / 250) % 2 == 0 ? LED_ON : LED_OFF);
        }

        delay(10);
    }

    flushSerial();
    Serial.println("\n>>> Oprit Modul 3. Revenire la MENIUL PRINCIPAL...");
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

    // Initializam pinul CS pentru IMU pe HIGH
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH);

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
                Serial.println("'. Alege '1' (Motoare), '2' (Calibrare) sau '3' (Test IMU 135°).");
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
