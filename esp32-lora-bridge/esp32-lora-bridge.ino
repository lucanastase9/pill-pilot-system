#include <RadioLib.h>
#include <SPI.h>

// ==========================================
// CONFIGURARE PINI ESP32 -> SX1262
// ==========================================
#define NSS_PIN   5
#define DIO1_PIN  4   // S-A MODIFICAT DE LA 2 LA 4 (Pinul 2 e periculos pe ESP32)
#define NRST_PIN  14
#define BUSY_PIN  32

#define LED_PIN   2   // LED-ul albastru built-in pe ESP32

// Instanțierea modulului SX1262
Module* mod = new Module(NSS_PIN, DIO1_PIN, NRST_PIN, BUSY_PIN);
SX1262 radio(mod);

// ==========================================
// VARIABILE GLOBALE
// ==========================================
volatile bool loraReceivedFlag = false;

uint8_t serialBuf[256];
size_t serialLen = 0;
unsigned long lastSerialTime = 0;

// Funcția chemată automat (Interrupt) când modulul LoRa primește un pachet
void IRAM_ATTR setFlag() {
    loraReceivedFlag = true;
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Comunicarea cu Ground Station (Laptop/C++)
    Serial.begin(115200);
    while (!Serial); // Așteaptă conectarea portului serial

    // Forțăm SPI-ul să folosească exact pinii hardware (SCK, MISO, MOSI).
    // ATENȚIE: Ultimul pin trebuie să fie -1 (NU NSS_PIN).
    // Dacă dăm pinul de CS către driverul hardware SPI, librăria RadioLib nu-l mai poate controla manual!
    SPI.begin(18, 19, 23, -1);

    // Inițializare modul LoRa cu ACELEAȘI SETĂRI CA PE DRONĂ
    // begin(freq, bw, sf, cr, syncWord, power)
    // Frecvență: 868.0 MHz, BW: 500.0 kHz, SF: 7, CR: 5 (adică 4/5), Sync: 0x12, Pwr: +22 dBm
    int state = radio.begin(868.0, 500.0, 7, 5, 0x12, 22);

    if (state == RADIOLIB_ERR_NONE) {
        // Afișăm un mesaj clar pentru a ști că a mers!
        Serial.println("\n[OK] ESP32 LoRa Bridge PORNIT! Astept MAVLink...");
        
        // Setăm pinul DIO1 să declanșeze o întrerupere la recepție
        radio.setDio1Action(setFlag);
        
        // Punem modulul în modul de ascultare
        radio.startReceive();
    } else {
        // Eroare la inițializare - o vom trimite pe Serial ca text pentru debugging (GCS o va ignora dacă nu e MAVLink)
        Serial.print("Eroare initializare LoRa, cod: ");
        Serial.println(state);
        while (true); // Blocare sistem în caz de eroare
    }
}

void loop() {
    // ========================================================
    // 1. RECEPȚIE DE LA GROUND STATION (SERIAL) -> LORA
    // ========================================================
    // Citim tot ce vine de la aplicația C++ prin USB
    while (Serial.available()) {
        if (serialLen < sizeof(serialBuf)) {
            serialBuf[serialLen++] = Serial.read();
            lastSerialTime = millis();
        } else {
            // Buffer plin, aruncăm surplusul (un pachet de control are doar ~21 bytes)
            Serial.read(); 
        }
    }

    // Dacă am primit date și a trecut o scurtă pauză (10 ms) de la ultimul byte,
    // înseamnă că s-a terminat de transmis pachetul MAVLink curent
    if (serialLen > 0 && (millis() - lastSerialTime > 10)) {
        digitalWrite(LED_PIN, HIGH); // Aprindem LED-ul pe durata emisiei
        
        // Oprim ascultarea pe DIO1 ca să nu o confundăm cu sfârșitul transmisiei
        radio.clearDio1Action();
        
        // Transmitem pachetul spre Dronă
        radio.transmit(serialBuf, serialLen);
        
        // Resetăm buffer-ul serial
        serialLen = 0;
        
        // Revenim imediat la modul de recepție
        radio.setDio1Action(setFlag);
        radio.startReceive();
        
        digitalWrite(LED_PIN, LOW); // Stingem LED-ul
    }

    // ========================================================
    // 2. RECEPȚIE DE LA DRONĂ (LORA) -> GROUND STATION
    // ========================================================
    if (loraReceivedFlag) {
        loraReceivedFlag = false;

        // Aflăm lungimea pachetului recepționat
        size_t len = radio.getPacketLength();
        if (len > 0 && len <= 256) {
            uint8_t loraBuf[256];
            int state = radio.readData(loraBuf, len);

            if (state == RADIOLIB_ERR_NONE) {
                // Trimitem instantaneu datele primite către aplicația C++ (GCS)
                // Serial.write trimite byți bruti, ceea ce MAVLink are nevoie
                Serial.write(loraBuf, len);
                
                // Pâlpâim scurt LED-ul pentru a indica recepția pe PC
                digitalWrite(LED_PIN, HIGH);
                delay(2);
                digitalWrite(LED_PIN, LOW);
            }
        }
        // După citire, reintrăm în modul de ascultare
        radio.startReceive();
    }
}
