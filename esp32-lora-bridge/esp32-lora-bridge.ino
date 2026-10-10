#include <RadioLib.h>
#include <SPI.h>

// ==========================================
// CONFIGURARE PINI ESP32 -> SX1262
// ==========================================
#define NSS_PIN   5
#define DIO1_PIN  4
#define NRST_PIN  14
#define BUSY_PIN  32

#define LED_PIN   2   // LED albastru built-in pe ESP32

Module* mod = nullptr;
SX1262* radio = nullptr;

// ==========================================
// VARIABILE GLOBALE
// ==========================================
volatile bool loraReceivedFlag = false;

uint8_t serialBuf[250]; // Buffer limitat la 250 bytes pentru siguranță SX1262 FIFO
size_t serialLen = 0;
unsigned long lastSerialTime = 0;

// Întrerupere recepție LoRa (DIO1)
void IRAM_ATTR setFlag() {
    loraReceivedFlag = true;
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Comunicarea cu Ground Station (Laptop/C++)
    Serial.begin(115200);
    delay(500);

    // Inițializare magistrală hardware SPI (VSPI: 18=SCK, 19=MISO, 23=MOSI, 5=CS)
    SPI.begin(18, 19, 23, 5);

    // Creăm instanțele după pornirea SPI
    mod = new Module(NSS_PIN, DIO1_PIN, NRST_PIN, BUSY_PIN);
    radio = new SX1262(mod);

    // Inițializare cu parametrii hardware confirmați:
    // Freq: 868.0 MHz, BW: 500.0 kHz, SF: 7, CR: 5, Sync: 0x12, Pwr: +22 dBm, Preamble: 8, TCXO: 3.3V, LDO: false
    int state = radio->begin(868.0, 500.0, 7, 5, 0x12, 22, 8, 3.3f, false);

    if (state == RADIOLIB_ERR_NONE) {
        // Activăm comutarea automată a antenei prin DIO2 (esențial pentru raza de acțiune)
        radio->setDio2AsRfSwitch(true);

        // Curățăm flag-ul și activăm modul de ascultare continuă
        loraReceivedFlag = false;
        radio->setDio1Action(setFlag);
        radio->startReceive();

        Serial.println("\n[OK] ESP32 LoRa Bridge PORNIT! Astept MAVLink...");
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
    } else {
        Serial.print("Eroare initializare LoRa, cod: ");
        Serial.println(state);
        while (true) {
            // Pâlpâire rapidă în caz de eroare
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }
}

void loop() {
    // ========================================================
    // 1. RECEPȚIE DE LA GROUND STATION (SERIAL / USB) -> LORA
    // ========================================================
    while (Serial.available()) {
        if (serialLen < sizeof(serialBuf)) {
            serialBuf[serialLen++] = Serial.read();
            lastSerialTime = millis();
        } else {
            Serial.read(); // Previne buffer overflow
        }
    }

    // Dacă a trecut o scurtă pauză (4 ms) de la ultimul byte, pachetul MAVLink s-a terminat
    if (serialLen > 0 && (millis() - lastSerialTime > 4)) {
        digitalWrite(LED_PIN, HIGH);
        
        // Oprim ascultarea pe DIO1 pe durata transmisiei
        radio->clearDio1Action();
        
        // Transmitem comanda către Dronă prin LoRa
        radio->transmit(serialBuf, serialLen);
        
        // Resetăm buffer-ul serial
        serialLen = 0;
        
        // CRITIC: Resetăm flag-ul înainte de a reveni în recepție (previne citiri fantomă)
        loraReceivedFlag = false;
        radio->setDio1Action(setFlag);
        radio->startReceive();
        
        digitalWrite(LED_PIN, LOW);
    }

    // ========================================================
    // 2. RECEPȚIE TELEMETRIE DE LA DRONĂ (LORA) -> SERIAL / USB
    // ========================================================
    if (loraReceivedFlag) {
        loraReceivedFlag = false;

        size_t len = radio->getPacketLength();
        if (len > 0 && len <= sizeof(serialBuf)) {
            uint8_t loraBuf[250];
            int state = radio->readData(loraBuf, len);

            if (state == RADIOLIB_ERR_NONE) {
                // Trimitem datele brute MAVLink către aplicația C++ (GCS)
                Serial.write(loraBuf, len);
                
                // Puls vizual scurt
                digitalWrite(LED_PIN, HIGH);
                delay(1);
                digitalWrite(LED_PIN, LOW);
            }
        }
        
        // Asigurăm reintrarea în ascultare
        radio->startReceive();
    }
}