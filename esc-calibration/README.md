# Utilitar Calibrare, Testare Motoare & Aliniere IMU - Pill-Pilot System

Proiect dedicat pentru **STM32 BlackPill F411CE** cu interfață Serială interactivă (115200 baud).

## ⚠️ AVERTISMENT CRITIC DE SIGURANȚĂ
> **DEMONTEAZĂ TOATE CELE 4 ELICE ÎNAINTE DE TESTARE SAU CALIBRARE!**  
> Nu monta elicele niciodată în timpul testelor pe banc!

---

## 1. Conexiuni Motoare & Direcții de Rotație (Quad-X)
Conform `MotorMixer.hpp`:
- **Motor 1** -> **`PA0`** (Față - Dreapta) | **CCW ↺** (sens antiorar)
- **Motor 2** -> **`PA1`** (Spate - Dreapta) | **CW ↻** (sens orar)
- **Motor 3** -> **`PA2`** (Spate - Stânga)  | **CCW ↺** (sens antiorar)
- **Motor 4** -> **`PA3`** (Față - Stânga)   | **CW ↻** (sens orar)

---

## 2. Conexiuni Senzor IMU (BMI160 pe SPI1)
- **CS** ➔ Pin **`PA4`**
- **SCK** ➔ Pin **`PA5`**
- **MISO** ➔ Pin **`PA6`**
- **MOSI** ➔ Pin **`PA7`**
- **Aliniere fizică**: Axa $X$ orientată spre **M3** (Spate-Stânga), axa $Y$ spre **M2** (Spate-Dreapta), cipul în **SUS**.

---

## 3. Meniul Principal la Boot (Serial 115200 baud)

### Tasta `1` ➔ Modul de Test Motoare
Pentru verificarea rotației și a sensului fiecărui motor (ESC-urile alimentate normal la 1000 µs):
- **`1` + Enter**: Rotește **Motorul 1** (`PA0` - Față-Dreapta) timp de 2 secunde.
- **`2` + Enter**: Rotește **Motorul 2** (`PA1` - Spate-Dreapta) timp de 2 secunde.
- **`3` + Enter**: Rotește **Motorul 3** (`PA2` - Spate-Stânga) timp de 2 secunde.
- **`4` + Enter**: Rotește **Motorul 4** (`PA3` - Față-Stânga) timp de 2 secunde.
- **`a` + Enter**: Rotește **toate cele 4 motoare** simultan.
- **`s` + Enter**: Comandă **STOP** de urgență.
- **`m` + Enter**: Înapoi la Meniul Principal.

### Tasta `2` ➔ Modul de Calibrare ESC
Procedura pas cu pas de calibrare a plajei de turație:
1. Bateria deconectată ➔ Apasă Enter ➔ Semnal MAX (2000 µs).
2. Conectează bateria ➔ Ascultă cele 2 bipuri scurte ➔ Apasă Enter.
3. Semnal MIN (1000 µs) ➔ ESC-urile confirmă minimul și se armează.
4. Trece automat în Modul de Test Motoare.

### Tasta `3` ➔ Modul de Test Aliniere IMU (135°)
Pentru vizualizarea în timp real a unghiurilor fără a porni motoarele:
1. Drona stă pe birou orizontală ➔ Calibrează automat offset-ul de zero.
2. Afișează continuu în consolă:
   `[DRONA 135°] Roll: +0.2° | Pitch: -0.1° | Gyro(X,Y): (0.0, 0.0) dps`
3. **Validare practică**:
   - Ridică botul dronei în sus ➔ **PITCH** trebuie să crească spre valori pozitive (+), iar Roll să rămână ~0.
   - Înclină drona spre dreapta ➔ **ROLL** trebuie să crească spre valori pozitive (+), iar Pitch să rămână ~0.
4. Trimite **`m` + Enter** pentru a opri testul și a reveni la Meniul Principal.