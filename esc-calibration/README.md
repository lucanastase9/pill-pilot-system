# Ghid Calibrare Interactivă ESC - Pill-Pilot System

Proiect dedicat pentru calibrarea controllerelor de turație (ESC) pe **STM32 BlackPill F411CE** cu ghidare interactivă și testare individuală a motoarelor prin **Serial Monitor** (115200 baud).

## ⚠️ AVERTISMENT CRITIC DE SIGURANȚĂ
> **DEMONTEAZĂ TOATE CELE 4 ELICE ÎNAINTE DE A CONECTA BATERIA!**  
> Nu monta elicele niciodată în timpul procedurilor de calibrare sau testare!

---

## 1. Conexiuni Motoare (Quad-X)
Conform `stm32-uav-core`:
- **Motor 1** -> **`PA0`** (Față - Dreapta)
- **Motor 2** -> **`PA1`** (Spate - Dreapta)
- **Motor 3** -> **`PA2`** (Spate - Stânga)
- **Motor 4** -> **`PA3`** (Față - Stânga)

---

## 2. Ce înseamnă sunetele ESC-ului?
1. La conectarea bateriei (când semnalul este la **2000 µs**):
   - Melodia inițială.
   - **2 bipuri scurte** (`bip-bip`) = ESC-ul a înregistrat limita maximă (2000 µs).
2. Când apeși Enter și semnalul coboară la **1000 µs**:
   - ESC-ul emite **sunetul clasic de pornire/armare** (melodie scurtă + bip lung) = ESC-ul a salvat limita minimă, s-a ARMAT și este gata de rotire! În acest moment motoarele stau nemișcate la 1000 µs.

---

## 3. Comenzi Testare în Serial Monitor (după calibrare)
După ce calibrarea s-a finalizat, poți trimite direct în consolă:
- **`1` + Enter**: Rotește doar **Motorul 1** (`PA0` - Față-Dreapta) timp de 2 secunde.
- **`2` + Enter**: Rotește doar **Motorul 2** (`PA1` - Spate-Dreapta) timp de 2 secunde.
- **`3` + Enter**: Rotește doar **Motorul 3** (`PA2` - Spate-Stânga) timp de 2 secunde.
- **`4` + Enter**: Rotește doar **Motorul 4** (`PA3` - Față-Stânga) timp de 2 secunde.
- **`a` + Enter**: Rotește **toate cele 4 motoare** simultan timp de 2 secunde.
- **`s` + Enter**: Comandă STOP de urgență (toate la 1000 µs).

## 4.Diagramă de Orientare a Dronei
                  FAȚĂ (FRONT)
                       ▲
                       │
       (M4) ───────────┴─────────── (M1)
    [Față-Stânga]               [Față-Dreapta]
       Pin: PA3                    Pin: PA0
               \               /
                \             /
                 \   STM32   /
                  \ F411CE  /
                 /           \
                /             \
               /               \
       Pin: PA2                    Pin: PA1
    [Spate-Stânga]              [Spate-Dreapta]
       (M3) ─────────────────────── (M2)
                       │
                  SPATE (REAR)