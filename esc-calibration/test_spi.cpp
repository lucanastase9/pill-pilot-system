#include <Arduino.h>
#include <SPI.h>
void setup() {
    Serial.begin(115200);
    SPI.begin();
    pinMode(PA4, OUTPUT);
    digitalWrite(PA4, HIGH);
}
void loop() {}
