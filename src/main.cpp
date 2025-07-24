#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial3.begin(57600, SERIAL_8N1_RXINV); // Inversion ON for S.Port
    Serial.println("Serial up!");
}

void loop() {
    if (Serial3.available()) {
        uint8_t b = Serial3.read();
        Serial.print("S.Port RX: ");
        Serial.println(b, HEX);
    }
    delay(5);
}