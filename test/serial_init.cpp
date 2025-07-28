#include <Arduino.h>

// Define your serial ports
#define RFD_SERIAL   Serial1   // Teensy pins: RX1=D0, TX1=D1
#define CUBE_SERIAL  Serial2   // Teensy pins: RX2=D7, TX2=D8

void setup() {
  // USB for debug output
  Serial.begin(115200);
  while (!Serial) { /* wait for host USB connection */ }

  // Initialize RFD-900x link
  // Typical RFD default is 57600 or 115200 baud—match your module config
  RFD_SERIAL.begin(57600);
  Serial.println("RFD link on Serial1 at 57600");

  // Initialize Cube Orange MAVLink link
  // ArduPilot default telemetry is often 115200 baud
  CUBE_SERIAL.begin(115200);
  Serial.println("Cube nav link on Serial2 at 115200");

  // (Optional) Give both ports a moment to settle
  delay(50);
}

void loop() {
  // Example: echo any incoming to USB for debug
  if (RFD_SERIAL.available()) {
    Serial.write(RFD_SERIAL.read());
  }
  if (CUBE_SERIAL.available()) {
    Serial.write(CUBE_SERIAL.read());
  }
}