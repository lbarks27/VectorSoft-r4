#include <TeensyThreads.h>
#include <Arduino.h>
#include "utils/timing.h"
#include "utils/params/params.h"

// Thread stubs
void setAngle() {
    if (enable_ground_test) {
        Serial.println("[TVC] BLOCKED: Ground test mode.");
        return;
    }
    // real output code
}
void guidThread() { while (true) { Serial.println("[GUID]"); threads.delay(50); } }
void ctrlThread() { while (true) { Serial.println("[CTRL]"); threads.delay(10); } }

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000) ; // Teensy USB wait
    threads.addThread(setAngle);
    threads.addThread(guidThread);
    threads.addThread(ctrlThread);
}
void loop() {
    delay(1000); // Optionally blink LED or print heartbeat
}