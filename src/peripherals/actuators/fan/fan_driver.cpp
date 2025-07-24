#include "peripherals/actuators/fan/fan_driver.h"
#include <Arduino.h>

static bool escArmed[40] = {false}; // Track armed state by pin number

void ESC_init(uint8_t escPin) {
    pinMode(escPin, OUTPUT);
    analogWriteFrequency(escPin, 400); // 400 Hz is typical for PWM ESCs
    analogWrite(escPin, 0);
}

void ESC_arm(uint8_t escPin) {
    escArmed[escPin] = true;
    ESC_setThrottle(escPin, 1000); // Minimum throttle
    delay(100);
}

void ESC_disarm(uint8_t escPin) {
    escArmed[escPin] = false;
    analogWrite(escPin, 0); // Stop PWM signal
}

void ESC_setThrottle(uint8_t escPin, uint16_t microseconds) {
    if (!escArmed[escPin]) return;

    uint8_t duty = map(constrain(microseconds, 1000, 2000), 1000, 2000, 26, 52); // ~5–10% duty
    analogWrite(escPin, duty);
}
