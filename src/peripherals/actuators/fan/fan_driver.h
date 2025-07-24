#pragma once
#include <Arduino.h>

void ESC_init(uint8_t escPin);
void ESC_arm(uint8_t escPin);
void ESC_disarm(uint8_t escPin);
void ESC_setThrottle(uint8_t escPin, uint16_t microseconds);