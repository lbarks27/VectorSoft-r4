#include <stdint.h>
#include <Arduino.h>

#pragma once

#define NAV_THREAD_RATE_HZ 20
#define CTRL_THREAD_RATE_HZ 100
#define GUID_THREAD_RATE_HZ 50

inline void sleepMs(uint32_t ms) { delay(ms); }
inline uint32_t millisSince(uint32_t t0) { return millis() - t0; }

// More advanced timing/profiling helpers as needed