#include "utils/timing.h"

void guidThread() {
    while (true) {
        // Guidance logic
        sleepMs(1000 / GUID_THREAD_RATE_HZ);
    }
}