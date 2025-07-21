#include "utils/timing.h"

void navThread() {
    while (true) {
        // Navigation logic
        sleepMs(1000 / NAV_THREAD_RATE_HZ);
    }
}