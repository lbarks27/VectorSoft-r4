#include "utils/timing.h"

void ctrlThread() {
    while (true) {
        // Control logic
        sleepMs(1000 / CTRL_THREAD_RATE_HZ);
    }
}