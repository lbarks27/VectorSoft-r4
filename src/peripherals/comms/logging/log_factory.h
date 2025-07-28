#pragma once
#include "log_sd_real.h"
#include "log_stub.h"
#include "utils/params/params.h" // For TEST_MODE, SD_CS_PIN, etc.

inline ILogChannel* createLogger(bool testMode) {
    if (testMode) return new LogStub();
    else return new LogSDReal(SD_CS_PIN, "/logs", "log_");
}