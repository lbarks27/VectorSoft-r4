#include "utils/params/config_loader.h"
#include "utils/params/params.h"
#include <Arduino.h>

bool reloadParamsFromSD(const char* filename) {
    // STUB: Pretend to load from SD
    Serial.print("[CONFIG LOADER] Would load params from SD file: ");
    Serial.println(filename);
    // Later: parse SD, update extern params
    return true;
}

bool reloadParamsFromTelemetry(const char* configLine) {
    // STUB: Pretend to load from telemetry
    Serial.print("[CONFIG LOADER] Would process telemetry config: ");
    Serial.println(configLine);
    // Later: parse string, update extern params
    return true;
}