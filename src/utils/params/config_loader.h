#pragma once

// Load from SD card file (returns true on success)
bool reloadParamsFromSD(const char* filename = "/config.txt");

// Parse and load from telemetry command string (e.g. "TVC_MAX_ANGLE=32.5")
bool reloadParamsFromTelemetry(const char* configLine);