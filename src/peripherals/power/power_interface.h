#pragma once
#include <Arduino.h>

struct PowerData {
    float voltage;      // V
    float current;      // A (optional)
    float temperature;  // C (optional)
    uint32_t timestamp_us;
};

class IPowerMonitor {
public:
    virtual bool begin() = 0;
    virtual bool read(PowerData& data) = 0;
    virtual bool isHealthy() const = 0;
    virtual int cellCount() const = 0;           // Added
    virtual float batteryLevel() const = 0;       // Added
    virtual ~IPowerMonitor() {}
};