#pragma once
#include "power_interface.h"

class PowerStub : public IPowerMonitor {
public:
    bool begin() override { return true; }
    bool read(PowerData& data) override {
        data.voltage = 12.6f;      // Simulate a 3S full battery
        data.current = 0.5f;
        data.temperature = 25.0f;
        data.timestamp_us = micros();
        return true;
    }
    bool isHealthy() const override { return true; }
    int cellCount() const override { return 3; }
    float batteryLevel() const override { return 1.0f; }
};