#pragma once
#include "imu_interface.h"
#include <Arduino.h>

class IMUStub : public IIMU {
public:
    bool begin() override { return true; }
    bool read(ImuData& data) override {
        data.accelX = data.accelY = 0; data.accelZ = 9.8f;
        data.gyroX = data.gyroY = data.gyroZ = 0;
        data.magX = data.magY = data.magZ = 0;
        data.roll = data.pitch = data.yaw = 0;
        data.temp = 25.0f;
        data.timestamp_us = micros();
        return true;
    }
    bool isHealthy() const override { return true; }
    bool setMode(ImuOpMode) override { return true; }
};