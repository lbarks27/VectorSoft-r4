#pragma once
#include <Arduino.h>

class IMU_BNO055 {
public:
    bool begin(); // Returns true if sensor detected and initialized
    bool readAccel(float& ax, float& ay, float& az);
    bool readGyro(float& gx, float& gy, float& gz);
    bool readOrientation(float& q0, float& q1, float& q2, float& q3);
    bool isHealthy() const;
private:
    bool initialized = false;
    // I2C address, internal error flags, etc.
};