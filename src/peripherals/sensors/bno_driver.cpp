#include "bno_driver.h"
// #include <Adafruit_BNO055.h> // Uncomment if using a library

bool IMU_BNO055::begin() {
    // Initialize sensor hardware (stub or use Adafruit_BNO055 object)
    // For now, stub:
    initialized = true;
    Serial.println("[IMU] BNO055 init STUB");
    return initialized;
}

bool IMU_BNO055::readAccel(float& ax, float& ay, float& az) {
    // Replace this with real sensor read
    ax = 0; ay = 0; az = 0;
    Serial.println("[IMU] ReadAccel STUB");
    return initialized;
}

bool IMU_BNO055::readGyro(float& gx, float& gy, float& gz) {
    gx = 0; gy = 0; gz = 0;
    Serial.println("[IMU] ReadGyro STUB");
    return initialized;
}

bool IMU_BNO055::readOrientation(float& q0, float& q1, float& q2, float& q3) {
    q0 = 1; q1 = 0; q2 = 0; q3 = 0;
    Serial.println("[IMU] ReadOrientation STUB");
    return initialized;
}

bool IMU_BNO055::isHealthy() const {
    return initialized;
}