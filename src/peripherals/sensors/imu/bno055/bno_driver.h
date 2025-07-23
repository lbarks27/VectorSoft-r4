#pragma once
#include <Adafruit_BNO055.h>
#include <Wire.h>

class BNO055Wrapper {
public:
    BNO055Wrapper(uint8_t addr = BNO055_ADDRESS_A, int sensorID = -1)
        : bno(addr, sensorID) {}

    bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF) {
        return bno.begin(mode);
    }
    void setMode(adafruit_bno055_opmode_t mode) {
        bno.setMode(mode);
        delay(20); // Mode switch needs brief delay
    }

    imu::Vector<3> getAccel()   { return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); }
    imu::Vector<3> getGyro()    { return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); }
    imu::Vector<3> getMag()     { return bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER); }
    imu::Vector<3> getLinAccel(){ return bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); }
    imu::Vector<3> getGravity() { return bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY); }
    imu::Quaternion getQuat()   { return bno.getQuat(); }
    float getTemp()             { return bno.getTemp(); }
    uint8_t getCalibrationStatus() { 
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        // Return combined calibration status as a single byte (e.g., pack into bits)
        return (sys << 6) | (gyro << 4) | (accel << 2) | mag;
    }
    bool isFullyCalibrated()    { 
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        return (sys == 3 && gyro == 3 && accel == 3 && mag == 3);
    }

private:
    Adafruit_BNO055 bno;
};