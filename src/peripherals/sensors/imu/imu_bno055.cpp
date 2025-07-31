#include "peripherals/sensors/imu/imu_bno055.h"
#include <Wire.h>
#include <Arduino.h>

static adafruit_bno055_opmode_t convertImuOpMode(ImuOpMode mode) {
    switch (mode) {
        case ImuOpMode::CONFIG:        return OPERATION_MODE_CONFIG;
        case ImuOpMode::ACCONLY:       return OPERATION_MODE_ACCONLY;
        case ImuOpMode::MAGONLY:       return OPERATION_MODE_MAGONLY;
        case ImuOpMode::GYRONLY:       return OPERATION_MODE_GYRONLY;
        case ImuOpMode::ACCMAG:        return OPERATION_MODE_ACCMAG;
        case ImuOpMode::ACCGYRO:       return OPERATION_MODE_ACCGYRO;
        case ImuOpMode::MAGGYRO:       return OPERATION_MODE_MAGGYRO;
        case ImuOpMode::AMG:           return OPERATION_MODE_AMG;
        case ImuOpMode::IMU:           return OPERATION_MODE_IMUPLUS;
        case ImuOpMode::COMPASS:       return OPERATION_MODE_COMPASS;
        case ImuOpMode::M4G:           return OPERATION_MODE_M4G;
        case ImuOpMode::NDOF_FMC_OFF:  return OPERATION_MODE_NDOF_FMC_OFF;
        case ImuOpMode::NDOF:          return OPERATION_MODE_NDOF;
        default:                       return OPERATION_MODE_CONFIG;
    }
}

IMUBNO055::IMUBNO055(uint8_t address, TwoWire* i2cPort)
    : _bno(-1, address, i2cPort), _healthy(false) {}

bool IMUBNO055::begin() {
    _healthy = _bno.begin();
    if (!_healthy) return false;
    _bno.setExtCrystalUse(true);
    return true;
}

bool IMUBNO055::read(ImuData& data) {
    if (!_healthy) return false;

    imu::Vector<3> acc = _bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr = _bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = _bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> eul = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    data.accelX = acc.x(); data.accelY = acc.y(); data.accelZ = acc.z();
    data.gyroX = gyr.x();  data.gyroY = gyr.y();  data.gyroZ = gyr.z();
    data.magX = mag.x();   data.magY = mag.y();   data.magZ = mag.z();
    data.roll = eul.x();   data.pitch = eul.y();  data.yaw = eul.z();
    data.temp = _bno.getTemp();
    data.timestamp_us = micros();

    return true;
}

bool IMUBNO055::isHealthy() const {
    return _healthy;
}

bool IMUBNO055::setMode(ImuOpMode mode) {
    adafruit_bno055_opmode_t native = convertImuOpMode(mode);
    _bno.setMode(native);
    delay(30); // Per BNO055 datasheet, wait 19ms after mode change
    return _bno.getMode() == native;
}