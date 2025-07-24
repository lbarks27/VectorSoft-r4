#pragma once
#include <Arduino.h>

enum class ImuOpMode {
    CONFIG,
    ACCONLY,
    MAGONLY,
    GYRONLY,
    ACCMAG,
    ACCGYRO,
    MAGGYRO,
    AMG,
    IMU,
    COMPASS,
    M4G,
    NDOF_FMC_OFF,
    NDOF
};

struct ImuData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float temp;
    float roll, pitch, yaw;
    uint32_t timestamp_us;
};

class IIMU {
public:
    virtual bool begin() = 0;
    virtual bool read(ImuData& data) = 0;
    virtual bool isHealthy() const = 0;
    virtual bool setMode(ImuOpMode mode) = 0; // NEW
    virtual ~IIMU() {}
};