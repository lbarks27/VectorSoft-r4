#pragma once
#include "imu_interface.h"
#include <Adafruit_BNO055.h>

class IMUBNO055 : public IIMU {
public:
    IMUBNO055(uint8_t address, TwoWire* i2cPort);
    bool begin() override;
    bool read(ImuData& data) override;
    bool isHealthy() const override;
    bool setMode(ImuOpMode mode) override;
private:
    Adafruit_BNO055 _bno;
    bool _healthy;
};