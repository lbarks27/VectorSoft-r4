#pragma once
#include "imu_interface.h"
#include "imu_bno055.h"
#include "imu_stub.h"
#include "utils/params/hw_pin_assign.h"
#include "utils/params/params.h"

inline IIMU* createIMU(bool testMode) {
    if (testMode)
        return new IMUStub();
    else
        return new IMUBNO055(IMU_I2C_ADDR, IMU_I2C_PORT);
}