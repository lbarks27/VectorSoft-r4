#pragma once
#include "power_interface.h"
#include "power_flvss.h"
#include "power_stub.h"
#include "utils/params/hw_pin_assign.h"
#include "utils/params/params.h"

inline IPowerMonitor* createPowerMonitor(bool testMode) {
    if (testMode)
        return new PowerStub();
    else
        return new PowerFLVSS(FLVSS_SERIAL);
}