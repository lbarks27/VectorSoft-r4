#pragma once
#include "pyro_interface.h"
#include "pyro_real.h"
#include "pyro_stub.h"

inline IPyroChannel* createPyroChannel(int pin, bool testMode, int channelNum = 0) {
    if (testMode)
        return new PyroStub(channelNum);
    else
        return new PyroReal(pin);
}