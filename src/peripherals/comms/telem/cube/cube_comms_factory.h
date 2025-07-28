#pragma once
#include "cube_comms_interface.h"
#include "cube_comms_stub.h"
// #include "cube_comms_real.h" // Real driver, to be implemented

inline ICubeComms* createCubeComms(bool testMode) {
    if (testMode) {
        return new CubeCommsStub();
    } else {
        // return new CubeCommsReal(/* params as needed */);
        // For now, return stub until your real driver is ready:
        return new CubeCommsStub();
    }
}