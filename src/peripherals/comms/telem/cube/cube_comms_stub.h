#pragma once
#include "cube_comms_interface.h"

class CubeCommsStub : public ICubeComms {
public:
    bool begin() override { return true; }
    bool send(const CubeMsg& msg) override {
        // Optionally log/print simulated message
        return true;
    }
    bool receive(CubeMsg& msg) override {
        // Simulate no incoming messages (or add test data here)
        return false;
    }
    bool isHealthy() const override { return true; }
};