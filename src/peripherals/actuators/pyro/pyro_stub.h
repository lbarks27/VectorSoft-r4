#pragma once
#include "pyro_interface.h"

class PyroStub : public IPyroChannel {
public:
    PyroStub(int channelNum = 0);
    void arm() override;
    void disarm() override;
    void fire() override;
    bool isArmed() const override;
    bool hasFired() const override;
private:
    int _channelNum;
    bool _armed;
    bool _fired;
};