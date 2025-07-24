#pragma once
#include "pyro_interface.h"

class PyroReal : public IPyroChannel {
public:
    PyroReal(int pin);
    void arm() override;
    void disarm() override;
    void fire() override;
    bool isArmed() const override;
    bool hasFired() const override;
private:
    int _pin;
    bool _armed;
    bool _fired;
};