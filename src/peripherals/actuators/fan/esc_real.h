#pragma once
#include "esc_interface.h"
#include <Servo.h>

class ESCReal : public IESC {
public:
    ESCReal(int pwmPin, int pwmMin, int pwmMax, float thrustMin, float thrustMax);
    void arm() override;
    void disarm() override;
    void setThrottle(float throttle) override;
    void calibrate() override;
    bool isArmed() const override;
    void setCalibration(int pwmMin, int pwmMax, float thrustMin, float thrustMax);

private:
    Servo _servo;
    int _pwmPin, _pwmMin, _pwmMax;
    float _thrustMin, _thrustMax;
    bool _armed;
    float _lastThrottle;
    void writeMicroseconds(int us);
    int throttleToPWM(float throttle) const;
};