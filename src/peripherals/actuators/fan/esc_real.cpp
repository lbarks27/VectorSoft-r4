#include "esc_real.h"
#include <Arduino.h>

ESCReal::ESCReal(int pwmPin, int pwmMin, int pwmMax, float thrustMin, float thrustMax)
    : _pwmPin(pwmPin), _pwmMin(pwmMin), _pwmMax(pwmMax), _thrustMin(thrustMin), _thrustMax(thrustMax),
      _armed(false), _lastThrottle(0.0f)
{
    _servo.attach(_pwmPin);
    writeMicroseconds(_pwmMin);
}

void ESCReal::arm() {
    _armed = true;
    setThrottle(0.0f);
}

void ESCReal::disarm() {
    _armed = false;
    setThrottle(0.0f);
}

void ESCReal::setThrottle(float throttle) {
    if (!_armed) return;
    _lastThrottle = constrain(throttle, 0.0f, 1.0f);
    writeMicroseconds(throttleToPWM(_lastThrottle));
}

void ESCReal::setCalibration(int pwmMin, int pwmMax, float thrustMin, float thrustMax) {
    _pwmMin = pwmMin;
    _pwmMax = pwmMax;
    _thrustMin = thrustMin;
    _thrustMax = thrustMax;
}

void ESCReal::calibrate() {
    writeMicroseconds(_pwmMax);
    delay(2000);
    writeMicroseconds(_pwmMin);
    delay(2000);
}

bool ESCReal::isArmed() const { return _armed; }

void ESCReal::writeMicroseconds(int us) {
    _servo.writeMicroseconds(us);
}

int ESCReal::throttleToPWM(float throttle) const {
    // Simple linear mapping
    return _pwmMin + static_cast<int>(throttle * (_pwmMax - _pwmMin));
}