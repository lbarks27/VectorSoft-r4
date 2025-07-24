#include "esc_stub.h"
#include <Arduino.h>

ESCStub::ESCStub(int channelNum)
    : _channelNum(channelNum), _armed(false), _lastThrottle(0.0f) {}

void ESCStub::arm() {
    _armed = true;
    Serial.print("ESCStub["); Serial.print(_channelNum); Serial.println("] ARMED");
}
void ESCStub::disarm() {
    _armed = false;
    Serial.print("ESCStub["); Serial.print(_channelNum); Serial.println("] DISARMED");
}
void ESCStub::setThrottle(float throttle) {
    if (!_armed) return;
    _lastThrottle = constrain(throttle, 0.0f, 1.0f);
    Serial.print("ESCStub["); Serial.print(_channelNum); Serial.print("] THROTTLE: ");
    Serial.println(_lastThrottle, 3);
}
void ESCStub::calibrate() {
    Serial.print("ESCStub["); Serial.print(_channelNum); Serial.println("] CALIBRATING...");
}
bool ESCStub::isArmed() const { return _armed; }