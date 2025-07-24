#include "pyro_stub.h"
#include <Arduino.h>

PyroStub::PyroStub(int channelNum)
    : _channelNum(channelNum), _armed(false), _fired(false) {}

void PyroStub::arm() {
    _armed = true;
    Serial.print("PyroStub["); Serial.print(_channelNum); Serial.println("] ARMED");
}
void PyroStub::disarm() {
    _armed = false;
    Serial.print("PyroStub["); Serial.print(_channelNum); Serial.println("] DISARMED");
}
void PyroStub::fire() {
    if (_armed && !_fired) {
        Serial.print("PyroStub["); Serial.print(_channelNum); Serial.println("] FIRED");
        _fired = true;
    }
}
bool PyroStub::isArmed()  const { return _armed; }
bool PyroStub::hasFired() const { return _fired; }