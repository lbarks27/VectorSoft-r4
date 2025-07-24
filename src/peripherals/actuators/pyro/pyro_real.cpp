#include "pyro_real.h"
#include <Arduino.h>

PyroReal::PyroReal(int pin)
    : _pin(pin), _armed(false), _fired(false)
{
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW); // Ensure safe state at boot
}

void PyroReal::arm()    { _armed = true; }
void PyroReal::disarm() { _armed = false; }
void PyroReal::fire()   {
    if (_armed && !_fired) {
        digitalWrite(_pin, HIGH);
        _fired = true;
    }
}
bool PyroReal::isArmed()  const { return _armed; }
bool PyroReal::hasFired() const { return _fired; }