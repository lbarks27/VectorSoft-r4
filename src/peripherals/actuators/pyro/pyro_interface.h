#pragma once

class IPyroChannel {
public:
    virtual void arm() = 0;
    virtual void disarm() = 0;
    virtual void fire() = 0;
    virtual bool isArmed() const = 0;
    virtual bool hasFired() const = 0;
    virtual ~IPyroChannel() {}
};