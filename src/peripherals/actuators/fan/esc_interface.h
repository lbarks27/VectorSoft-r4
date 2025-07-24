#pragma once

class IESC {
public:
    virtual void arm() = 0;
    virtual void disarm() = 0;
    virtual void setThrottle(float throttle) = 0; // 0.0 - 1.0
    virtual void calibrate() = 0;
    virtual bool isArmed() const = 0;
    virtual ~IESC() {}
};