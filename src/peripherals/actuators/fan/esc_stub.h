#pragma once
#include "esc_interface.h"

class ESCStub : public IESC {
public:
    ESCStub(int channelNum = 0);
    void arm() override;
    void disarm() override;
    void setThrottle(float throttle) override;
    void calibrate() override;
    bool isArmed() const override;
private:
    int _channelNum;
    bool _armed;
    float _lastThrottle;
};