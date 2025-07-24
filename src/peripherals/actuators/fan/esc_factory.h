#pragma once
#include "esc_interface.h"
#include "esc_real.h"
#include "esc_stub.h"
#include "utils/params/params.h"

inline IESC* createESC(int idx, bool testMode) {
    if (testMode)
        return new ESCStub(idx + 1);
    else
        return new ESCReal(
            ESC_PINS[idx],
            ESC_PWM_MIN[idx],
            ESC_PWM_MAX[idx],
            ESC_THRUST_MIN[idx],
            ESC_THRUST_MAX[idx]
        );
}