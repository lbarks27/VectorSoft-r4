#pragma once

#include <cstdint>
#include "utils/math/math_utils.h"

enum class guidanceOpMode {
    STUB,
    GROUND,
    MULTI_TVC,
    DIFF_THROTTLE,
};

struct guidanceInput {
    float roll, pitch, yaw;
    Vector3 position; // Position in meters
    float throttle;
    uint32_t timestamp_us;
};

struct guidanceOutput {
    float roll, pitch, yaw;
    Vector3 position; // Position in meters
    float throttle;
    uint32_t timestamp_us;
};

struct guidanceTelemetry {

};