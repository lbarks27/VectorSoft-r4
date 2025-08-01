#pragma once

#include <cstdint>
#include "utils/math/math_utils.h"

enum class controlOpMode {
    STUB,
    GROUND,
    MULTI_TVC,
    DIFF_THROTTLE,
};

struct controlInput {
    float roll, pitch, yaw;
    Vector3 position; // Position in meters
    float throttle;
    uint32_t timestamp_us;
};

struct controlOutput {
    float roll, pitch, yaw;
    Vector3 position; // Position in meters
    float throttle;
    uint32_t timestamp_us;
};

struct controlTelemetry {

};