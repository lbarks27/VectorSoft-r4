#pragma once

#include <cstdint>
#include "utils/math/math_utils.h"

enum class navigationOpMode {
    STUB,
    GROUND,
    MULTI_TVC,
    DIFF_THROTTLE,
};

struct navigationInput {
    float roll, pitch, yaw;
    Vector3 position; // Position in meters
    float throttle;
    uint32_t timestamp_us;
};

struct navigationOutput {
    float roll, pitch, yaw;
    Vector3 position; // Position in meters
    float throttle;
    uint32_t timestamp_us;
};

struct navigationTelemetry {

};