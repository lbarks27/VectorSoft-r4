#include "utils/params/params.h"

// Feature Flags - initial state is always safe (false/disarmed)
bool ENABLE_ESC_OUTPUTS = false;
bool ENABLE_TVC_OUTPUTS = false;
bool ENABLE_PYRO_OUTPUTS = false;

// Tunable parameters - set defaults for safe bench testing
float TVC_MAX_ANGLE = 25.0f;  // degrees
float CONTROL_LOOP_HZ = 200.0f;

// System mode - start in ground test or disarmed state
SysMode systemMode = SysMode::Drone;

// Add additional variables as needed
// e.g., float MAX_ALLOWED_TILT_DEG = 45.0f;