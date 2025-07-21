#include "utils/params/params.h"

// Feature Flags - initial state is always safe (false/disarmed)
bool enable_esc = false;
bool enable_tvc = false;
bool enable_pyros = false;
bool enable_logging = false;
bool enable_telemetry = false;
bool enable_debug = false;
bool enable_SITL = false;
bool enable_ground_test = true;
bool debug_mode = true;

// Tunable parameters - set defaults for safe bench testing
float TVC_MAX_ANGLE = 25.0f;  // degrees
float CONTROL_LOOP_HZ = 200.0f;

// System mode - start in ground test or disarmed state
SysState sys_state = SysState::Drone;

// Add additional variables as needed
// e.g., float MAX_ALLOWED_TILT_DEG = 45.0f;