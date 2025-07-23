/*
1. System-wide Configuration
    • Flight/ground test mode flags
    • Enable/disable features (ESC outputs, TVC, pyros, logging, telemetry, etc.)
    • Hardware pin assignments (if not in a dedicated defines.h)
    • Flight mode selection (drone, rocket, hybrid)

2. Tunable Parameters & Limits
    • Control loop rates (Hz)
    • TVC max/min angles
    • Servo rate/limits
    • Motor thrust min/max
    • Safety thresholds (abort triggers, allowed tilt, max altitude, etc.)
    • Dead-reckoning fallback thresholds

3. Compile-time Constants
    • Physical constants (gravity, standard atmosphere, etc.)
    • Conversion factors (deg2rad, etc.)
    • Mathematical tolerances (epsilon, filter constants)

4. System Identity & Versioning
    • Firmware version, project name
    • Vehicle identifier (if applicable)

5. Optionally, References to External Config
    • Paths or keys for config files (if you load settings from SD card or groundstation)
*/

#pragma once

#include <Arduino.h>
#include "peripherals/sensors/imu/imu_base.h"   // For IMUOrientation and helpers

// ===================== FEATURE ENABLE FLAGS =====================
extern bool enable_esc;
extern bool enable_tvc;
extern bool enable_pyros;
extern bool enable_logging;
extern bool enable_telemetry;
extern bool enable_debug;
extern bool enable_SITL;
extern bool enable_ground_test;
extern bool debug_mode;

// Debug print macros
#define DBG_PRINTLN(msg)    if (debug_mode) { Serial.println(msg); }
#define DBG_PRINT(msg)      if (debug_mode) { Serial.print(msg); }

// ===================== TUNABLE PARAMETERS & LIMITS =====================

// TVC limits (degrees)
constexpr float TVC_MAX_DEFLECTION = 45.0f;

// Safety thresholds
constexpr float SAFETY_ABORT_TILT_DEG = 30.0f;
constexpr float SAFETY_MAX_ALTITUDE_M = 10000.0f;
constexpr float SAFETY_MIN_THRUST     = 0.1f;

// Future tunable parameters can be added here (PID gains, rates, etc.)

// ===================== IMU ORIENTATION CONFIG =====================
// Example for two IMUs (expand as needed)
constexpr IMUOrientation IMU0_ORIENTATION = IMUOrientation::FORWARD_UP;

// ===================== SYSTEM IDENTITY & VERSIONING =====================
constexpr const char* FIRMWARE_VERSION = "v1.0.0";
constexpr const char* PROJECT_NAME     = "VectorSoft r4";

// ===================== EXTERNAL CONFIG REFERENCES =====================
extern const char* CONFIG_FILE_PATH;  // Path to config file on SD card or EEPROM

// Additional config variables or structs can be added here