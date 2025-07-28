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
constexpr float TVC_MAX_DEFLECTION = 30.0f;

// Safety thresholds
constexpr float SAFETY_ABORT_TILT_DEG = 30.0f;
constexpr float SAFETY_MAX_ALTITUDE_M = 10000.0f;
constexpr float SAFETY_MIN_THRUST     = 0.1f;

// Future tunable parameters can be added here (PID gains, rates, etc.)

constexpr int NUM_ESC_CHANNELS = 3; // Update as needed
constexpr int ESC_PINS[NUM_ESC_CHANNELS] = {3, 4, 5}; // Your ESC PWM pins

// Per-ESC calibration values
constexpr int ESC_PWM_MIN[NUM_ESC_CHANNELS] = {1040, 1060, 1020};
constexpr int ESC_PWM_MAX[NUM_ESC_CHANNELS] = {1960, 1980, 1950};
constexpr float ESC_THRUST_MIN[NUM_ESC_CHANNELS] = {0.0f, 0.0f, 0.0f};
constexpr float ESC_THRUST_MAX[NUM_ESC_CHANNELS] = {2.5f, 2.3f, 2.4f};
constexpr bool ENABLE_ESC[NUM_ESC_CHANNELS] = {true, true, true};
constexpr bool TEST_MODE = false; // false for flight hardware

constexpr int SD_CS_PIN = 10;    // SD card CS pin

#define IMU_I2C_ADDR 0x28
#define IMU_I2C_PORT &Wire  // or &Wire1 for alternate ports

// ===================== SYSTEM IDENTITY & VERSIONING =====================
constexpr const char* FIRMWARE_VERSION = "v1.0.0";
constexpr const char* PROJECT_NAME     = "VectorSoft r4";

// ===================== EXTERNAL CONFIG REFERENCES =====================
extern const char* CONFIG_FILE_PATH;  // Path to config file on SD card or EEPROM

// Additional config variables or structs can be added here