/*
1. System-wide Configuration
	•	Flight/ground test mode flags
	•	Enable/disable features (ESC outputs, TVC, pyros, logging, telemetry, etc.)
	•	Hardware pin assignments (if not in a dedicated defines.h)
	•	Flight mode selection (drone, rocket, hybrid)

2. Tunable Parameters & Limits
	•	Control loop rates (Hz)
	•	TVC max/min angles
	•	Servo rate/limits
	•	Motor thrust min/max
	•	Safety thresholds (abort triggers, allowed tilt, max altitude, etc.)
	•	Dead-reckoning fallback thresholds

3. Compile-time Constants
	•	Physical constants (gravity, standard atmosphere, etc.)
	•	Conversion factors (deg2rad, etc.)
	•	Mathematical tolerances (epsilon, filter constants)

4. System Identity & Versioning
	•	Firmware version, project name
	•	Vehicle identifier (if applicable)

5. Optionally, References to External Config
	•	Paths or keys for config files (if you load settings from SD card or groundstation)*/

#pragma once

// Feature enable flags
extern bool enable_esc;
extern bool enable_tvc;
extern bool enable_pyros;
extern bool enable_logging;
extern bool enable_telemetry;
extern bool enable_debug;
extern bool enable_SITL;
extern bool enable_ground_test;
extern bool debug_mode;
#define DBG_PRINTLN(msg)    if (DEBUG_MODE) { Serial.println(msg); }
#define DBG_PRINT(msg)      if (DEBUG_MODE) { Serial.print(msg); }


//TUNABLE PARAMETERS & LIMITS

// TVC limits (degrees)
#define TVC_MAX_DEFLECTION 45.0f

// Safety thresholds
#define SAFETY_ABORT_TILT_DEG 30.0f
#define SAFETY_MAX_ALTITUDE_M 10000.0f
#define SAFETY_MIN_THRUST 0.1f

// Future tunable parameters can be added here


//EXTERNAL CONFIG REFERENCES

extern const char* CONFIG_FILE_PATH;  // Path to config file on SD card or EEPROM

// Note: Config file loading from SD card or EEPROM can be implemented here

