#pragma once
#include "utils/math/math_utils.h"
#include <stdint.h>
#include <stddef.h>
#include <array>

#ifndef DEG2RAD
#define DEG2RAD (0.01745329251994329576923690768489f)
#endif

// Define enums for modes/calibration later
enum class IMUFusionMode {
    CONFIG,      // Configuration mode (BNO)
    NDOF,        // 9DOF fusion (Gyro + Accel + Mag)
    IMU,         // 6DOF fusion (Gyro + Accel only)
    COMPASS,     // Accel + Mag
    M4G,         // Accel + Mag, no gyro fusion (BNO)
    // Generic fallback
    NONE,        // Raw sensor, no fusion
    AUTO,        // Let driver choose best
    CUSTOM       // For custom/external fusion
    // Add more as you encounter new sensor families!
};

enum class IMUPowerMode {
    NORMAL,
    LOWPOWER,
    SUSPEND
};

struct IMUCalibrationStatus {
    uint8_t sys;     // System calibration (0-3)
    uint8_t gyro;    // Gyro calibration (0-3)
    uint8_t accel;   // Accel calibration (0-3)
    uint8_t mag;     // Mag calibration (0-3)
};

enum class IMUOrientation {
    FORWARD_UP,
    FORWARD_DOWN,
    RIGHT_UP,
    RIGHT_DOWN,
    LEFT_UP,
    LEFT_DOWN,
    BACKWARD_UP,
    BACKWARD_DOWN,
    // ...extend as needed
};

class IMUBase {
public:
    virtual ~IMUBase() {}

    // ----- Initialization and Configuration -----
    virtual bool begin() = 0;
    virtual bool reset() { return false; } // Optional: software reset
    virtual bool setFusionMode(IMUFusionMode mode) { (void)mode; return false; }
    virtual bool setPowerMode(IMUPowerMode mode) { (void)mode; return false; }

    // ----- Data Acquisition -----
    virtual bool read() = 0;   // Returns true if new valid data is available

    // ----- Sensor Data Access -----
    virtual Vector3 getAccel() const = 0;         // m/s^2
    virtual Vector3 getGyro() const = 0;          // rad/s
    virtual Vector3 getMag() const = 0;           // microtesla
    virtual float getTemperature() const { return 0.0f; } // Celsius
    virtual Quaternion getQuat() const { return Quaternion(); }
    virtual void getEuler(float& roll, float& pitch, float& yaw) const { roll = pitch = yaw = 0.0f; }
    virtual Vector3 getLinAccel() const { return Vector3(); }    // Linear acceleration (accel minus gravity)
    virtual Vector3 getGravity() const { return Vector3(); }     // Gravity vector
    virtual IMUOrientation getOrientation() const = 0;

    // ----- Calibration and Health -----
    virtual bool healthy() const = 0;              // True if sensor online and working
    virtual bool dataAvailable() const = 0;        // True if new data available
    virtual IMUCalibrationStatus getCalibrationStatus() const { return {0,0,0,0}; }
    virtual bool isFullyCalibrated() const { return false; }
    virtual bool setOffsets(const Vector3& accel, const Vector3& gyro, const Vector3& mag) { (void)accel; (void)gyro; (void)mag; return false; }
    virtual bool getOffsets(Vector3& accel, Vector3& gyro, Vector3& mag) const { (void)accel; (void)gyro; (void)mag; return false; }

    // ----- Identification -----
    virtual uint8_t getChipID() const { return 0; }
    virtual uint8_t getSoftwareVersion() const { return 0; }

    // ----- Error Codes and Status -----
    virtual uint8_t getErrorCode() const { return 0; }
    virtual uint8_t getSystemStatus() const { return 0; }

    // ----- Miscellaneous -----
    virtual bool setUnitsSI() { return false; }
    virtual bool setUnitsNonSI() { return false; }

    // ----- Raw Register Access (optional for advanced use) -----
    virtual bool readRegister(uint8_t reg, uint8_t& value) { (void)reg; (void)value; return false; }
    virtual bool writeRegister(uint8_t reg, uint8_t value) { (void)reg; (void)value; return false; }

    IMUBase(IMUOrientation orientation = IMUOrientation::FORWARD_UP)
      : _orientation(orientation) {}

    void setOrientation(IMUOrientation o) { _orientation = o; }

    /**
     * Helper to remap a vector from sensor frame to body frame.
     * All derived IMU drivers should use this on all vector outputs.
     */
    static Vector3 remapToBody(const Vector3& v, IMUOrientation ori) {
        switch (ori) {
            case IMUOrientation::FORWARD_UP:    return Vector3( v.x,  v.y,  v.z);
            case IMUOrientation::FORWARD_DOWN:  return Vector3( v.x, -v.y, -v.z);
            case IMUOrientation::RIGHT_UP:      return Vector3( v.y, -v.x,  v.z);
            case IMUOrientation::RIGHT_DOWN:    return Vector3( v.y,  v.x, -v.z);
            case IMUOrientation::LEFT_UP:       return Vector3(-v.y,  v.x,  v.z);
            case IMUOrientation::LEFT_DOWN:     return Vector3(-v.y, -v.x, -v.z);
            case IMUOrientation::BACKWARD_UP:   return Vector3(-v.x, -v.y,  v.z);
            case IMUOrientation::BACKWARD_DOWN: return Vector3(-v.x,  v.y, -v.z);
            default:                            return v;
        }
    }

    /**
     * Remapped getters: use orientation quaternion to map to body frame.
     * These call the protected virtual raw getters, implemented by derived classes.
     * The default implementation rotates the raw sensor output into the body frame.
     */
    virtual Quaternion getQuatRemapped() const {
        // NOTE: Quaternion::operator* not defined, must use Quaternion::multiply(a, b)
        // Remap the raw quaternion using the orientation quaternion.
        return Quaternion::multiply(orientationQuaternion(_orientation), getQuatRaw());
    }
    virtual Vector3 getAccelRemapped() const {
        // Remap the raw acceleration, subtracting bias, into the body frame.
        return orientationQuaternion(_orientation).rotate(getRawAccel() - _accelBias);
    }
    virtual Vector3 getGyroRemapped() const {
        // Remap the raw gyro vector into the body frame.
        return orientationQuaternion(_orientation).rotate(getRawGyro());
    }
    virtual Vector3 getMagRemapped() const {
        // Remap the raw magnetometer vector into the body frame.
        return orientationQuaternion(_orientation).rotate(getRawMag());
    }
    virtual Vector3 getLinAccelRemapped() const {
        // Remap the raw linear acceleration vector into the body frame.
        return orientationQuaternion(_orientation).rotate(getRawLinAccel());
    }
    virtual Vector3 getGravityRemapped() const {
        // Remap the raw gravity vector into the body frame.
        return orientationQuaternion(_orientation).rotate(getRawGravity());
    }

    /**
     * Helper to get the sensor-to-body orientation quaternion for the given IMUOrientation.
     * This quaternion rotates from sensor (IMU chip) frame to body frame.
     * All derived IMU drivers should use this on all quaternion outputs.
     * Uses Euler XYZ order for rotations.
     */
    /**
     * Returns the sensor-to-body orientation quaternion for the given IMUOrientation.
     * This quaternion rotates from sensor (IMU chip) frame to body frame.
     * Uses Euler XYZ order for rotations.
     */
    static inline Quaternion orientationQuaternion(IMUOrientation ori) {
        switch (ori) {
            case IMUOrientation::FORWARD_UP:    return Quaternion::fromEuler(0.0f, 0.0f, 0.0f, EulerOrder::XYZ);
            case IMUOrientation::FORWARD_DOWN:  return Quaternion::fromEuler(180.0f * DEG2RAD, 0.0f, 0.0f, EulerOrder::XYZ);
            case IMUOrientation::RIGHT_UP:      return Quaternion::fromEuler(-90.0f * DEG2RAD, 0.0f, 90.0f * DEG2RAD, EulerOrder::XYZ);
            case IMUOrientation::RIGHT_DOWN:    return Quaternion::fromEuler(90.0f * DEG2RAD, 0.0f, 90.0f * DEG2RAD, EulerOrder::XYZ);
            case IMUOrientation::LEFT_UP:       return Quaternion::fromEuler(-90.0f * DEG2RAD, 0.0f, -90.0f * DEG2RAD, EulerOrder::XYZ);
            case IMUOrientation::LEFT_DOWN:     return Quaternion::fromEuler(90.0f * DEG2RAD, 0.0f, -90.0f * DEG2RAD, EulerOrder::XYZ);
            case IMUOrientation::BACKWARD_UP:   return Quaternion::fromEuler(0.0f, 0.0f, 180.0f * DEG2RAD, EulerOrder::XYZ);
            case IMUOrientation::BACKWARD_DOWN: return Quaternion::fromEuler(180.0f * DEG2RAD, 0.0f, 180.0f * DEG2RAD, EulerOrder::XYZ);
            default:                            return Quaternion::fromEuler(0.0f, 0.0f, 0.0f, EulerOrder::XYZ);
        }
    }

    /**
     * Optionally remap a quaternion from sensor to body frame using orientation.
     * Equivalent to: orientationQuaternion(ori) * q
     */
    /**
     * Optionally remap a quaternion from sensor to body frame using orientation.
     * Equivalent to: orientationQuaternion(ori) * q
     */
    static inline Quaternion remapQuatToBody(const Quaternion& q, IMUOrientation ori) {
        // NOTE: Quaternion::operator* not defined, must use Quaternion::multiply(a, b)
        return Quaternion::multiply(orientationQuaternion(ori), q);
    }

protected:
    // NOTE: All derived IMU drivers should call remapToBody()/orientationQuaternion()/remapQuatToBody()
    // on all vector and quaternion outputs before returning them to ensure correct body frame alignment.
    IMUOrientation _orientation;
    Vector3 _accelBias = Vector3(0,0,0); ///< Acceleration bias (in sensor frame), default zero.

    // ---- Raw sensor getters: must be implemented by derived classes ----
    /**
     * Pure virtual raw getters: implemented by derived class to provide raw sensor data.
     * These are used by the remapped getters.
     */
    virtual Quaternion getQuatRaw() const = 0;
    virtual Vector3 getRawAccel() const = 0;
    virtual Vector3 getRawGyro() const = 0;
    virtual Vector3 getRawMag() const = 0;
    virtual Vector3 getRawLinAccel() const = 0;
    virtual Vector3 getRawGravity() const = 0;
};
