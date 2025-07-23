#include <cmath>

#pragma once

// ==== Euler Class ====
enum class EulerOrder {
    XYZ,
    XZY,
    YXZ,
    YZX,
    ZXY,
    ZYX
};


// ==== Vector3 Struct ====
struct Vector3 {
    float x, y, z;

    // Default constructor (zero vector)
    constexpr Vector3() : x(0), y(0), z(0) {}
    constexpr Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // Cross product
    static Vector3 cross(const Vector3& a, const Vector3& b) {
        return Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    /**
     * @brief Checks if all components of the vector are finite.
     *
     * This method returns true if x, y, and z are all finite values (not NaN or infinity).
     * Use this to ensure the vector is valid for mathematical operations.
     *
     * @return True if all components are finite, false otherwise.
     */
    bool isFinite() const {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    /**
     * @brief Checks if the vector is normalized (i.e., has unit length) within a tolerance.
     *
     * This method compares the vector's norm to 1 within the given epsilon.
     * Use this to verify if a vector is a unit vector before using it for direction or orientation.
     *
     * @param eps The allowable deviation from unit length (default 1e-6f).
     * @return True if the norm is within eps of 1, false otherwise.
     */
    bool isNormalized(float eps = 1e-6f) const {
        float n = std::sqrt(x * x + y * y + z * z);
        return std::fabs(n - 1.0f) < eps;
    }
};


// ==== Quaternion Struct ====
struct Quaternion {
    float w, x, y, z;
    constexpr Quaternion() : w(1), x(0), y(0), z(0) {}
    constexpr Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion inverse() const {
        float n2 = w*w + x*x + y*y + z*z;
        if (n2 < 1e-6f) return Quaternion(1, 0, 0, 0);
        return Quaternion(w/n2, -x/n2, -y/n2, -z/n2);
    }

    float norm() const {
        return std::sqrt(w*w + x*x + y*y + z*z);
    }

    static float dot(const Quaternion& a, const Quaternion& b) {
        return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    }

    static Quaternion multiply(const Quaternion& q1, const Quaternion& q2) {
        // Hamilton product
        return Quaternion(
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        );
    }

    Quaternion normalized() const {
        float n = norm();
        if (n < 1e-6f) return Quaternion(1, 0, 0, 0);
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    bool isFinite() const {
        return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    bool isNormalized(float eps = 1e-6f) const {
        float n = norm();
        return std::fabs(n - 1.0f) < eps;
    }

    bool isIdentity(float eps = 1e-6f) const {
        return std::fabs(w - 1.0f) < eps && std::fabs(x) < eps &&
               std::fabs(y) < eps && std::fabs(z) < eps;
    }

    void setIdentity() {
        w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
    }

    static Quaternion fromEuler(float a, float b, float c, EulerOrder order) {
        // Create single-axis quaternions
        float ca = std::cos(a * 0.5f), sa = std::sin(a * 0.5f);
        float cb = std::cos(b * 0.5f), sb = std::sin(b * 0.5f);
        float cc = std::cos(c * 0.5f), sc = std::sin(c * 0.5f);

        Quaternion qA, qB, qC;
        // Assign single-axis quaternions for each axis
        switch (order) {
            case EulerOrder::XYZ:
                qA = Quaternion(ca, sa, 0, 0);
                qB = Quaternion(cb, 0, sb, 0);
                qC = Quaternion(cc, 0, 0, sc);
                break;
            case EulerOrder::XZY:
                qA = Quaternion(ca, sa, 0, 0);
                qB = Quaternion(cb, 0, 0, sb);
                qC = Quaternion(cc, 0, sc, 0);
                break;
            case EulerOrder::YXZ:
                qA = Quaternion(ca, 0, sa, 0);
                qB = Quaternion(cb, sb, 0, 0);
                qC = Quaternion(cc, 0, 0, sc);
                break;
            case EulerOrder::YZX:
                qA = Quaternion(ca, 0, sa, 0);
                qB = Quaternion(cb, 0, 0, sb);
                qC = Quaternion(cc, sc, 0, 0);
                break;
            case EulerOrder::ZXY:
                qA = Quaternion(ca, 0, 0, sa);
                qB = Quaternion(cb, sb, 0, 0);
                qC = Quaternion(cc, 0, sc, 0);
                break;
            case EulerOrder::ZYX:
                qA = Quaternion(ca, 0, 0, sa);
                qB = Quaternion(cb, 0, sb, 0);
                qC = Quaternion(cc, sc, 0, 0);
                break;
            default:
                return Quaternion(1,0,0,0);
        }

        // Combine in the specified intrinsic order: Q = qA * qB * qC
        // (intrinsic: first axis, then second, then third)
        return Quaternion::multiply(Quaternion::multiply(qA, qB), qC);
    }

    void toEuler(float& a, float& b, float& c, EulerOrder order) const {
        // Abbreviations for readability
        const float qw = w, qx = x, qy = y, qz = z;
        switch(order) {
            case EulerOrder::ZYX: {
                // yaw (Z), pitch (Y), roll (X)
                float siny_cosp = 2.0f * (qw * qz + qx * qy);
                float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
                a = std::atan2(siny_cosp, cosy_cosp); // yaw

                float sinp = 2.0f * (qw * qy - qz * qx);
                if (std::fabs(sinp) >= 1.0f)
                    b = std::copysign(M_PI/2.0f, sinp); // pitch
                else
                    b = std::asin(sinp);

                float sinr_cosp = 2.0f * (qw * qx + qy * qz);
                float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
                c = std::atan2(sinr_cosp, cosr_cosp); // roll
                break;
            }
            case EulerOrder::ZXY: {
                // yaw (Z), roll (X), pitch (Y)
                float sinp = 2.0f * (qw * qx - qy * qz);
                if (std::fabs(sinp) >= 1.0f)
                    b = std::copysign(M_PI/2.0f, sinp);
                else
                    b = std::asin(sinp);

                float siny_cosp = 2.0f * (qw * qy + qx * qz);
                float cosy_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
                a = std::atan2(siny_cosp, cosy_cosp);

                float sinz_cosp = 2.0f * (qw * qz + qx * qy);
                float cosz_cosp = 1.0f - 2.0f * (qx * qx + qz * qz);
                c = std::atan2(sinz_cosp, cosz_cosp);
                break;
            }
            case EulerOrder::YXZ: {
                // pitch (Y), roll (X), yaw (Z)
                float sinp = 2.0f * (qw * qx - qy * qz);
                if (std::fabs(sinp) >= 1.0f)
                    b = std::copysign(M_PI/2.0f, sinp);
                else
                    b = std::asin(sinp);

                float siny_cosp = 2.0f * (qw * qz + qx * qy);
                float cosy_cosp = 1.0f - 2.0f * (qx * qx + qz * qz);
                a = std::atan2(siny_cosp, cosy_cosp);

                float sinz_cosp = 2.0f * (qw * qy + qx * qz);
                float cosz_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
                c = std::atan2(sinz_cosp, cosz_cosp);
                break;
            }
            case EulerOrder::YZX: {
                // pitch (Y), yaw (Z), roll (X)
                float sinp = 2.0f * (qw * qz - qx * qy);
                if (std::fabs(sinp) >= 1.0f)
                    b = std::copysign(M_PI/2.0f, sinp);
                else
                    b = std::asin(sinp);

                float siny_cosp = 2.0f * (qw * qy + qz * qx);
                float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
                a = std::atan2(siny_cosp, cosy_cosp);

                float sinx_cosp = 2.0f * (qw * qx + qz * qy);
                float cosx_cosp = 1.0f - 2.0f * (qx * qx + qz * qz);
                c = std::atan2(sinx_cosp, cosx_cosp);
                break;
            }
            case EulerOrder::XZY: {
                // roll (X), yaw (Z), pitch (Y)
                float sinp = 2.0f * (qw * qz - qx * qy);
                if (std::fabs(sinp) >= 1.0f)
                    b = std::copysign(M_PI/2.0f, sinp);
                else
                    b = std::asin(sinp);

                float sinx_cosp = 2.0f * (qw * qx + qz * qy);
                float cosx_cosp = 1.0f - 2.0f * (qx * qx + qz * qz);
                a = std::atan2(sinx_cosp, cosx_cosp);

                float siny_cosp = 2.0f * (qw * qy + qz * qx);
                float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
                c = std::atan2(siny_cosp, cosy_cosp);
                break;
            }
            case EulerOrder::XYZ: {
                // roll (X), pitch (Y), yaw (Z)
                float siny_cosp = 2.0f * (qw * qz + qx * qy);
                float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
                c = std::atan2(siny_cosp, cosy_cosp); // yaw

                float sinp = 2.0f * (qw * qy - qz * qx);
                if (std::fabs(sinp) >= 1.0f)
                    b = std::copysign(M_PI/2.0f, sinp); // pitch
                else
                    b = std::asin(sinp);

                float sinx_cosp = 2.0f * (qw * qx + qy * qz);
                float cosx_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
                a = std::atan2(sinx_cosp, cosx_cosp); // roll
                break;
            }
            default:
                a = b = c = 0.0f;
                break;
        }
    }

    Vector3 rotate(const Vector3& v) const {
        // Quaternion-vector multiplication: v' = q * (0, v) * q^-1
        Quaternion qv(0, v.x, v.y, v.z);
        Quaternion q_conj = this->conjugate();
        Quaternion q_result = Quaternion::multiply(Quaternion::multiply(*this, qv), q_conj);
        return Vector3(q_result.x, q_result.y, q_result.z);
    }

    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t) {
        // Compute the cosine of the angle between the two quaternions
        float dot = Quaternion::dot(q1, q2);
        Quaternion q2b = q2;
        // If dot < 0, use the shortest path by negating q2
        if (dot < 0.0f) {
            q2b = Quaternion(-q2.w, -q2.x, -q2.y, -q2.z);
            dot = -dot;
        }
        const float DOT_THRESHOLD = 0.9995f;
        if (dot > DOT_THRESHOLD) {
            // If very close, use linear interpolation to avoid division by zero
            Quaternion result(
                q1.w + t * (q2b.w - q1.w),
                q1.x + t * (q2b.x - q1.x),
                q1.y + t * (q2b.y - q1.y),
                q1.z + t * (q2b.z - q1.z)
            );
            return result.normalized();
        }
        // Compute the angle between the quaternions
        float theta_0 = std::acos(dot);        // angle between input quaternions
        float theta = theta_0 * t;             // angle between q1 and result
        float sin_theta_0 = std::sin(theta_0);
        float sin_theta = std::sin(theta);
        float s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        float s1 = sin_theta / sin_theta_0;
        Quaternion result(
            q1.w * s0 + q2b.w * s1,
            q1.x * s0 + q2b.x * s1,
            q1.y * s0 + q2b.y * s1,
            q1.z * s0 + q2b.z * s1
        );
        return result.normalized();
    }

    void toAxisAngle(Vector3& axis, float& angle) const {
        Quaternion qn = this->normalized();
        angle = 2.0f * std::acos(qn.w);
        float s = std::sqrt(1.0f - qn.w * qn.w);
        if (s < 1e-6f) {
            // Axis not important; default to X axis
            axis = Vector3(1, 0, 0);
        } else {
            axis = Vector3(qn.x / s, qn.y / s, qn.z / s);
        }
    }

    static Quaternion fromAxisAngle(const Vector3& axis, float angle) {
        float half = angle * 0.5f;
        float s = std::sin(half);
        return Quaternion(std::cos(half), axis.x * s, axis.y * s, axis.z * s);
    }

    void toRotationMatrix(float R[3][3]) const {
        float ww = w*w, xx = x*x, yy = y*y, zz = z*z;
        float wx = w*x, wy = w*y, wz = w*z;
        float xy = x*y, xz = x*z, yz = y*z;
        R[0][0] = ww + xx - yy - zz;
        R[0][1] = 2*(xy - wz);
        R[0][2] = 2*(xz + wy);
        R[1][0] = 2*(xy + wz);
        R[1][1] = ww - xx + yy - zz;
        R[1][2] = 2*(yz - wx);
        R[2][0] = 2*(xz - wy);
        R[2][1] = 2*(yz + wx);
        R[2][2] = ww - xx - yy + zz;
    }
};



// Vector addition
/**
 * Adds two vectors component-wise.
 *
 * @param a The first vector operand.
 * @param b The second vector operand.
 * @return The resulting vector after addition.
 *
 * Usage: Commonly used to combine displacements or velocities.
 */
inline Vector3 operator+(const Vector3& a, const Vector3& b) {
    return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

// Vector subtraction
/**
 * Subtracts the second vector from the first component-wise.
 *
 * @param a The vector to subtract from.
 * @param b The vector to subtract.
 * @return The resulting vector after subtraction.
 *
 * Usage: Useful for calculating displacement or difference between points.
 */
inline Vector3 operator-(const Vector3& a, const Vector3& b) {
    return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

// Scalar multiplication (vector * scalar)
/**
 * Multiplies each component of the vector by a scalar.
 *
 * @param v The vector to be scaled.
 * @param s The scalar multiplier.
 * @return The scaled vector.
 *
 * Usage: Used to scale vectors by a constant factor.
 */
inline Vector3 operator*(const Vector3& v, float s) {
    return Vector3(v.x * s, v.y * s, v.z * s);
}

// Scalar multiplication (scalar * vector)
/**
 * Multiplies each component of the vector by a scalar (scalar on left).
 *
 * @param s The scalar multiplier.
 * @param v The vector to be scaled.
 * @return The scaled vector.
 *
 * Usage: Same as above but supports scalar first syntax.
 */
inline Vector3 operator*(float s, const Vector3& v) {
    return Vector3(v.x * s, v.y * s, v.z * s);
}

// Scalar division
/**
 * Divides each component of the vector by a scalar.
 *
 * @param v The vector to be divided.
 * @param s The scalar divisor.
 * @return The scaled vector after division.
 *
 * Note: Division by zero is undefined; ensure s != 0.
 */
inline Vector3 operator/(const Vector3& v, float s) {
    return Vector3(v.x / s, v.y / s, v.z / s);
}

// Unary negation
/**
 * Negates each component of the vector.
 *
 * @param v The vector to negate.
 * @return The negated vector.
 *
 * Usage: Useful for reversing direction of a vector.
 */
inline Vector3 operator-(const Vector3& v) {
    return Vector3(-v.x, -v.y, -v.z);
}

/**
 * Computes the dot product (scalar product) of two vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The dot product, representing the cosine of the angle times magnitudes.
 *
 * Usage: Used to find projection length, angle between vectors, or test orthogonality.
 */
inline float dot(const Vector3& a, const Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * Computes the cross product of two vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return A vector perpendicular to both a and b following the right-hand rule.
 *
 * Usage: Commonly used in 3D graphics and physics to find normals or rotational axes.
 */
inline Vector3 cross(const Vector3& a, const Vector3& b) {
    return Vector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

/**
 * Computes the Euclidean norm (length) of a vector.
 *
 * @param v The vector whose norm is to be calculated.
 * @return The length (magnitude) of the vector.
 *
 * Usage: Used to measure vector magnitude; important in normalization and distance calculations.
 */
inline float norm(const Vector3& v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Normalizes a vector to have unit length.
 *
 * @param v The vector to normalize.
 * @return A unit vector in the same direction as v, or zero vector if input is near zero.
 *
 * Note: Returns zero vector if the input vector's length is very small (to avoid division by zero).
 */
inline Vector3 normalize(const Vector3& v) {
    float n = norm(v);
    if (n < 1e-6f) return Vector3(0, 0, 0);
    return Vector3(v.x / n, v.y / n, v.z / n);
}

/**
 * Performs element-wise multiplication of two vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return A vector where each component is the product of corresponding components of a and b.
 *
 * Usage: Useful in graphics and physics for scaling components independently.
 */
inline Vector3 elemMul(const Vector3& a, const Vector3& b) {
    return Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
}

/**
 * Performs element-wise division of two vectors.
 *
 * @param a The numerator vector.
 * @param b The denominator vector.
 * @return A vector where each component is the quotient of corresponding components of a and b.
 *
 * Note: Division by zero in any component of b is undefined.
 */
inline Vector3 elemDiv(const Vector3& a, const Vector3& b) {
    return Vector3(a.x / b.x, a.y / b.y, a.z / b.z);
}

/**
 * Computes the angle in radians between two vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The angle in radians between vectors a and b.
 *
 * Note: Returns 0 if either vector is near zero length.
 * The dot product is clamped to [-1, 1] to avoid numerical errors causing NaN.
 */
inline float angleBetween(const Vector3& a, const Vector3& b) {
    float nA = norm(a), nB = norm(b);
    if (nA < 1e-6f || nB < 1e-6f) return 0.0f;
    float d = dot(a, b) / (nA * nB);
    // Clamp to [-1, 1] to avoid NaN due to floating point error
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    return acos(d);
}

/**
 * Projects vector a onto vector b.
 *
 * @param a The vector to be projected.
 * @param b The vector onto which a is projected.
 * @return The projection vector of a onto b.
 *
 * Note: Returns zero vector if b is near zero length.
 */
inline Vector3 project(const Vector3& a, const Vector3& b) {
    float nB2 = dot(b, b);
    if (nB2 < 1e-6f) return Vector3(0, 0, 0);
    return b * (dot(a, b) / nB2);
}

/**
 * Computes the Euclidean distance between two points represented as vectors.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The distance between points a and b.
 */
inline float distance(const Vector3& a, const Vector3& b) {
    return norm(a - b);
}

/**
 * Performs linear interpolation between two vectors.
 *
 * @param a The start vector.
 * @param b The end vector.
 * @param t The interpolation factor (0.0 returns a, 1.0 returns b).
 * @return The interpolated vector.
 *
 * Usage: Useful for smooth transitions or animations.
 */
inline Vector3 lerp(const Vector3& a, const Vector3& b, float t) {
    return a * (1.0f - t) + b * t;
}

/**
 * Clamps each component of the vector within the specified range.
 *
 * @param v The vector to clamp.
 * @param minVal The minimum allowed value for each component.
 * @param maxVal The maximum allowed value for each component.
 * @return The clamped vector.
 *
 * Usage: Ensures vector components stay within bounds.
 */
inline Vector3 clamp(const Vector3& v, float minVal, float maxVal) {
    return Vector3(
        fmax(fmin(v.x, maxVal), minVal),
        fmax(fmin(v.y, maxVal), minVal),
        fmax(fmin(v.z, maxVal), minVal)
    );
}

/**
 * Checks if a vector is approximately zero within a tolerance.
 *
 * @param v The vector to check.
 * @param eps The tolerance threshold.
 * @return True if all components are within eps of zero, false otherwise.
 *
 * Usage: Useful for avoiding numerical instability or special cases.
 */
inline bool isZero(const Vector3& v, float eps = 1e-6f) {
    return (fabs(v.x) < eps) && (fabs(v.y) < eps) && (fabs(v.z) < eps);
}

/**
 * Checks if two vectors are approximately equal within a tolerance.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @param eps The tolerance threshold.
 * @return True if vectors are approximately equal, false otherwise.
 */
inline bool equals(const Vector3& a, const Vector3& b, float eps = 1e-6f) {
    return isZero(a - b, eps);
}

/**
 * Copies the components of a vector into a float array.
 *
 * @param v The vector to copy from.
 * @param arr The array to copy into (must have size >= 3).
 */
inline void toArray(const Vector3& v, float arr[3]) {
    arr[0] = v.x; arr[1] = v.y; arr[2] = v.z;
}

/**
 * Constructs a vector from a float array.
 *
 * @param arr The array containing at least 3 float elements.
 * @return The constructed vector.
 */
inline Vector3 fromArray(const float arr[3]) {
    return Vector3(arr[0], arr[1], arr[2]);
}


