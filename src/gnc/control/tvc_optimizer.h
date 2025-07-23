#ifndef TVC_OPTIMIZER_H
#define TVC_OPTIMIZER_H

#include <math.h>
#include "utils/math/adam_optimizer.h"
#include "utils/math/math_utils.h"   // <-- Include your Vector3, etc.

/* AdamOptimizer::CostFunction signature */
float tvcCostWrapper(const float* x, int n, const void* ctx);

#define SERVO_ANGLE_LIMIT  (0.2618f * 2.0f)  // ~30 degrees in radians
#define SERVO_DT           0.02f             // 50 Hz loop rate

extern float targetWrenchInput[4];
extern float targetThrusts[3];
void configTVC(float tx, float ty, float tz, const float thrusts[3]);

struct TVCMotor {
    float roll;         // radians
    float thrust;       // newtons
    Vector3 position;   // meters, use new Vector3 struct
};

struct TVCState {
    TVCMotor M1, M2, M3;
    Vector3 TVC_target;   // Target torque: [tx, ty, tz]
    float TVC_target_mag; // Target total thrust
};

/* Clamp value between min and max */
inline float clamp(float x, float minVal, float maxVal) {
    return fmaxf(minVal, fminf(maxVal, x));
}

/* Clamp servo pitch/yaw to lie within a circular cone limit */
inline void clampServoCircle(float& pitch, float& yaw, float maxAngleRad) {
    float mag = sqrtf(pitch * pitch + yaw * yaw);
    if (mag > maxAngleRad) {
        float scale = maxAngleRad / mag;
        pitch *= scale;
        yaw   *= scale;
    }
}

/* Convert servo angles to thrust vector directions (implementation needed) */
void TVC_servo_to_vec(float S1, float S2, float S3, float S4, float S5, float S6);

/* Cost function to evaluate torque error */
float tvcCost(const float* x, int n, const void* ctx);

/* Run optimization using Adam to minimize torque error */
void optimizeTVC(TVCState& tvc, float* x, AdamOptimizer& optimizer, int steps = 100, bool debug = true);

void setupServos();
void TVC_out(float* servo_cmd_rad);

/**
 * @brief Initialize TVC motors in a circular configuration around the vehicle's center.
 * @param tvc Reference to the TVCState struct to configure motor positions and thrust.
 */
void initializeTVCCircle(TVCState& tvc);

#endif