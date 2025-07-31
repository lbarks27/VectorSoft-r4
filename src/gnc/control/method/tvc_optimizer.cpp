// tvc_optimizer.cpp
#include "gnc/control/method/tvc_optimizer.h"
#include "utils/math/math_utils.h"   // <-- Use new math utilities
#include <Arduino.h>
#include <Servo.h>
#include <cmath>

// Forward declaration for tvcCostImpl (used by optimizer)
float tvcCost(const float* x, int n, const void* ctx);
float tvcCostImpl(const float* x, int n, const TVCState* tvc);

float tvcCostWrapper(const float* x, int n, const void* ctx) {
    return tvcCostImpl(x, n, static_cast<const TVCState*>(ctx));
}

// TVC circle configuration constants
static const float TVC_CIRCLE_RADIUS = 0.09f;
static const float TVC_CIRCLE_Z_POS = -1.0f;
static const float TVC_CIRCLE_THRUST[3] = { 3.0f, 3.0f, 3.0f };  // Example

static const float TVC_CIRCLE_ANGLES[3] = {
    240.0f * PI / 180.0f,
    120.0f * PI / 180.0f,
    0.0f * PI / 180.0f
};

#define TVC_MOTORS_ARRAY(tvc_ref) { &tvc_ref.M1, &tvc_ref.M2, &tvc_ref.M3 }

/**
 * @brief Initialize TVC motors in a circular configuration around the vehicle's center.
 */
void initializeTVCCircle(TVCState& tvc) {
    TVCMotor* motors[3] = TVC_MOTORS_ARRAY(tvc);
    for (int i = 0; i < 3; ++i) {
        float theta = TVC_CIRCLE_ANGLES[i];
        motors[i]->position = Vector3(
            TVC_CIRCLE_RADIUS * cosf(theta),    // x
            TVC_CIRCLE_RADIUS * sinf(theta),    // y
            TVC_CIRCLE_Z_POS                   // z (below CoM)
        );
        motors[i]->roll = theta + PI + (45.0f * PI / 180.0f);
        motors[i]->thrust = TVC_CIRCLE_THRUST[i];
    }
}

// (Servo macros and variables remain unchanged)

Servo S1, S2, S3, S4, S5, S6;
float servoCmd[6] = {0};

extern TVCState tvc;

void configTVC(float tx, float ty, float tz, const float thrusts[3]) {
    tvc.TVC_target = Vector3(tx, ty, tz);
    tvc.TVC_target_mag = thrusts[0] + thrusts[1] + thrusts[2];
    tvc.M1.thrust = thrusts[0];
    tvc.M2.thrust = thrusts[1];
    tvc.M3.thrust = thrusts[2];
}

void TVC_servo_to_vec(float S1, float S2, float S3, float S4, float S5, float S6) {
    // Optional: For future implementation, can use Vector3 for thrust vector per motor
}

float tvcCostImpl(const float* x, int n, const TVCState* tvc) {
    Vector3 torqueAccum(0.0f, 0.0f, 0.0f);
    Vector3 forceAccum(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 3; ++i) {
        const TVCMotor* motor = (i == 0) ? &tvc->M1 : (i == 1) ? &tvc->M2 : &tvc->M3;
        float pitch = x[i * 2 + 0];
        float yaw   = x[i * 2 + 1];

        // Calculate thrust vector for this motor
        Vector3 thrust(
            -motor->thrust * sinf(yaw),
            -motor->thrust * sinf(pitch),
            -motor->thrust * cosf(yaw) * cosf(pitch)
        );

        // Cross product: r x F for torque
        Vector3 torque = Vector3::cross(motor->position, thrust);
        torqueAccum = torqueAccum + torque;
        forceAccum = forceAccum + thrust;
    }

    // Use 3D torque target vector [roll, pitch, yaw]
    float torqueError = (tvc->TVC_target - torqueAccum).norm();

    float upwardThrust = forceAccum.z;
    float netThrust = forceAccum.norm();
    float thrustError = fabsf(netThrust - tvc->TVC_target_mag);

    float k1 = 0.1f; // upward thrust weight
    float k2 = 0.05f; // thrust match weight

    return torqueError + k1 * upwardThrust - k2 * thrustError;
}

void optimizeTVC(TVCState& tvc, float* x, AdamOptimizer& optimizer, int steps, bool debug) {
    for (int step = 0; step < steps; ++step) {
        optimizer.step(tvcCostWrapper, x, &tvc);

        // Clamp each servo's pitch/yaw to circular cone limit
        for (int i = 0; i < 3; ++i) {
            clampServoCircle(x[i * 2 + 0], x[i * 2 + 1], SERVO_ANGLE_LIMIT);
        }

        if (debug) {
            Serial.print("Step "); Serial.print(step);
            Serial.print(" | Cost: "); Serial.print(tvcCostImpl(x, 6, &tvc), 6);
            Serial.print(" | Angles: ");
            for (int i = 0; i < 6; ++i) {
                Serial.print(x[i], 4);
                Serial.print(i < 5 ? ", " : "\n");
            }
        }
    }

    // Output commands to servos directly
    servoCmd[0] = x[0] * RAD_TO_DEG * M_gear_coef + S1_zero;
    servoCmd[1] = x[1] * RAD_TO_DEG * M_gear_coef + S2_zero;
    servoCmd[2] = x[2] * RAD_TO_DEG * M_gear_coef + S3_zero;
    servoCmd[3] = x[3] * RAD_TO_DEG * M_gear_coef + S4_zero;
    servoCmd[4] = x[4] * RAD_TO_DEG * M_gear_coef + S5_zero;
    servoCmd[5] = x[5] * RAD_TO_DEG * M_gear_coef + S6_zero;

    if (!S_microseconds) {
        S1.write(servoCmd[0]);
        S2.write(servoCmd[1]);
        S3.write(servoCmd[2]);
        S4.write(servoCmd[3]);
        S5.write(servoCmd[4]);
        S6.write(servoCmd[5]);
    } else {
        // Optionally support microsecond PWM here
    }
}

void setupServos() {
    S1.attach(S1_pin);
    S2.attach(S2_pin);
    S3.attach(S3_pin);
    S4.attach(S4_pin);
    S5.attach(S5_pin);
    S6.attach(S6_pin);
}

void TVC_out(float* servo_cmd_rad) {
    const float angle_limit = SERVO_ANGLE_LIMIT;

    for (int i = 0; i < 3; ++i) {
        clampServoCircle(servo_cmd_rad[i * 2 + 0], servo_cmd_rad[i * 2 + 1], angle_limit);
    }

    float command[6];
    command[0] = servo_cmd_rad[0] * RAD_TO_DEG * M_gear_coef + S1_zero;
    command[1] = servo_cmd_rad[1] * RAD_TO_DEG * M_gear_coef + S2_zero;
    command[2] = servo_cmd_rad[2] * RAD_TO_DEG * M_gear_coef + S3_zero;
    command[3] = servo_cmd_rad[3] * RAD_TO_DEG * M_gear_coef + S4_zero;
    command[4] = servo_cmd_rad[4] * RAD_TO_DEG * M_gear_coef + S5_zero;
    command[5] = servo_cmd_rad[5] * RAD_TO_DEG * M_gear_coef + S6_zero;

    if (!S_microseconds) {
        S1.write(command[0]);
        S2.write(command[1]);
        S3.write(command[2]);
        S4.write(command[3]);
        S5.write(command[4]);
        S6.write(command[5]);
    } else {
        // Optionally support microsecond PWM here
    }
}

float tvcCost(const float* x, int n, const void* ctx) {
    return tvcCostImpl(x, n, static_cast<const TVCState*>(ctx));
}