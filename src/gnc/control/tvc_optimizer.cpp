// tvc_optimizer.cpp
#include "gnc/control/tvc_optimizer.h"
#include "utils/math/defines.h"
#include <Arduino.h> // For Serial output if using on Teensy/Arduino
#include <Servo.h>   // Ensure Servo library is available

#include <cmath>   // for std::sqrt, sinf, cosf, sqrtf



// Forward declaration for tvcCostImpl (used by optimizer)
float tvcCost(const float* x, int n, const void* ctx);
float tvcCostImpl(const float* x, int n, const TVCState* tvc);

// Wrapper to match AdamOptimizer::CostFunction signature
float tvcCostWrapper(const float* x, int n, const void* ctx) {
    // Cast context pointer back to TVCState*
    return tvcCostImpl(x, n, static_cast<const TVCState*>(ctx));
}

// TVC circle configuration constants
static const float TVC_CIRCLE_RADIUS = 0.09f;
static const float TVC_CIRCLE_Z_POS = -1.0f;
static const float TVC_CIRCLE_THRUST[3] = { 3.0f, 3.0f, 3.0f };  // Example independent thrusts

// Motor angles around the circle in radians (0°, 120°, 240°)
static const float TVC_CIRCLE_ANGLES[3] = {
    240.0f * PI / 180.0f,
    120.0f * PI / 180.0f,
    0.0f * PI / 180.0f
};

// Macro to list TVC motor pointers given a TVCState reference
#define TVC_MOTORS_ARRAY(tvc_ref) { &tvc_ref.M1, &tvc_ref.M2, &tvc_ref.M3 }

/**
 * @brief Initialize TVC motors in a circular configuration around the vehicle's center.
 * @param tvc Reference to the TVCState struct to configure motor positions and thrust.
 */
void initializeTVCCircle(TVCState& tvc) {
    // Use file-level constants and macro for configuration
    TVCMotor* motors[3] = TVC_MOTORS_ARRAY(tvc);

    for (int i = 0; i < 3; ++i) {
        float theta = TVC_CIRCLE_ANGLES[i];
        motors[i]->position[0] = TVC_CIRCLE_RADIUS * cosf(theta);  // x
        motors[i]->position[1] = TVC_CIRCLE_RADIUS * sinf(theta);  // y
        motors[i]->position[2] = TVC_CIRCLE_Z_POS;                 // z (below CoM)
        motors[i]->roll = theta + PI + (45.0f * PI / 180.0f);  // rolled outward + 45° structural offset
        motors[i]->thrust = TVC_CIRCLE_THRUST[i];  // Replace constant with array of per-motor thrusts
    }
}

#ifndef S_microseconds
#define S_microseconds 0
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795f
#endif

#ifndef M_gear_coef
#define M_gear_coef 1.0f
#endif

#ifndef S1_zero
#define S1_zero 90.0f
#endif
#ifndef S2_zero
#define S2_zero 90.0f
#endif
#ifndef S3_zero
#define S3_zero 90.0f
#endif
#ifndef S4_zero
#define S4_zero 90.0f
#endif
#ifndef S5_zero
#define S5_zero 90.0f
#endif
#ifndef S6_zero
#define S6_zero 90.0f
#endif

#ifndef S1_pin
#define S1_pin 6
#endif
#ifndef S2_pin
#define S2_pin 7
#endif
#ifndef S3_pin
#define S3_pin 5
#endif
#ifndef S4_pin
#define S4_pin 4
#endif
#ifndef S5_pin
#define S5_pin 3
#endif
#ifndef S6_pin
#define S6_pin 2
#endif

Servo S1, S2, S3, S4, S5, S6;
float servoCmd[6] = {0};

// Global TVC state
extern TVCState tvc;

void configTVC(float tx, float ty, float tz, const float thrusts[3]) {
  tvc.TVC_target[0] = tx;
  tvc.TVC_target[1] = ty;
  tvc.TVC_target[2] = tz;
  // Ensure TVC_target[3] is preserved for thrust target (if needed)
  // For example, set it to sum of thrusts or a desired thrust target
  tvc.TVC_target[3] = thrusts[0] + thrusts[1] + thrusts[2];
  tvc.M1.thrust = thrusts[0];
  tvc.M2.thrust = thrusts[1];
  tvc.M3.thrust = thrusts[2];
}

void TVC_servo_to_vec(float S1, float S2, float S3, float S4, float S5, float S6) {
  // Placeholder: convert servo angles (pitch, yaw) to thrust vectors (update M1_vec, etc.)
}

float tvcCostImpl(const float* x, int n, const TVCState* tvc) {
  float torqueAccum[3] = {0.0f, 0.0f, 0.0f};
  float forceAccum[3] = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < 3; ++i) {
    const TVCMotor* motor = (i == 0) ? &tvc->M1 : (i == 1) ? &tvc->M2 : &tvc->M3;
    float pitch = x[i * 2 + 0];
    float yaw   = x[i * 2 + 1];

    float thrust_x = -motor->thrust * sinf(yaw);
    float thrust_y = -motor->thrust * sinf(pitch);
    float thrust_z = -motor->thrust * cosf(yaw) * cosf(pitch);

    float pos_x = motor->position[0];
    float pos_y = motor->position[1];
    float pos_z = motor->position[2];

    torqueAccum[0] += vecCross1(pos_x, pos_y, pos_z, thrust_x, thrust_y, thrust_z);
    torqueAccum[1] += vecCross2(pos_x, pos_y, pos_z, thrust_x, thrust_y, thrust_z);
    torqueAccum[2] += vecCross3(pos_x, pos_y, pos_z, thrust_x, thrust_y, thrust_z);

    forceAccum[0] += thrust_x;
    forceAccum[1] += thrust_y;
    forceAccum[2] += thrust_z;
  }

  // Use 3D torque target vector [roll, pitch, yaw]
  float torqueError = vecLength(
    tvc->TVC_target[0] - torqueAccum[0],
    tvc->TVC_target[1] - torqueAccum[1],
    tvc->TVC_target[2] - torqueAccum[2]
  );

  float upwardThrust = forceAccum[2];
  float netThrust = vecLength(forceAccum[0], forceAccum[1], forceAccum[2]);
  float thrustError = fabsf(netThrust - tvc->TVC_target[3]);

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

// Call this in setup() to attach the servos to their pins
void setupServos() {
  S1.attach(S1_pin);
  S2.attach(S2_pin);
  S3.attach(S3_pin);
  S4.attach(S4_pin);
  S5.attach(S5_pin);
  S6.attach(S6_pin);
}

// Send servo angles (in radians) to the physical servos, with clamping and transformation
void TVC_out(float* servo_cmd_rad) {
  const float angle_limit = SERVO_ANGLE_LIMIT;

  // Clamp each servo's pitch/yaw to circular cone limit
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