#include <Arduino.h>
#include "utils/math/math_utils.h"

void printVector(const char* name, const Vector3& v) {
    Serial.print(name);
    Serial.print(": (");
    Serial.print(v.x, 4); Serial.print(", ");
    Serial.print(v.y, 4); Serial.print(", ");
    Serial.print(v.z, 4); Serial.println(")");
}

void printQuaternion(const char* name, const Quaternion& q) {
    Serial.print(name);
    Serial.print(": (");
    Serial.print(q.w, 4); Serial.print(", ");
    Serial.print(q.x, 4); Serial.print(", ");
    Serial.print(q.y, 4); Serial.print(", ");
    Serial.print(q.z, 4); Serial.println(")");
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("==== VECTOR3 TESTS ====");

    Vector3 a(1, 2, 3);
    Vector3 b(-4, 5, -6);

    printVector("a", a);
    printVector("b", b);

    printVector("a+b", a + b);
    printVector("a-b", a - b);
    printVector("a*2", a * 2);
    printVector("2*b", 2 * b);
    printVector("a/2", a / 2);

    Serial.print("dot(a, b): "); Serial.println(dot(a, b), 4);
    printVector("cross(a, b)", cross(a, b));

    Serial.print("norm(a): "); Serial.println(norm(a), 4);
    printVector("normalize(a)", normalize(a));
    printVector("elemMul(a, b)", elemMul(a, b));
    printVector("elemDiv(a, b)", elemDiv(a, b));
    Serial.print("angleBetween(a, b): "); Serial.println(angleBetween(a, b), 4);

    printVector("project(a, b)", project(a, b));
    Serial.print("distance(a, b): "); Serial.println(distance(a, b), 4);
    printVector("lerp(a, b, 0.5)", lerp(a, b, 0.5f));
    printVector("clamp(a, -1, 1)", clamp(a, -1, 1));
    Serial.print("isZero(a): "); Serial.println(isZero(a) ? "true" : "false");
    Serial.print("equals(a, a): "); Serial.println(equals(a, a) ? "true" : "false");

    float arr[3];
    toArray(a, arr);
    Vector3 c = fromArray(arr);
    printVector("fromArray(toArray(a))", c);

    Serial.print("a.isFinite(): "); Serial.println(a.isFinite() ? "true" : "false");
    Serial.print("a.isNormalized(): "); Serial.println(a.isNormalized() ? "true" : "false");
    Serial.println();

    Serial.println("==== QUATERNION TESTS ====");

    Quaternion q1(0.7071f, 0.0f, 0.7071f, 0.0f); // 90 deg around Y
    Quaternion q2(0.7071f, 0.7071f, 0.0f, 0.0f); // 90 deg around X
    printQuaternion("q1", q1);
    printQuaternion("q2", q2);

    printQuaternion("q1 * q2", Quaternion::multiply(q1, q2));
    printQuaternion("q1.conjugate()", q1.conjugate());
    printQuaternion("q1.inverse()", q1.inverse());
    Serial.print("q1.norm(): "); Serial.println(q1.norm(), 4);
    Serial.print("Quaternion::dot(q1, q2): "); Serial.println(Quaternion::dot(q1, q2), 4);
    Serial.print("q1.isIdentity(): "); Serial.println(q1.isIdentity() ? "true" : "false");
    Serial.print("q1.isFinite(): "); Serial.println(q1.isFinite() ? "true" : "false");
    Serial.print("q1.isNormalized(): "); Serial.println(q1.isNormalized() ? "true" : "false");

    // Euler conversion (ZYX)
    float roll, pitch, yaw;
    Quaternion q_euler = Quaternion::fromEuler(0.1f, 0.2f, 0.3f, EulerOrder::ZYX);
    printQuaternion("fromEuler(0.1,0.2,0.3,ZYX)", q_euler);
    q_euler.toEuler(roll, pitch, yaw, EulerOrder::ZYX);
    Serial.print("toEuler (ZYX): roll="); Serial.print(roll, 4); Serial.print(" pitch="); Serial.print(pitch, 4); Serial.print(" yaw="); Serial.println(yaw, 4);

    // SLERP
    Quaternion slerped = Quaternion::slerp(q1, q2, 0.5f);
    printQuaternion("slerp(q1, q2, 0.5)", slerped);

    // Rotate vector
    Vector3 v(1, 0, 0);
    Vector3 v_rot = q1.rotate(v);
    printVector("q1.rotate((1,0,0))", v_rot);

    // Axis-angle
    Vector3 axis;
    float angle;
    q1.toAxisAngle(axis, angle);
    printVector("q1 axis", axis);
    Serial.print("q1 angle (rad): "); Serial.println(angle, 4);

    // Rotation matrix
    float R[3][3];
    q1.toRotationMatrix(R);
    Serial.println("q1 rotation matrix:");
    for (int i = 0; i < 3; ++i) {
        Serial.print("[");
        for (int j = 0; j < 3; ++j) {
            Serial.print(R[i][j], 4);
            if (j < 2) Serial.print(", ");
        }
        Serial.println("]");
    }
}

void loop() {
    // nothing
}