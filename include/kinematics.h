#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <math.h>

// KINEMATICS for a 3-DOF arm: base yaw (q0), shoulder (q1) and elbow (q2).
// The shoulder and elbow form a planar 2-link manipulator; the base rotates
// that plane about the z axis. Link lengths default to 100 (units arbitrary).

struct Vec2 { float x; float y; };
struct Vec3 { float x; float y; float z; };

class KINEMATICS {
public:
    float L1, L2; // planar link lengths (shoulder, elbow)

    KINEMATICS(float l1 = 100.0f, float l2 = 100.0f) {
        L1 = l1; L2 = l2;
    }

    // Forward kinematics for 3-DOF: q = [q0, q1, q2]
    // q0: base yaw, q1: shoulder elevation, q2: elbow relative
    Vec3 forwardFK(const float q[3]) const {
        float q0 = q[0];
        float q1 = q[1];
        float q2 = q[2];
        float r = L1 * cosf(q1) + L2 * cosf(q1 + q2);
        float z = L1 * sinf(q1) + L2 * sinf(q1 + q2);
        Vec3 p;
        p.x = r * cosf(q0);
        p.y = r * sinf(q0);
        p.z = z;
        return p;
    }

    // Inverse kinematics: given EE position (x,y,z), compute q[3]. Returns true on success.
    // q0 is determined from atan2(y,x). The planar inverse on (r,z) gives q1 and q2.
    bool inverseIK(const Vec3 &p, float q_out[3]) const {
        float x = p.x; float y = p.y; float z = p.z;
        float r = sqrtf(x*x + y*y);
        // planar 2-link inverse for r,z
        float r2 = r*r + z*z;
        float c2 = (r2 - L1*L1 - L2*L2) / (2.0f * L1 * L2);
        if (c2 < -1.0f || c2 > 1.0f) return false;
        float s2 = sqrtf(max(0.0f, 1.0f - c2*c2));
        float q2 = atan2f(s2, c2);
        float k1 = L1 + L2 * c2;
        float k2 = L2 * s2;
        float q1 = atan2f(z, r) - atan2f(k2, k1);
        float q0 = atan2f(y, x);
        q_out[0] = q0; q_out[1] = q1; q_out[2] = q2;
        return true;
    }
};
