#pragma once
#include <Arduino.h>
#include "kinematics.h"

class EE_CONTROLLER {
public:
    KINEMATICS kin;

    EE_CONTROLLER(const KINEMATICS &k = KINEMATICS()): kin(k) {}

    // Compute joint velocities qdot (size 3) from desired end-effector velocity v (vx, vy, vz)
    // Uses Jacobian inverse for the 3-DOF configuration described in KINEMATICS.
    bool computeJointVelocities(const float q[3], const Vec3 &v, float qdot_out[3]) const
    {
        float q0 = q[0];
        float q1 = q[1];
        float q2 = q[2];

        float c0 = cosf(q0), s0 = sinf(q0);
        float s1 = sinf(q1), c1 = cosf(q1);
        float s12 = sinf(q1 + q2), c12 = cosf(q1 + q2);

        float r = kin.L1 * c1 + kin.L2 * c12;
        float z_partial = kin.L1 * s1 + kin.L2 * s12; // z

        // Build Jacobian columns
        // dP/dq0
        float J0x = -r * s0;
        float J0y =  r * c0;
        float J0z = 0.0f;

        // dP/dq1
        float dr_dq1 = - (kin.L1 * s1 + kin.L2 * s12); // -z
        float dz_dq1 =   kin.L1 * c1 + kin.L2 * c12;  // r
        float J1x = dr_dq1 * c0;
        float J1y = dr_dq1 * s0;
        float J1z = dz_dq1;

        // dP/dq2
        float dr_dq2 = - kin.L2 * s12;
        float dz_dq2 =   kin.L2 * c12;
        float J2x = dr_dq2 * c0;
        float J2y = dr_dq2 * s0;
        float J2z = dz_dq2;

        // Construct 3x3 matrix J = [J0 J1 J2]
        float J[3][3] = {
            { J0x, J1x, J2x },
            { J0y, J1y, J2y },
            { J0z, J1z, J2z }
        };

        // Compute inverse of 3x3 (use adjugate/determinant). Very small det -> singular.
        // Compute determinant
        float det = J[0][0]*(J[1][1]*J[2][2] - J[1][2]*J[2][1])
                  - J[0][1]*(J[1][0]*J[2][2] - J[1][2]*J[2][0])
                  + J[0][2]*(J[1][0]*J[2][1] - J[1][1]*J[2][0]);

        if (fabsf(det) < 1e-8f) return false;

        // Compute inverse = adj(J)/det
        float inv[3][3];
        inv[0][0] =  (J[1][1]*J[2][2] - J[1][2]*J[2][1]) / det;
        inv[0][1] = -(J[0][1]*J[2][2] - J[0][2]*J[2][1]) / det;
        inv[0][2] =  (J[0][1]*J[1][2] - J[0][2]*J[1][1]) / det;

        inv[1][0] = -(J[1][0]*J[2][2] - J[1][2]*J[2][0]) / det;
        inv[1][1] =  (J[0][0]*J[2][2] - J[0][2]*J[2][0]) / det;
        inv[1][2] = -(J[0][0]*J[1][2] - J[0][2]*J[1][0]) / det;

        inv[2][0] =  (J[1][0]*J[2][1] - J[1][1]*J[2][0]) / det;
        inv[2][1] = -(J[0][0]*J[2][1] - J[0][1]*J[2][0]) / det;
        inv[2][2] =  (J[0][0]*J[1][1] - J[0][1]*J[1][0]) / det;

        // multiply inv * v
        qdot_out[0] = inv[0][0]*v.x + inv[0][1]*v.y + inv[0][2]*v.z;
        qdot_out[1] = inv[1][0]*v.x + inv[1][1]*v.y + inv[1][2]*v.z;
        qdot_out[2] = inv[2][0]*v.x + inv[2][1]*v.y + inv[2][2]*v.z;

        return true;
    }
};
