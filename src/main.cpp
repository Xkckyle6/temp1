#include <Arduino.h>
#include "joint_controller.h"
#include "motor_controller.h"
#include "kinematics.h"
#include "ee_controller.h"

// Simple official test harness:
// - generates a slow circular EE velocity (vx, vy) in the XY plane
// - computes joint velocities via EE_CONTROLLER
// - sets JOINT_CONTROLLER velocity targets
// - updates MOTOR_CONTROLLER to generate pulses

// Example: three joints (base, shoulder, elbow)
JOINT_CONTROLLER joints[3] = {
    JOINT_CONTROLLER(),
    JOINT_CONTROLLER(),
    JOINT_CONTROLLER()
};

// Motor controllers for each joint. Use distinct pins for each motor in this example.
MOTOR_CONTROLLER motors[3] = {
    MOTOR_CONTROLLER(5, 6, 7, 4),
    MOTOR_CONTROLLER(8, 9, 10, 4),
    MOTOR_CONTROLLER(11, 12, 13, 4)
};

// Kinematics and EE controller (3-DOF)
// Use equal link lengths (units arbitrary) — default 100 as requested
KINEMATICS kin(100.0f, 100.0f);
EE_CONTROLLER ee(kin);
/*
    Arm(float linkLength = 1.0f,
        float maxVel      = 10.0f,
        float maxAccel    = 1000.0f,
        float maxJerk     = 5000.0f,
        float kpVel       = 10.0f)
*/

unsigned long prevMicros = 0;
unsigned long last_print_ms = 0;
float test_phase = 0.0f;

// Desired EE linear velocity amplitude (units per second)
const float EE_VEL_R = 20.0f; // e.g., mm/s if links are mm
const float EE_VEL_OMEGA = 0.5f; // rad/s

void setup()
{
    Serial.begin(115200);
    if(Serial) Serial.println("Starting up...");
    prevMicros = micros();
    //
    // Example: initial velocity targets (zero)
    joints[0].setVelocityTarget(0.0f);
    joints[1].setVelocityTarget(0.0f);
    joints[2].setVelocityTarget(0.0f);
}



void loop()
{
    unsigned long now = micros();
    unsigned long dt_us = now - prevMicros;
    prevMicros = now;

    // Here you would:
    // 1) Compute desired joint velocities from your EE controller / Jacobian
    //    float th0_dot_des = ...;
    //    float th1_dot_des = ...;
    //    float th2_dot_des = ...;
    //
    // 2) Set them as targets:
    //    joints[0].setVelocityTarget(th0_dot_des);
    //    joints[1].setVelocityTarget(th1_dot_des);
    //    joints[2].setVelocityTarget(th2_dot_des);

    // 1) Example EE velocity profile: circular in XY, no Z
    float dt = dt_us * 1e-6f;
    test_phase += EE_VEL_OMEGA * dt;
    // vx = -R * omega * sin(omega*t), vy = R * omega * cos(omega*t)
    float vx = -EE_VEL_R * EE_VEL_OMEGA * sinf(test_phase);
    float vy =  EE_VEL_R * EE_VEL_OMEGA * cosf(test_phase);
    float vz = 0.0f;

    // 2) Compute joint velocities via EE_CONTROLLER
    float q[3];
    for (int i = 0; i < 3; ++i) q[i] = joints[i].th;
    Vec3 v; v.x = vx; v.y = vy; v.z = vz;
    float qdot[3];
    bool ok = ee.computeJointVelocities(q, v, qdot);
    if (ok) {
        // 3) Set joint velocity targets
        for (int i = 0; i < 3; ++i) {
            joints[i].setVelocityTarget(qdot[i]);
        }
    } else {
        // singular or invalid; zero targets
        for (int i = 0; i < 3; ++i) joints[i].setVelocityTarget(0.0f);
    }

    // 4) Update all joint motion controllers and motors
    for (int i = 0; i < 3; ++i)
    {
        joints[i].update(dt_us);
        float vel_cmd = joints[i].getVelocityCommand();
        motors[i].update(now, vel_cmd);
    }
    
    // Debug print every 100 ms
    unsigned long now_ms = millis();
    if (now_ms - last_print_ms >= 100) {
        last_print_ms = now_ms;
        // print joint angles (rad) and commanded velocities
        Serial.print("q = [");
        for (int i = 0; i < 3; ++i) {
            Serial.print(joints[i].th, 4);
            if (i < 2) Serial.print(", ");
        }
        Serial.print("]  qdot_cmd = [");
        for (int i = 0; i < 3; ++i) {
            Serial.print(joints[i].getVelocityCommand(), 4);
            if (i < 2) Serial.print(", ");
        }
        Serial.print("]  EE_v = (");
        Serial.print(vx, 2); Serial.print(", "); Serial.print(vy, 2); Serial.print(")");
        Serial.print("  ok="); Serial.println(ok ? "1" : "0");
    }
    // dt_us
    delayMicroseconds(5);

}
