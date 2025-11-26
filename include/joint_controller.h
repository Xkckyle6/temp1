#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

class JOINT_CONTROLLER {
public:
    // State
    float th;            // position [rad] (estimated)
    float th_d_cmd;      // commanded velocity after limiting [rad/s]
    float th_dd;         // commanded acceleration [rad/s^2]

    // Setpoint
    float th_d_target;   // desired velocity [rad/s]

    // Limits
    float max_vel;       // [rad/s]
    float max_accel;     // [rad/s^2]
    float jerk;          // [rad/s^3]
    float kp_vel;        // simple P on velocity

    // internals
    float prev_th_d;
    const float vel_eps = 1e-4f;

    JOINT_CONTROLLER(float linkLength = 1.0f,
                     float maxVel = 10.0f,
                     float maxAccel = 1000.0f,
                     float maxJerk = 5000.0f,
                     float kp = 2.0f)
    {
        th = 0.0f;
        th_d_cmd = 0.0f;
        th_dd = 0.0f;
        th_d_target = 0.0f;
        max_vel = maxVel;
        max_accel = maxAccel;
        jerk = maxJerk;
        kp_vel = kp;
        prev_th_d = 0.0f;
    }

    void setVelocityTarget(float v) { th_d_target = v; }

    void setLimits(float v, float a, float j) { max_vel = v; max_accel = a; jerk = j; }

    // Call this with dt in microseconds. Produces limited th_d_cmd.
    void update(uint32_t dt_us)
    {
        if (dt_us == 0) return;
        float dt = dt_us * 1e-6f;

        float err_v = th_d_target - th_d_cmd;
        float a_cmd = kp_vel * err_v;

        float a_delta = a_cmd - th_dd;
        float max_a_step = fabsf(jerk) * dt;
        if (a_delta > max_a_step) a_delta = max_a_step;
        if (a_delta < -max_a_step) a_delta = -max_a_step;
        th_dd += a_delta;

        // accel limits
        if (th_dd > max_accel) th_dd = max_accel;
        if (th_dd < -max_accel) th_dd = -max_accel;

        // integrate
        th_d_cmd += th_dd * dt;

        // vel limit
        if (th_d_cmd > max_vel) th_d_cmd = max_vel;
        if (th_d_cmd < -max_vel) th_d_cmd = -max_vel;

        // optionally update estimated position
        th += th_d_cmd * dt;
    }

    float getVelocityCommand() const { return th_d_cmd; }
};
