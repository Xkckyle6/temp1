#pragma once
#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

class Arm
{
public:
    int pin_pul; // example pin for pulse output
    int pin_dir; // example pin for direction control
    int pin_en;  // example pin for enable control
    // Core state (matches your spec)
    float th;      // position [rad]
    // float th_d;    // velocity [rad/s]
    float th_d_cmd;    // velocity [rad/s]
    float th_dd;   // acceleration [rad/s^2]
    float L;          // link length
    float jerk;       // max jerk [rad/s^3]
    uint32_t dt_us; // last dt in microseconds

    uint32_t us_per_Pulse; // time between pulses for the current velocity target, in microseconds
    uint32_t last_pulse_ts; // last time we sent a pulse for this joint, in microseconds
    uint32_t transmission_ratio; // gear ratio
    uint32_t PPR; // pulses per revolution (integer)
    int32_t  pulse_counter; // count of pulses sent, for debugging (signed for direction)
    uint32_t pulse_dt_us;
    uint32_t pulse_end_ts_us; // timestamp when current pulse should be cleared
    uint16_t pulse_width_us; // pulse width in microseconds
    float prev_th_d; // cache previous velocity to avoid repeated math
    const float vel_eps = 1e-3f; // velocity change threshold for recompute

    // Extra control/limit parameters
    float max_vel;    // [rad/s]
    float max_accel;  // [rad/s^2]
    float kp_vel;     // P gain on velocity error

    // Velocity setpoint
    float th_d_target;

    // Constructor with defaults
    Arm(float linkLength  = 1.0f,
        float maxVel      = 10.0f,
        float maxAccel    = 1000.0f,
        float maxJerk     = 5000.0f,
        float kpVel       = 2.0f)
    {
        th    = 0.0;
        th_d_cmd  = 0.0f; us_per_Pulse = 0; last_pulse_ts = 0;
        th_dd = 0.0f;
        L        = linkLength;

        max_vel   = maxVel;
        max_accel = maxAccel;
        jerk      = maxJerk;
        kp_vel    = kpVel;

        th_d_target = 0.0f;
        dt_us       = 0;

        pin_pul = 5; // example pin for pulse output
        pin_dir = 6; // example pin for direction control
        pin_en = 7;  // example pin for enable control
        // pinMode(LED_BUILTIN, OUTPUT);
        pinMode(pin_pul, OUTPUT);
        pinMode(pin_dir, OUTPUT);
        transmission_ratio = 4; // gear ratio
        PPR = 3200u * transmission_ratio; // pulses per revolution (200 steps/rev, 16 microsteps)
        pulse_counter = (int32_t)((th / (2.0f * M_PI)) * (float)PPR); // initial pulse count based on initial position
        pulse_dt_us = 0;
        pulse_end_ts_us = 0;
        pulse_width_us = 5; // default pulse width
        prev_th_d = 0.0f;
    }

    // Set desired joint velocity [rad/s]
    void setVelocityTarget(float v)
    {
        th_d_target = v;
    }

    // Optional: update limits
    void setLimits(float maxVel, float maxAccel, float maxJerk)
    {
        max_vel   = maxVel;
        max_accel = maxAccel;
        jerk      = maxJerk;
    }

    // Call this every loop with elapsed microseconds
    void update(uint32_t dtMicro, uint32_t dtMicros_now)
    {
        // Clear any pending pulse if its width elapsed (non-blocking pulse)
        if (pulse_end_ts_us != 0 && dtMicros_now >= pulse_end_ts_us) {
            digitalWrite(pin_pul, LOW);
            pulse_end_ts_us = 0;
        }

        dt_us = dtMicro;
        if (dt_us == 0) return;

        float dt = dt_us * 1e-6f; // convert to seconds

        // 1) Velocity error
        float err_v = th_d_target - th_d_cmd;

        // 2) Desired acceleration from P on velocity
        float a_cmd = kp_vel * err_v; // [rad/s^2]

        // 3) Jerk limit: limit how fast th_dd moves toward a_cmd
        float a_delta    = a_cmd - th_dd;
        float max_a_step = jerk * dt;  // jerk [rad/s^3] * dt -> max change in accel

        if (a_delta >  max_a_step) a_delta =  max_a_step;
        if (a_delta < -max_a_step) a_delta = -max_a_step;

        th_dd += a_delta;

        // 4) Acceleration limit
        if (th_dd >  max_accel) th_dd =  max_accel;
        if (th_dd < -max_accel) th_dd = -max_accel;

        // 5) Integrate acceleration to velocity
        th_d_cmd += th_dd * dt;

        // update dt for pulses
        if (th_d_cmd != 0.0f) {
            // 6) Velocity limit
            if (th_d_cmd >  max_vel) th_d_cmd =  max_vel;
            if (th_d_cmd < -max_vel) th_d_cmd = -max_vel;
            // Recompute pulse interval only when velocity changes meaningfully
            if (fabsf(th_d_cmd - prev_th_d) > vel_eps) {
                prev_th_d= th_d_cmd;
                float pulses_per_s = th_d_cmd * ((float)PPR * (1.0f / (2.0f * M_PI)));
                if (fabsf(pulses_per_s) > 1e-6f) {
                    us_per_Pulse = (uint32_t)(1000000.0f / fabsf(pulses_per_s));
                } else {
                    us_per_Pulse = 0;
                }
            }

            // if it's time to send a pulse, do so and update the pulse counter
            if (us_per_Pulse != 0 && (dtMicros_now - last_pulse_ts) >= us_per_Pulse) {
                pulse_dt_us = dtMicros_now - last_pulse_ts;
                last_pulse_ts = dtMicros_now;
                // direction and pulse counter update
                if (th_d_cmd < 0) {
                    // digitalWrite(pin_dir, HIGH);
                    pulse_counter++;
                } else {
                    // digitalWrite(pin_dir, LOW);
                    pulse_counter--;
                }
                // ensure pulse counter is within bounds
                int32_t ppr_i = (int32_t)PPR;
                if (pulse_counter < 0) pulse_counter = ppr_i;
                if (pulse_counter > ppr_i) pulse_counter = 0;

                // update th based on pulse count
                th = ((float)pulse_counter / (float)PPR) * (2.0f * M_PI); // convert pulse count to radians

                // Send pulse (non-blocking): set pin high and schedule when to clear it
                digitalWrite(pin_pul, HIGH);
                pulse_end_ts_us = dtMicros_now + pulse_width_us;
            }
        }

    }
};
