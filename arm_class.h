#pragma once
#include <Arduino.h>
#include <math.h>

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
    float theta;      // position [rad]
    float theta_d;    // velocity [rad/s]
    float theta_dd;   // acceleration [rad/s^2]
    float L;          // link length
    float jerk;       // max jerk [rad/s^3]
    unsigned long dt_us; // last dt in microseconds

    unsigned long us_per_Pulse; // time between pulses for the current velocity target, in microseconds
    unsigned long last_pulse_ts; // last time we sent a pulse for this joint, in microseconds
    unsigned long transmission_ratio; // gear ratio
    float PPR; // pulses per revolution
    long pulse_counter; // count of pulses sent, for debugging
    long pulse_dt_us;

    // Extra control/limit parameters
    float max_vel;    // [rad/s]
    float max_accel;  // [rad/s^2]
    float kp_vel;     // P gain on velocity error

    // Velocity setpoint
    float theta_d_des;

    // Constructor with defaults
    Arm(float linkLength  = 1.0f,
        float maxVel      = 10.0f,
        float maxAccel    = 1000.0f,
        float maxJerk     = 5000.0f,
        float kpVel       = 10.0f)
    {
        theta    = 0.0;
        theta_d  = 0.0f; us_per_Pulse = 0.0f; last_pulse_ts = 0.0f;
        theta_dd = 0.0f;
        L        = linkLength;

        max_vel   = maxVel;
        max_accel = maxAccel;
        jerk      = maxJerk;
        kp_vel    = kpVel;

        theta_d_des = 0.0f;
        dt_us       = 0.0;

        pin_pul = 5; // example pin for pulse output
        pin_dir = 6; // example pin for direction control
        pin_en = 7;  // example pin for enable control
        // pinMode(LED_BUILTIN, OUTPUT);
        pinMode(pin_pul, OUTPUT);
        pinMode(pin_dir, OUTPUT);
        transmission_ratio = 4; // gear ratio
        PPR = 3200 * transmission_ratio; // pulses per revolution (200 steps/rev, 16 microsteps)
        pulse_counter = (theta/(2.0f * (3.14159))) * PPR; // initial pulse count based on initial position
        pulse_dt_us = 0;
        // pulse_counter = (theta/(2.0f * (M_PI))) * PPR; // initial pulse count based on initial position
    }

    // Set desired joint velocity [rad/s]
    void setVelocityTarget(float theta_d_target)
    {
        theta_d_des = theta_d_target;
    }

    // Optional: update limits
    void setLimits(float maxVel, float maxAccel, float maxJerk)
    {
        max_vel   = maxVel;
        max_accel = maxAccel;
        jerk      = maxJerk;
    }

    // Call this every loop with elapsed microseconds
    void update(unsigned long dtMicro, unsigned long dtMicros_now)
    {
        // now = dtMicros_now;
        dt_us = dtMicro;
        if (dt_us == 0 ) return;

        float dt = dt_us / 1e6f;  // convert to seconds

        // 1) Velocity error
        float err_v = theta_d_des - theta_d;

        // 2) Desired acceleration from P on velocity
        float a_cmd = kp_vel * err_v; // [rad/s^2]

        // 3) Jerk limit: limit how fast theta_dd moves toward a_cmd
        float a_delta    = a_cmd - theta_dd;
        float max_a_step = jerk * dt;  // jerk [rad/s^3] * dt -> max change in accel

        if (a_delta >  max_a_step) a_delta =  max_a_step;
        if (a_delta < -max_a_step) a_delta = -max_a_step;

        theta_dd += a_delta;

        // 4) Acceleration limit
        if (theta_dd >  max_accel) theta_dd =  max_accel;
        if (theta_dd < -max_accel) theta_dd = -max_accel;

        // 5) Integrate acceleration to velocity
        theta_d += theta_dd * dt;

        // update dt for pulses
        if(theta_d != 0.0f){
            // 6) Velocity limit
            if (theta_d >  max_vel) theta_d =  max_vel;
            if (theta_d < -max_vel) theta_d = -max_vel;
            //
            us_per_Pulse = 1.0f / ((theta_d/(2.0f*3.14159f)) * PPR / 1000000.0f);

            // if it's time to send a pulse, do so and update the pulse counter
            if((dtMicros_now - last_pulse_ts) >= us_per_Pulse){
                pulse_dt_us = dtMicros_now - last_pulse_ts;
                last_pulse_ts = dtMicros_now;
                // direction and pulse counter update
                if(theta_d < 0){
                    // digitalWrite(pin_dir, HIGH);
                    pulse_counter++;
                } else {
                    // digitalWrite(pin_dir, LOW);
                    pulse_counter--;
                }
                // ensure pulse counter is within bounds
                if(pulse_counter < 0) pulse_counter = PPR;
                if(pulse_counter > PPR) pulse_counter = 0;

                // update theta based on pulse count
                theta = ((pulse_counter) / (PPR)) * (2.0f * 3.14159f); // convert pulse count to radians

                // Send pulse
                digitalWrite(pin_pul, HIGH);
                delayMicroseconds(5);
                digitalWrite(pin_pul, LOW);
            }
        }

    }
};
