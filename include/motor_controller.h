#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

class MOTOR_CONTROLLER {
public:
    int pin_pul;
    int pin_dir;
    int pin_en;

    uint32_t transmission_ratio;
    uint32_t PPR; // pulses per revolution
    int32_t pulse_counter;

    uint16_t pulse_width_us;
    uint32_t last_pulse_ts;
    uint32_t pulse_end_ts_us;

    float th; // estimated position [rad]

    MOTOR_CONTROLLER(int pul = 5, int dir = 6, int en = 7, uint32_t trans = 4, uint32_t basePPR = 3200u)
    {
        pin_pul = pul; pin_dir = dir; pin_en = en;
        transmission_ratio = trans;
        PPR = basePPR * transmission_ratio;
        pulse_counter = 0;
        pulse_width_us = 5;
        last_pulse_ts = 0;
        pulse_end_ts_us = 0;
        th = 0.0f;
        pinMode(pin_pul, OUTPUT);
        pinMode(pin_dir, OUTPUT);
        pinMode(pin_en, OUTPUT);
        digitalWrite(pin_pul, LOW);
        digitalWrite(pin_en, LOW);
    }

    // Non-blocking: call each loop with current micros and desired joint velocity (rad/s)
    void update(uint32_t now_us, float desired_vel_rad_s)
    {
        // clear pulse if its width elapsed
        if (pulse_end_ts_us != 0 && now_us >= pulse_end_ts_us) {
            digitalWrite(pin_pul, LOW);
            pulse_end_ts_us = 0;
        }

        // compute pulses per second
        float pulses_per_s = desired_vel_rad_s * ((float)PPR / (2.0f * M_PI));
        if (fabsf(pulses_per_s) < 1e-6f) return; // no motion

        uint32_t us_per_pulse = (uint32_t)(1000000.0f / fabsf(pulses_per_s));
        if (us_per_pulse == 0) return;

        if (last_pulse_ts == 0) last_pulse_ts = now_us;
        if ((now_us - last_pulse_ts) >= us_per_pulse) {
            last_pulse_ts = now_us;
            // set direction
            if (desired_vel_rad_s < 0) digitalWrite(pin_dir, HIGH);
            else digitalWrite(pin_dir, LOW);

            // toggle pulse high and schedule clear
            digitalWrite(pin_pul, HIGH);
            pulse_end_ts_us = now_us + pulse_width_us;

            // update pulse counter and estimated angle
            if (desired_vel_rad_s < 0) pulse_counter++;
            else pulse_counter--;
            int32_t ppr_i = (int32_t)PPR;
            if (pulse_counter < 0) pulse_counter = ppr_i;
            if (pulse_counter > ppr_i) pulse_counter = 0;
            th = ((float)pulse_counter / (float)PPR) * (2.0f * M_PI);
        }
    }
};
