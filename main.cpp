#include <Arduino.h>
#include "arm_class.h"




// unsigned long transmission_ratio = 4; // gear ratio
// unsigned long ppr = 200*16*transmission_ratio; // pulses per revolution
// // unsigned long rpm_target = 5; // revolutions per minute
// // unsigned long expected_pulse_dt_us = 1000000*(60.0/(ppr*rpm_target)); // time between pulses in us

// unsigned long pulse_dt_from_RPM(float thd) {
//   return 1000000*(60.0/(ppr*thd));
// } 




// Example: three joints (base, shoulder, elbow)
Arm joints[3] = {
    Arm(),  // joint 0
    Arm(),  // joint 1
    Arm()   // joint 2
};
/*
    Arm(float linkLength = 1.0f,
        float maxVel      = 10.0f,
        float maxAccel    = 1000.0f,
        float maxJerk     = 5000.0f,
        float kpVel       = 10.0f)
*/

unsigned long prevMicros = 0;

void setup()
{
    Serial.begin(115200);
    if(Serial) Serial.println("Starting up...");
    prevMicros = micros();
    //

    // Example: initial velocity targets
    joints[0].setVelocityTarget(0.5); // rad/s
    joints[1].setVelocityTarget(0.0f);
    joints[2].setVelocityTarget(0.0f);
}



void loop()
{
//     for(int i = 0; i < 3200*4; ++i)
//     {
//         // Example: update velocity targets based on some control logic
//         // Here we just set them to some constant values for demonstration
//         // joints[i].setVelocityTarget(0.523598); // rad/s
        
//         digitalWrite(5, HIGH);
//         delayMicroseconds(5);
//         digitalWrite(5, LOW);
//         delayMicroseconds(5);


//         delayMicroseconds(500);
//     }
// delay(5000);



    unsigned long now = micros();
    unsigned long dt_us = now - prevMicros;
    prevMicros = now;



    // Here you would:
    // 1) Compute desired joint velocities from your EE controller / Jacobian
    //    float theta0_dot_des = ...;
    //    float theta1_dot_des = ...;
    //    float theta2_dot_des = ...;
    //
    // 2) Set them as targets:
    //    joints[0].setVelocityTarget(theta0_dot_des);
    //    joints[1].setVelocityTarget(theta1_dot_des);
    //    joints[2].setVelocityTarget(theta2_dot_des);

    // Update all joints
    for (int i = 0; i < 3; ++i)
    {
        joints[i].update(dt_us, now);
    }
    // joints[0].update(dt_us, now);
    
    // Debug print
    if (millis() % 100 == 0)  // rough slow print
    {
        Serial.print("th0=");
        Serial.print(joints[0].theta);
        Serial.print("  v0=");
        Serial.print(joints[0].theta_d);
        Serial.print("  a0=");
        Serial.print(joints[0].theta_dd);
        Serial.print("  pt=");
        Serial.print(joints[0].us_per_Pulse);
        // Serial.print("  pulse=");
        // Serial.print(joints[0].pulse_counter);
    Serial.print("  PPR="); Serial.print(joints[0].PPR);
    Serial.print("  pulse_dt_us="); Serial.print(joints[0].pulse_dt_us);

    Serial.print("  RPM="); Serial.print(  1000000.0f* (1.0/joints[0].pulse_dt_us)/(float)joints[0].PPR * 60.0f );

    Serial.print("  dt_us="); Serial.println(dt_us);

    }
    // dt_us
    delayMicroseconds(5);

}
