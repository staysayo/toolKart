#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Kinematics.h
 * DESCRIPTION: Mathematical core for skid-steer topology. Translates raw 
 * X (Steering) and Y (Throttle) vectors from the joystick into independent 
 * Left and Right wheel speeds.
 * * NOTE: This is the framework. Advanced floating-point kinematics, 
 * deadband filtering, and acceleration ramping are stubbed for future implementation.
 * ========================================================================= */

class KinematicsCore {
public:
    // Initializes kinematic parameters (wheelbase, max velocities, etc.)
    void begin();

    // The primary mathematical solver. Takes raw joystick vectors and computes wheel speeds.
    // joyX: Steering vector (negative = left, positive = right)
    // joyY: Throttle vector (negative = reverse, positive = forward)
    void calculateMixedSpeeds(int16_t joyX, int16_t joyY);

    // Getters for the motor bus to consume
    int16_t getLeftSpeed() const { return currentLeftSpeed; }
    int16_t getRightSpeed() const { return currentRightSpeed; }

    // --- SAFETY OVERRIDE INTERFACE ---
    // Triggered by Safety_Systems (Pins 24, 25, 26). 
    // Clamps Y (Throttle) to 0, but allows X (Steering) to pass through 
    // so the operator can steer out of a forward-collision trap.
    void setSafetyThrottleClamp(bool isClamped);

private:
    // Output velocities mapped to EM-282D ranges (-255 to 255)
    int16_t currentLeftSpeed = 0;
    int16_t currentRightSpeed = 0;

    // Internal state for hardware safety overrides
    bool throttleClamped = false;

    // --- FUTURE PHYSICAL PARAMETERS (Stubbed) ---
    // These will be used for accurate odometry and ICC (Instantaneous Center of Curvature) math
    // float wheelRadius = 0.0; 
    // float wheelBaseWidth = 0.0;
    
    // Internal helper methods
    int16_t applyDeadband(int16_t input, int16_t threshold);
    int16_t clampSpeed(int16_t speed, int16_t minSpeed, int16_t maxSpeed);
};

// Expose a global instance for the main application orchestration
extern KinematicsCore Kinematics;

#endif // KINEMATICS_H
