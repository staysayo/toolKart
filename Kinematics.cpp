#include "Kinematics.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Kinematics.cpp
 * DESCRIPTION: Implementation of the skid-steer kinematics framework.
 * Focuses on safe vector mixing and enforcing hardware safety overrides.
 * ========================================================================= */

KinematicsCore Kinematics; // Global instance

void KinematicsCore::begin() {
    // Future initialization for acceleration profiles, PID limits, 
    // and physical wheelbase measurements will go here.
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    throttleClamped = false;
}

void KinematicsCore::setSafetyThrottleClamp(bool isClamped) {
    // This provides a strict, decoupled mechanism for the safety system 
    // to override the kinematics without modifying the core math logic.
    throttleClamped = isClamped;
}

void KinematicsCore::calculateMixedSpeeds(int16_t joyX, int16_t joyY) {
    // 1. APPLY SAFETY OVERRIDES FIRST
    // If the 490mm laser threshold is breached, the throttle (forward/reverse) 
    // is zeroed out, but steering remains active.
    int16_t processedY = throttleClamped ? 0 : joyY;
    int16_t processedX = joyX;

    // 2. DEADBAND FILTERING (Placeholder)
    // Prevents motor whining and creep when the joystick is near its mechanical center.
    // processedX = applyDeadband(processedX, 10);
    // processedY = applyDeadband(processedY, 10);

    // 3. SKID-STEER MIXING ALGORITHM (Basic Differential Drive Stub)
    // Left Motor = Throttle + Steering
    // Right Motor = Throttle - Steering
    // * NOTE: To be replaced with advanced ICC matrix math in the next phase.
    
    int32_t rawLeft = processedY + processedX;
    int32_t rawRight = processedY - processedX;

    // 4. SPEED CLAMPING & NORMALIZATION
    // Ensure the mathematical output never exceeds the EM-282D hardware limits (-255 to 255).
    // Future implementation will scale proportionally rather than hard-clipping 
    // to maintain the correct turn radius at maximum throttle.
    
    currentLeftSpeed = clampSpeed((int16_t)rawLeft, -255, 255);
    currentRightSpeed = clampSpeed((int16_t)rawRight, -255, 255);
}

int16_t KinematicsCore::applyDeadband(int16_t input, int16_t threshold) {
    // Simple deadband to eliminate joystick noise at zero
    if (abs(input) < threshold) {
        return 0;
    }
    return input;
}

int16_t KinematicsCore::clampSpeed(int16_t speed, int16_t minSpeed, int16_t maxSpeed) {
    if (speed > maxSpeed) return maxSpeed;
    if (speed < minSpeed) return minSpeed;
    return speed;
}
