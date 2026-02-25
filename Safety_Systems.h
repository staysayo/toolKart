#ifndef SAFETY_SYSTEMS_H
#define SAFETY_SYSTEMS_H

#include <Arduino.h>
#include "Hardware_Config.h"
#include "Kinematics.h" // Required to trigger the throttle clamp

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Safety_Systems.h
 * DESCRIPTION: Non-blocking hardware safety monitor. 
 * Reads isolated mechanical relays triggered by 24V Keyence lasers. 
 * Enforces a 20ms software debounce to reject EMI and mechanical bounce, 
 * then commands the Kinematics core to clamp forward/reverse throttle 
 * while preserving steering.
 * ========================================================================= */

class SafetySystemsCore {
public:
    // Initializes internal states. (Pin modes are handled in Hardware_Config.h)
    void begin();

    // The primary non-blocking polling loop. Must be called rapidly in loop().
    void update();

    // Global system status for UI/Logging
    bool isForwardPathBlocked() const { return systemTripped; }

private:
    // 20ms Debounce constraint as defined in the Blueprint
    static constexpr uint32_t DEBOUNCE_DELAY_MS = 20;

    // Struct to manage individual sensor states without blocking
    struct Sensor {
        uint8_t pin;
        bool rawState = false;
        bool debouncedState = false;
        uint32_t lastDebounceTime = 0;
    };

    // Array of our forward-facing laser relays
    Sensor lasers[3] = {
        {PIN_LASER_LEFT, false, false, 0},
        {PIN_LASER_CENTER, false, false, 0},
        {PIN_LASER_RIGHT, false, false, 0}
    };

    // Master system state
    bool systemTripped = false;

    // Internal helper method to process individual pin debouncing
    void debounceSensor(Sensor &sensor, uint32_t currentMillis);
};

// Expose a global instance for the main application orchestration
extern SafetySystemsCore Safety;

#endif // SAFETY_SYSTEMS_H
