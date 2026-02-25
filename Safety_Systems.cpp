#include "Safety_Systems.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Safety_Systems.cpp
 * DESCRIPTION: Implementation of the software debouncing and kinematic 
 * override bridging.
 * ========================================================================= */

SafetySystemsCore Safety; // Global instance

void SafetySystemsCore::begin() {
    systemTripped = false;
    
    // Ensure all internal trackers start clean
    uint32_t startMillis = millis();
    for (int i = 0; i < 3; i++) {
        lasers[i].rawState = false;
        lasers[i].debouncedState = false;
        lasers[i].lastDebounceTime = startMillis;
    }
}

void SafetySystemsCore::update() {
    uint32_t currentMillis = millis();
    bool anyLaserTripped = false;

    // 1. NON-BLOCKING DEBOUNCE OF ALL SENSORS
    for (int i = 0; i < 3; i++) {
        debounceSensor(lasers[i], currentMillis);
        
        // If any *debounced* sensor registers a LOW (contact closed to GND),
        // it means a 490mm threshold breach is confirmed.
        if (lasers[i].debouncedState == true) { 
            anyLaserTripped = true;
        }
    }

    // 2. KINEMATIC OVERRIDE LOGIC
    // We only want to spam the Kinematics object if there is a state change 
    // to keep the instruction cache clean, but for maximum safety in the 
    // alpha phase, we will continually assert the required state.
    
    if (anyLaserTripped != systemTripped) {
        systemTripped = anyLaserTripped;
        
        // Handoff to the Kinematics core:
        // True = Clamp Throttle to 0, allow Steering to persist.
        // False = Normal operation.
        Kinematics.setSafetyThrottleClamp(systemTripped);
        
        // Future Alpha-Testing hook: 
        // Trigger an audible alarm or status LED sequence here.
    }
}

void SafetySystemsCore::debounceSensor(Sensor &sensor, uint32_t currentMillis) {
    // Read the dry contact. 
    // LOW = Relay Closed (Laser Tripped)
    // HIGH = Relay Open (Path Clear, due to INPUT_PULLUP)
    bool reading = (digitalRead(sensor.pin) == LOW);

    // If the switch changed state (due to noise or actual press)
    if (reading != sensor.rawState) {
        sensor.lastDebounceTime = currentMillis; // Reset the timer
        sensor.rawState = reading;
    }

    // If the state has been stable longer than our strict 20ms threshold
    if ((currentMillis - sensor.lastDebounceTime) > DEBOUNCE_DELAY_MS) {
        // Confirm the new state
        sensor.debouncedState = sensor.rawState;
    }
}
