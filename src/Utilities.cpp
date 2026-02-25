#include "Utilities.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Utilities.cpp
 * DESCRIPTION: Implementation of the Utilities framework. Handles 
 * non-blocking debouncing for the Yield switch and PWM output for lighting.
 * ========================================================================= */

UtilitiesCore Utilities; // Global instance

void UtilitiesCore::begin() {
    // Pin modes are strictly defined in Hardware_Config.h, but we ensure 
    // safe default outputs here.
    setHeadlights(0);
    setTaillights(0);

    // Initialize yield state
    busYielded = false;
    rawYieldState = false;
    lastYieldDebounceTime = millis();

    // Teensy 4.1 PWM Configuration (Optional, but good practice)
    // Default analogWrite frequency is usually ideal for LEDs, but we explicitly 
    // set the resolution to 8-bit (0-255) to match our uint8_t types.
    analogWriteResolution(8); 
}

void UtilitiesCore::update() {
    uint32_t currentMillis = millis();
    processYieldSwitch(currentMillis);
}

void UtilitiesCore::setHeadlights(uint8_t brightness) {
    // analogWrite on Teensy 4.1 handles the hardware timer PWM generation
    analogWrite(PIN_LIGHT_HEAD, brightness);
}

void UtilitiesCore::setTaillights(uint8_t brightness) {
    analogWrite(PIN_LIGHT_TAIL, brightness);
}

void UtilitiesCore::processYieldSwitch(uint32_t currentMillis) {
    // Read the dry contact. (Assuming Pin 32 is initialized as INPUT_PULLUP)
    // LOW = Switch Closed to GND (Yield Mode ACTIVATED)
    // HIGH = Switch Open (Normal Operation)
    bool reading = (digitalRead(PIN_DEBUG_YIELD) == LOW);

    // If the switch changed state (due to noise, bounce, or a physical flip)
    if (reading != rawYieldState) {
        lastYieldDebounceTime = currentMillis; // Reset the debounce timer
        rawYieldState = reading;
    }

    // If the state has been stable longer than the 20ms threshold
    if ((currentMillis - lastYieldDebounceTime) > DEBOUNCE_DELAY_MS) {
        // If the debounced state doesn't match our official system state, update it.
        if (reading != busYielded) {
            busYielded = reading;
            
            // NOTE: We do NOT manipulate the RE/DE pins here. 
            // Separation of Concerns dictates that the Utilities module 
            // only reports the state. The main.cpp orchestrator will read 
            // isBusYielded() and stop calling the Modbus objects, effectively 
            // leaving the transceivers in their default listening (LOW) state.
        }
    }
}
