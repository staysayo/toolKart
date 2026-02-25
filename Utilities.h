#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>
#include "Hardware_Config.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Utilities.h
 * DESCRIPTION: Manages auxiliary peripherals (PWM lighting) and the 
 * hardware Debug Yield switch. 
 * * The Yield Switch (Pin 32) allows the developer to physically command 
 * the Teensy to halt all Modbus transmissions, freeing the RS-485 lines 
 * so a PC diagnostic tool can poll the slave devices without collisions.
 * ========================================================================= */

class UtilitiesCore {
public:
    // Initializes pins, default PWM frequencies, and internal states
    void begin();

    // Polling loop for the debug switch debounce. Must be called in loop().
    void update();

    // --- LIGHTING CONTROL ---
    // Accepts 0-255 for 8-bit PWM brightness control
    void setHeadlights(uint8_t brightness);
    void setTaillights(uint8_t brightness);

    // --- DIAGNOSTIC CONTROL ---
    // Returns true if the hardware yield switch is closed to GND.
    // The main orchestration loop MUST check this before calling Modbus.update()
    bool isBusYielded() const { return busYielded; }

private:
    // Yield Switch State Tracking
    bool busYielded = false;
    bool rawYieldState = false;
    uint32_t lastYieldDebounceTime = 0;
    
    // 20ms debounce constraint to prevent rapid toggling of the bus state
    static constexpr uint32_t DEBOUNCE_DELAY_MS = 20; 

    // Internal helper for switch debouncing
    void processYieldSwitch(uint32_t currentMillis);
};

// Expose a global instance for the main application orchestration
extern UtilitiesCore Utilities;

#endif // UTILITIES_H
