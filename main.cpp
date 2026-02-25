#include <Arduino.h>

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: main.cpp (PlatformIO Entry Point)
 * DESCRIPTION: The central orchestrator for the dual-bus RS-485 architecture.
 * Ties together hardware configuration, safety overrides, high-speed joystick 
 * polling, kinematic math, and motor control into a single, non-blocking 
 * high-frequency loop.
 * ========================================================================= */

// --- MODULE INCLUDES ---
#include "Hardware_Config.h"
#include "Utilities.h"
#include "Safety_Systems.h"
#include "Modbus_Bus2_Joystick.h"
#include "Kinematics.h"
#include "Modbus_Bus1_Motors.h"

// --- DIAGNOSTIC TIMER ---
// Used strictly for pushing human-readable data to the USB Serial monitor
uint32_t lastDebugPrintTime = 0;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 500; // 2Hz updates

void setup() {
    // 1. Initialize native USB Serial for PC debugging (Not RS-485)
    Serial.begin(115200);
    // Note: We don't use while(!Serial) because the toolbox must boot and drive 
    // even if a laptop isn't plugged in.

    // 2. Initialize Hardware Pins (Sets RE/DE low, pulls up safety pins, etc.)
    initHardwarePins();

    // 3. Boot Sub-Modules
    Utilities.begin();
    Safety.begin();
    Kinematics.begin();
    Joystick.begin();
    Motors.begin();

    // Visual Confirmation of Boot Complete (Flash Headlights)
    Utilities.setHeadlights(255);
    delay(500); // Only time delay() is permitted is during boot sequence
    Utilities.setHeadlights(0);

    Serial.println("=========================================");
    Serial.println("HEAVY-DUTY TOOLBOX: ALPHA FIRMWARE BOOTED");
    Serial.println("=========================================");
}

void loop() {
    // Current time for the main loop
    uint32_t currentMillis = millis();

    // 1. ALWAYS UPDATE UTILITIES (Monitors the Yield Switch)
    Utilities.update();

    // 2. CHECK BUS MASTERSHIP
    if (Utilities.isBusYielded()) {
        // --- PC DIAGNOSTIC MODE ---
        // The hardware switch on Pin 32 is closed. 
        // We intentionally skip Joystick and Motor updates. The Teensy goes silent.
        // The RE_DE pins naturally rest LOW, releasing the RS-485 lines to your PC.
        
        // Flash taillights to indicate Yield Mode is active
        if ((currentMillis / 250) % 2 == 0) {
            Utilities.setTaillights(255);
        } else {
            Utilities.setTaillights(0);
        }
    } 
    else {
        // --- NORMAL DRIVING MODE ---
        Utilities.setTaillights(0); // Ensure indicator is off

        // A. Update Safety Systems (Debounces lasers, checks 490mm threshold)
        Safety.update();

        // B. Update Joystick (Polls 115,200 Baud bus for human input)
        Joystick.update();

        // C. Extract vectors, enforcing Loss-Of-Signal Failsafe
        int16_t targetX = 0;
        int16_t targetY = 0;

        if (Joystick.isConnected()) {
            targetX = Joystick.getRawX();
            targetY = Joystick.getRawY();
        } else {
            // LOS triggered. The Joystick module has timed out 3 consecutive times.
            // Feed strict zeros to the kinematics to halt the toolbox.
            targetX = 0;
            targetY = 0;
        }

        // D. Calculate Kinematics (Handles safety throttle clamping internally)
        Kinematics.calculateMixedSpeeds(targetX, targetY);

        // E. Command Motors
        Motors.setMotorSpeeds(Kinematics.getLeftSpeed(), Kinematics.getRightSpeed());
        
        // F. Update Motor State Machine (Transmits 19,200 Baud Modbus frames)
        Motors.update();
    }

    // 3. USB DIAGNOSTIC TELEMETRY (Non-Blocking)
    if (currentMillis - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL_MS) {
        lastDebugPrintTime = currentMillis;

        if (Utilities.isBusYielded()) {
            Serial.println("*** BUS YIELDED TO PC. TEENSY RS-485 SILENT. ***");
        } else {
            Serial.print("JoyConn: ");
            Serial.print(Joystick.isConnected() ? "OK" : "LOS FAILSAFE!");
            Serial.print(" | Safety Trip: ");
            Serial.print(Safety.isForwardPathBlocked() ? "YES (Y-CLAMPED)" : "NO");
            Serial.print(" | Motors L/R: ");
            Serial.print(Kinematics.getLeftSpeed());
            Serial.print("/");
            Serial.print(Kinematics.getRightSpeed());
            Serial.print(" | EMI Drops L/R: ");
            Serial.print(Motors.getDroppedPacketsLeft());
            Serial.print("/");
            Serial.println(Motors.getDroppedPacketsRight());
        }
    }
}
