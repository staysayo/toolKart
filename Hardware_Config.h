#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h> // Essential for PlatformIO/C++ migration from .ino

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Hardware_Config.h
 * DESCRIPTION: Strict hardware constraint map for the Teensy 4.1.
 * Centralizes all pinout definitions, baud rates, and 
 * slave IDs to ensure architectural integrity.
 * * NOTE: Teensy 4.1 operates at 3.3V logic. DO NOT expose pins to 5V/24V.
 * ========================================================================= */

// -------------------------------------------------------------------------
// 1. DUAL-BUS RS-485 COMMUNICATION PINS
// -------------------------------------------------------------------------

// BUS 1: Motor Bus (Modbus RTU to EM-282D Controllers)
// Hardware Serial1
constexpr uint8_t PIN_BUS1_RX    = 0; // Receive from EM-282Ds
constexpr uint8_t PIN_BUS1_TX    = 1; // Transmit to EM-282Ds
constexpr uint8_t PIN_BUS1_RE_DE = 2; // Receiver Enable / Driver Enable (Output)

// BUS 2: Joystick Bus (Modbus RTU to Custom Joystick Microcontroller)
// Hardware Serial2
constexpr uint8_t PIN_BUS2_RX    = 7; // Receive from Joystick
constexpr uint8_t PIN_BUS2_TX    = 8; // Transmit to Joystick
constexpr uint8_t PIN_BUS2_RE_DE = 9; // Receiver Enable / Driver Enable (Output)

// -------------------------------------------------------------------------
// 2. NETWORK CONFIGURATION CONSTANTS
// -------------------------------------------------------------------------

// Baud Rates
constexpr uint32_t BUS1_BAUD_RATE = 19200;  // Optimized for EMI noise immunity over speed
constexpr uint32_t BUS2_BAUD_RATE = 115200; // Optimized for human-perception response times

// Modbus Slave IDs
constexpr uint8_t SLAVE_ID_MOTOR_LEFT  = 1; // Left EM-282D
constexpr uint8_t SLAVE_ID_MOTOR_RIGHT = 2; // Right EM-282D
constexpr uint8_t SLAVE_ID_JOYSTICK    = 3; // Custom Joystick Microcontroller

// -------------------------------------------------------------------------
// 3. SAFETY & SENSOR PINS (Galvanically Isolated)
// -------------------------------------------------------------------------
// Keyence LR-ZH490CB Lasers. 24V triggers physical interposing relays. 
// Teensy reads dry contacts closing to Ground (requires INPUT_PULLUP).
constexpr uint8_t PIN_LASER_LEFT   = 24; 
constexpr uint8_t PIN_LASER_CENTER = 25; 
constexpr uint8_t PIN_LASER_RIGHT  = 26; 

// -------------------------------------------------------------------------
// 4. UTILITIES & PERIPHERALS
// -------------------------------------------------------------------------
// Lighting (PWM capable pins for future dimming/effects if required)
constexpr uint8_t PIN_LIGHT_HEAD = 22; // Front Headlights (PWM Output)
constexpr uint8_t PIN_LIGHT_TAIL = 23; // Rear Taillights (PWM Output)

// Debugging / Development
// Closes to Ground (requires INPUT_PULLUP). When LOW, Teensy releases Modbus.
constexpr uint8_t PIN_DEBUG_YIELD = 32; 

// -------------------------------------------------------------------------
// HELPER FUNCTION: Initialize basic pin modes
// -------------------------------------------------------------------------
inline void initHardwarePins() {
    // RS-485 Transceiver Control Pins
    pinMode(PIN_BUS1_RE_DE, OUTPUT);
    pinMode(PIN_BUS2_RE_DE, OUTPUT);
    
    // Default Transceivers to Listening Mode (LOW) to prevent bus collisions
    digitalWrite(PIN_BUS1_RE_DE, LOW);
    digitalWrite(PIN_BUS2_RE_DE, LOW);

    // Safety Sensors (Dry contacts to GND)
    pinMode(PIN_LASER_LEFT, INPUT_PULLUP);
    pinMode(PIN_LASER_CENTER, INPUT_PULLUP);
    pinMode(PIN_LASER_RIGHT, INPUT_PULLUP);

    // Debug Yield Switch (Dry contact to GND)
    pinMode(PIN_DEBUG_YIELD, INPUT_PULLUP);

    // Lighting Outputs
    pinMode(PIN_LIGHT_HEAD, OUTPUT);
    pinMode(PIN_LIGHT_TAIL, OUTPUT);
    digitalWrite(PIN_LIGHT_HEAD, LOW);
    digitalWrite(PIN_LIGHT_TAIL, LOW);
}

#endif // HARDWARE_CONFIG_H
