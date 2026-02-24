#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>

/*
 * ==============================================================================
 * HEAVY-DUTY MOTORIZED TOOLBOX - HARDWARE CONFIGURATION
 * ==============================================================================
 * Architecture: Dual-Bus RS-485 Skid-Steer Platform
 * Central Brain: Teensy 4.1 (3.3V Logic)
 * * STRICT PINOUT MAP: Do not reassign pins without architectural review due to 
 * Teensy 4.1 hardware multiplexing limitations.
 * ==============================================================================
 */

namespace Hardware {

    // --------------------------------------------------------------------------
    // BUS 1: MOTOR CONTROL (EM-282D Controllers)
    // Protocol: Modbus RTU (Half-Duplex RS-485)
    // --------------------------------------------------------------------------
    constexpr uint8_t PIN_BUS1_RX    = 0;  // Serial1 RX
    constexpr uint8_t PIN_BUS1_TX    = 1;  // Serial1 TX
    constexpr uint8_t PIN_BUS1_RE_DE = 2;  // RS-485 Receive/Drive Enable (Output)
    
    constexpr uint32_t BUS1_BAUD     = 19200; // Optimized for EMI noise immunity over speed

    // Modbus Slave IDs for Bus 1
    constexpr uint8_t ID_MOTOR_LEFT  = 1;
    constexpr uint8_t ID_MOTOR_RIGHT = 2;

    // --------------------------------------------------------------------------
    // BUS 2: JOYSTICK INPUT
    // Protocol: Modbus RTU (Half-Duplex RS-485)
    // --------------------------------------------------------------------------
    constexpr uint8_t PIN_BUS2_RX    = 7;  // Serial2 RX
    constexpr uint8_t PIN_BUS2_TX    = 8;  // Serial2 TX
    constexpr uint8_t PIN_BUS2_RE_DE = 9;  // RS-485 Receive/Drive Enable (Output)

    constexpr uint32_t BUS2_BAUD     = 115200; // Optimized for human-perception response times

    // Modbus Slave IDs for Bus 2
    constexpr uint8_t ID_JOYSTICK    = 3;

    // --------------------------------------------------------------------------
    // SAFETY SENSORS: GALVANICALLY ISOLATED LASER OVERRIDES
    // --------------------------------------------------------------------------
    // Keyence LR-ZH490CB Lasers triggering physical relays. 
    // Teensy reads dry contacts switching to Ground. 
    // MUST BE CONFIGURED AS: INPUT_PULLUP
    constexpr uint8_t PIN_LASER_LEFT   = 24;
    constexpr uint8_t PIN_LASER_CENTER = 25;
    constexpr uint8_t PIN_LASER_RIGHT  = 26;

    // --------------------------------------------------------------------------
    // LIGHTING CONTROL
    // --------------------------------------------------------------------------
    // Hardware PWM outputs for lighting intensity control.
    constexpr uint8_t PIN_HEADLIGHTS = 22; // PWM Output
    constexpr uint8_t PIN_TAILLIGHTS = 23; // PWM Output

    // --------------------------------------------------------------------------
    // DEVELOPMENT & DEBUGGING
    // --------------------------------------------------------------------------
    // Hardware Yield Switch for PC-based Modbus injection.
    // Mode 1 (Normal): Switch Open (HIGH) - Teensy is Master.
    // Mode 2 (Yield): Switch Closed (LOW) - Teensy releases RE/DE pins.
    // MUST BE CONFIGURED AS: INPUT_PULLUP
    constexpr uint8_t PIN_DEBUG_YIELD = 32;

} // end namespace Hardware

#endif // HARDWARE_CONFIG_H
