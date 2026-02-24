#ifndef MODBUS_BUS1_MOTORS_H
#define MODBUS_BUS1_MOTORS_H

#include <Arduino.h>
#include "Hardware_Config.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Modbus_Bus1_Motors.h
 * DESCRIPTION: Non-blocking Modbus RTU driver for EM-282D Motor Controllers.
 * Handles exact speed commands, Bus Mode 2 (Timeout Failsafe) enforcement, 
 * and EMI resilience via dropped packet counting.
 * ========================================================================= */

// Modbus Function Codes
constexpr uint8_t MODBUS_FC_WRITE_MULTIPLE = 0x10; // Function 16

// EM-282D Specific Control Registers 
// (Derived from EM-C Modbus specification)
constexpr uint16_t EM282D_REG_CONTROL = 41001; // Control Register Address

// Bus Modes & Directions
constexpr uint8_t BUS_MODE_TIMEOUT = 2; // Bus control with timeout (Failsafe)
constexpr uint8_t DIR_FORWARD      = 1;
constexpr uint8_t DIR_STOP         = 2;
constexpr uint8_t DIR_BACKWARD     = 3;
constexpr uint8_t DIR_RESET_FAULT  = 4;

class ModbusBus1Motors {
public:
    // Initializes the hardware serial and pin states
    void begin();

    // The core non-blocking state machine. Must be called rapidly in loop().
    void update();

    // Sets the target speeds (-255 to 255) for the kinematic mixed output
    void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    // Getters for EMI diagnostics
    uint32_t getDroppedPacketsLeft() const { return droppedPacketsLeft; }
    uint32_t getDroppedPacketsRight() const { return droppedPacketsRight; }

private:
    // Speeds clamped to -255 to 255
    int16_t targetSpeedLeft = 0;
    int16_t targetSpeedRight = 0;

    // EMI resilience counters
    uint32_t droppedPacketsLeft = 0;
    uint32_t droppedPacketsRight = 0;

    // State Machine definition
    enum ModbusState {
        STATE_IDLE,
        STATE_SEND_LEFT,
        STATE_WAIT_LEFT,
        STATE_SEND_RIGHT,
        STATE_WAIT_RIGHT
    };
    ModbusState currentState = STATE_IDLE;

    uint32_t stateTimer = 0;
    constexpr static uint32_t TIMEOUT_MS = 20;     // 20ms timeout for response
    constexpr static uint32_t POLLING_RATE_MS = 50; // Update motors every 50ms

    // Internal helper methods
    void sendSpeedCommand(uint8_t slaveId, int16_t speed);
    uint16_t calculateCRC(const uint8_t *buffer, uint16_t length);
    void flushReceiveBuffer();
};

// Expose a global instance for the main application
extern ModbusBus1Motors Motors;

#endif // MODBUS_BUS1_MOTORS_H
