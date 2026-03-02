#ifndef MODBUS_BUS2_JOYSTICK_H
#define MODBUS_BUS2_JOYSTICK_H

#include <Arduino.h>
#include "Hardware_Config.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Modbus_Bus2_Joystick.h
 * DESCRIPTION: Non-blocking Modbus RTU driver for the Custom Joystick MCU.
 * Operates at 115,200 Baud for low-latency human-machine interface (HMI).
 * Continuously polls Slave ID 3 for raw X (Steering) and Y (Throttle) vectors.
 * Implements a strict Loss-of-Signal (LOS) failsafe.
 * ========================================================================= */

// Modbus Function Codes
constexpr uint8_t MODBUS_FC_READ_HOLDING = 0x03; // Function 3

class ModbusBus2Joystick {
public:
    // Initializes the hardware serial and pin states
    void begin();

    // The core non-blocking polling state machine. Must be called rapidly in loop().
    void update();

    // Getters for the Kinematics module to consume
    int16_t getRawX() const { return currentX; }
    int16_t getRawY() const { return currentY; }

    // System diagnostic
    bool isConnected() const { return connectionActive; }

private:
    // Decoded joystick vectors. 
    // Y: Positive = Forward, Negative = Reverse
    // X: Positive = Right, Negative = Left
    int16_t currentX = 0;
    int16_t currentY = 0;

    // Failsafe tracking
    bool connectionActive = false;
    uint8_t consecutiveTimeouts = 0;
    constexpr static uint8_t MAX_TIMEOUTS = 3; // Number of missed polls before failsafe triggers

    // State Machine definition
    enum ModbusState {
        STATE_IDLE,
        STATE_SEND_REQUEST,
        STATE_WAIT_RESPONSE
    };
    ModbusState currentState = STATE_IDLE;

    uint32_t stateTimer = 0;
    
    // 115,200 Baud is fast. 10ms is plenty of time for the slave to reply.
    constexpr static uint32_t TIMEOUT_MS = 10;     
    
    // 20ms polling rate yields a crisp 50Hz update loop to the motors
    constexpr static uint32_t POLLING_RATE_MS = 20; 

    // Internal helper methods
    void sendReadRequest();
    void processResponse();
    uint16_t calculateCRC(const uint8_t *buffer, uint16_t length);
    void flushReceiveBuffer();
};

// Expose a global instance for the main application
extern ModbusBus2Joystick Joystick;

#endif // MODBUS_BUS2_JOYSTICK_H
