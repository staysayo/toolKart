#include "Modbus_Bus1_Motors.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Modbus_Bus1_Motors.cpp
 * DESCRIPTION: Implementation of the non-blocking Modbus RTU motor driver.
 * ========================================================================= */

ModbusBus1Motors Motors; // Global instance

void ModbusBus1Motors::begin() {
    // Standard Modbus RTU is 8 data bits, Even parity, 1 stop bit (SERIAL_8E1)
    // However, many EM-282D defaults use 8N1 or 8N2. We will use SERIAL_8N1 
    // as it is the most robust generic fallback. Adjust if EM-282D is strictly 8E1.
    Serial1.begin(BUS1_BAUD_RATE, SERIAL_8N1);
    
    // Ensure RE/DE pin is in receive mode (LOW)
    pinMode(PIN_BUS1_RE_DE, OUTPUT);
    digitalWrite(PIN_BUS1_RE_DE, LOW);
    
    // Allow serial line to stabilize
    delay(10); 
    flushReceiveBuffer();
}

void ModbusBus1Motors::setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    // Clamp speeds to EM-282D max range (-255 to 255)
    targetSpeedLeft = constrain(leftSpeed, -255, 255);
    targetSpeedRight = constrain(rightSpeed, -255, 255);
}

void ModbusBus1Motors::update() {
    uint32_t currentMillis = millis();

    switch (currentState) {
        case STATE_IDLE:
            if (currentMillis - stateTimer >= POLLING_RATE_MS) {
                stateTimer = currentMillis;
                currentState = STATE_SEND_LEFT;
            }
            break;

        case STATE_SEND_LEFT:
            flushReceiveBuffer();
            sendSpeedCommand(SLAVE_ID_MOTOR_LEFT, targetSpeedLeft);
            stateTimer = currentMillis;
            currentState = STATE_WAIT_LEFT;
            break;

        case STATE_WAIT_LEFT:
            // Check if we received a response (usually an echo or acknowledgment)
            // For now, we utilize a strict timeout-based approach for non-blocking pacing.
            if (Serial1.available() >= 8) {
                // Response received successfully. Clear buffer.
                flushReceiveBuffer();
                currentState = STATE_SEND_RIGHT;
            } else if (currentMillis - stateTimer >= TIMEOUT_MS) {
                // Timeout occurred. EMI spike or disconnected wire.
                droppedPacketsLeft++;
                currentState = STATE_SEND_RIGHT; // Move on, don't block the system
            }
            break;

        case STATE_SEND_RIGHT:
            flushReceiveBuffer();
            sendSpeedCommand(SLAVE_ID_MOTOR_RIGHT, targetSpeedRight);
            stateTimer = currentMillis;
            currentState = STATE_WAIT_RIGHT;
            break;

        case STATE_WAIT_RIGHT:
            if (Serial1.available() >= 8) {
                // Response received successfully. Clear buffer.
                flushReceiveBuffer();
                currentState = STATE_IDLE; // Cycle complete
            } else if (currentMillis - stateTimer >= TIMEOUT_MS) {
                // Timeout occurred.
                droppedPacketsRight++;
                currentState = STATE_IDLE; // Cycle complete, return to idle
            }
            break;
    }
}

void ModbusBus1Motors::sendSpeedCommand(uint8_t slaveId, int16_t speed) {
    uint8_t frame[15];
    uint8_t frameIdx = 0;

    // Decode direction and absolute speed
    uint8_t direction = DIR_STOP;
    uint8_t absSpeed = 0;

    if (speed > 0) {
        direction = DIR_FORWARD;
        absSpeed = (uint8_t)speed;
    } else if (speed < 0) {
        direction = DIR_BACKWARD;
        absSpeed = (uint8_t)(speed * -1);
    } else {
        direction = DIR_STOP;
        absSpeed = 0;
    }

    // --- Format Modbus RTU Write Multiple Registers (FC 16 / 0x10) ---
    // Note: The EM-C driver maps Bus Mode, Direction, Speed, Current Limit into specific bytes.
    // Assuming standard 16-bit register mapping for EM-282D Control Word.
    
    frame[frameIdx++] = slaveId;                      // Slave Address
    frame[frameIdx++] = MODBUS_FC_WRITE_MULTIPLE;     // Function Code 16
    
    // Starting Register Address (41001 offset, often 0x03E8 depending on 0/1 indexing)
    // Sending raw address offset for control.
    frame[frameIdx++] = 0x03; 
    frame[frameIdx++] = 0xE8; 
    
    // Number of Registers (2 registers = 4 bytes of data)
    frame[frameIdx++] = 0x00; 
    frame[frameIdx++] = 0x02; 
    
    // Byte Count
    frame[frameIdx++] = 0x04; 

    // DATA BYTE 1: Bus Mode (Mandatory Mode 2: Timeout Failsafe)
    frame[frameIdx++] = BUS_MODE_TIMEOUT; 
    // DATA BYTE 2: Direction
    frame[frameIdx++] = direction;        
    // DATA BYTE 3: Speed (0-255)
    frame[frameIdx++] = absSpeed;         
    // DATA BYTE 4: Current Limit (0 = Use Driver's internal limit)
    frame[frameIdx++] = 0x00;             

    // Calculate CRC
    uint16_t crc = calculateCRC(frame, frameIdx);
    frame[frameIdx++] = crc & 0xFF;        // CRC LSB
    frame[frameIdx++] = (crc >> 8) & 0xFF; // CRC MSB

    // --- Hardware Transmission ---
    // Enable RS-485 Driver (HIGH)
    digitalWrite(PIN_BUS1_RE_DE, HIGH);
    
    // Write the full frame
    Serial1.write(frame, frameIdx);
    
    // Wait for the transmission to complete before dropping RE_DE.
    // Serial1.flush() blocks until the TX buffer is empty, ensuring the 
    // last stop bit clears the transceiver before we switch back to receive mode.
    Serial1.flush();
    
    // Disable RS-485 Driver (LOW) to listen for the slave's response
    digitalWrite(PIN_BUS1_RE_DE, LOW);
}

void ModbusBus1Motors::flushReceiveBuffer() {
    // Clear out any EMI garbage or lingering responses
    while (Serial1.available()) {
        Serial1.read();
    }
}

uint16_t ModbusBus1Motors::calculateCRC(const uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos]; // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1;             // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else {                   // Else LSB is not set
                crc >>= 1;             // Just shift right
            }
        }
    }
    return crc;
}
