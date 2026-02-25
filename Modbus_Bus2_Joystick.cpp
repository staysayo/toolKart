#include "Modbus_Bus2_Joystick.h"

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox
 * FILE: Modbus_Bus2_Joystick.cpp
 * DESCRIPTION: Implementation of the high-speed Modbus RTU joystick poller.
 * ========================================================================= */

ModbusBus2Joystick Joystick; // Global instance

void ModbusBus2Joystick::begin() {
    // 115,200 Baud for rapid HMI updates. 8 Data bits, No parity, 1 Stop bit.
    Serial2.begin(BUS2_BAUD_RATE, SERIAL_8N1);
    
    // Ensure RE/DE pin is in receive mode (LOW)
    pinMode(PIN_BUS2_RE_DE, OUTPUT);
    digitalWrite(PIN_BUS2_RE_DE, LOW);
    
    delay(10); 
    flushReceiveBuffer();
}

void ModbusBus2Joystick::update() {
    uint32_t currentMillis = millis();

    switch (currentState) {
        case STATE_IDLE:
            if (currentMillis - stateTimer >= POLLING_RATE_MS) {
                currentState = STATE_SEND_REQUEST;
            }
            break;

        case STATE_SEND_REQUEST:
            flushReceiveBuffer();
            sendReadRequest();
            stateTimer = millis(); // Reset timer right after transmission
            currentState = STATE_WAIT_RESPONSE;
            break;

        case STATE_WAIT_RESPONSE:
            // Expected Response Size: 
            // SlaveID(1) + FC(1) + ByteCount(1) + DataX(2) + DataY(2) + CRC(2) = 9 Bytes
            if (Serial2.available() >= 9) {
                processResponse();
                currentState = STATE_IDLE;
            } 
            else if (currentMillis - stateTimer >= TIMEOUT_MS) {
                // Timeout Handler
                consecutiveTimeouts++;
                if (consecutiveTimeouts >= MAX_TIMEOUTS) {
                    // LOSS OF SIGNAL FAILSAFE:
                    // If the joystick cable is cut or the MCU dies, zero out 
                    // the vectors immediately to stop the 500lb toolbox.
                    currentX = 0;
                    currentY = 0;
                    connectionActive = false;
                }
                currentState = STATE_IDLE; // Reset cycle
            }
            break;
    }
}

void ModbusBus2Joystick::sendReadRequest() {
    uint8_t frame[8];
    uint8_t frameIdx = 0;

    // --- Format Modbus RTU Read Holding Registers (FC 03) ---
    frame[frameIdx++] = SLAVE_ID_JOYSTICK;      // Slave Address (3)
    frame[frameIdx++] = MODBUS_FC_READ_HOLDING; // Function Code (3)
    
    // Starting Register Address (Assuming X is mapped to 0x0000, Y to 0x0001)
    frame[frameIdx++] = 0x00; 
    frame[frameIdx++] = 0x00; 
    
    // Number of Registers to Read (2 registers = X and Y)
    frame[frameIdx++] = 0x00; 
    frame[frameIdx++] = 0x02; 

    // Calculate and append CRC
    uint16_t crc = calculateCRC(frame, frameIdx);
    frame[frameIdx++] = crc & 0xFF;        // CRC LSB
    frame[frameIdx++] = (crc >> 8) & 0xFF; // CRC MSB

    // --- Hardware Transmission ---
    digitalWrite(PIN_BUS2_RE_DE, HIGH); // Enable Driver
    Serial2.write(frame, frameIdx);
    Serial2.flush();                    // Block until last stop bit clears
    digitalWrite(PIN_BUS2_RE_DE, LOW);  // Return to Receiver mode
}

void ModbusBus2Joystick::processResponse() {
    uint8_t response[9];
    
    // Read the exact expected number of bytes
    for (int i = 0; i < 9; i++) {
        response[i] = Serial2.read();
    }

    // Validate the CRC to ensure the payload wasn't corrupted by EMI
    uint16_t receivedCrc = response[7] | (response[8] << 8);
    uint16_t calculatedCrc = calculateCRC(response, 7);

    if (receivedCrc == calculatedCrc && response[0] == SLAVE_ID_JOYSTICK && response[1] == MODBUS_FC_READ_HOLDING) {
        // Data is clean and valid. Reset failsafe counters.
        consecutiveTimeouts = 0;
        connectionActive = true;

        // Extract 16-bit signed integers (MSB first)
        currentX = (int16_t)((response[3] << 8) | response[4]);
        currentY = (int16_t)((response[5] << 8) | response[6]);
    } else {
        // CRC failed or invalid frame. Treat as a timeout drop.
        consecutiveTimeouts++;
    }

    flushReceiveBuffer(); // Clean up any lingering trailing bytes
}

void ModbusBus2Joystick::flushReceiveBuffer() {
    while (Serial2.available()) {
        Serial2.read();
    }
}

uint16_t ModbusBus2Joystick::calculateCRC(const uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos]; 
        for (int i = 8; i != 0; i--) { 
            if ((crc & 0x0001) != 0) { 
                crc >>= 1;             
                crc ^= 0xA001;
            } else {                   
                crc >>= 1;             
            }
        }
    }
    return crc;
}
