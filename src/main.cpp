#include <Arduino.h>

/* =========================================================================
 * PROJECT: Heavy-Duty Motorized Toolbox (ALPHA DIAGNOSTIC)
 * FILE: main.cpp 
 * DESCRIPTION: "Hello World" Modbus connection test for EM-282D.
 * Assumes factory default settings: Slave ID 1, 9600 Baud, 8N1.
 * Tests physical RS-485 wiring (RX, TX, RE/DE) and basic Modbus Read Holding Registers.
 * ========================================================================= */

// --- HARDWARE PINOUT (From your Blueprint) ---
constexpr uint8_t PIN_BUS1_RX    = 0; 
constexpr uint8_t PIN_BUS1_TX    = 1; 
constexpr uint8_t PIN_BUS1_RE_DE = 2; 

// --- EM-282D FACTORY DEFAULTS ---
constexpr uint8_t TARGET_SLAVE_ID = 1;
constexpr uint32_t TARGET_BAUD    = 9600; 

// --- MODBUS CONSTANTS ---
constexpr uint8_t FC_READ_HOLDING = 0x03;

// --- UTILITY PROTOTYPES ---
uint16_t calculateCRC(const uint8_t *buffer, uint16_t length);
void flushReceiveBuffer();
bool readAndDisplayDeviceInfo();
bool readAndDisplayParameters(uint8_t paramCount);

void setup() {
    // 1. Initialize PC USB Terminal
    Serial.begin(115200);
    // Wait for the serial monitor to be opened before proceeding
    while (!Serial && millis() < 5000); 

    // 2. Initialize RS-485 Control Pin
    pinMode(PIN_BUS1_RE_DE, OUTPUT);
    digitalWrite(PIN_BUS1_RE_DE, LOW); // Default to Receive Mode

    // 3. Initialize Hardware Serial 1 (Motor Bus)
    Serial1.begin(TARGET_BAUD, SERIAL_8N1);

    Serial.println("\n=======================================================");
    Serial.println("  ALPHA TEST: EM-282D MODBUS 'HELLO WORLD' DIAGNOSTIC  ");
    Serial.println("=======================================================");
    Serial.print("Target Slave ID: "); Serial.println(TARGET_SLAVE_ID);
    Serial.print("Target Baud Rate: "); Serial.println(TARGET_BAUD);
    Serial.println("-------------------------------------------------------");
    Serial.println("Send 'r' in the terminal to read data from the controller.");
    Serial.println("-------------------------------------------------------");
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        // Clear out any trailing newlines or carriage returns
        while (Serial.available()) {
            Serial.read();
        }

        if (cmd == 'r' || cmd == 'R') {
            Serial.println("\n--- INITIATING MODBUS READ SEQUENCE ---");
            
            // Step 1: Prove we can talk to it and find out how many parameters it has
            if (readAndDisplayDeviceInfo()) {
                // The EM-282D usually has 24 parameters. We will read exactly what it reports.
                // Step 2: Read the actual configuration memory
                // Small delay to prevent bus saturation
                delay(50); 
                readAndDisplayParameters(24); // Hardcoding 24 based on EM-282D spec, but can be dynamic
            } else {
                Serial.println("\n[!] COMMUNICATION FAILED.");
                Serial.println("Troubleshooting Checklist:");
                Serial.println(" 1. Is the EM-282D powered on (12V-48V)?");
                Serial.println(" 2. Are RS-485 'A' and 'B' wires swapped on the breadboard?");
                Serial.println(" 3. Is the Teensy GND connected to the MAX3485 transceiver GND?");
                Serial.println(" 4. Did the controller ID or Baud rate get changed previously?");
            }
            Serial.println("\nSend 'r' to try again...");
        }
    }
}

// =========================================================================
// MODBUS IMPLEMENTATION FUNCTIONS
// =========================================================================

bool readAndDisplayDeviceInfo() {
    flushReceiveBuffer();
    
    uint8_t frame[8];
    frame[0] = TARGET_SLAVE_ID;
    frame[1] = FC_READ_HOLDING;
    // Address 40001 (Offset 0x0000) - Device Info Block
    frame[2] = 0x00; 
    frame[3] = 0x00;
    // Quantity to read: 10 registers (0x0A)
    frame[4] = 0x00; 
    frame[5] = 0x0A;
    
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    Serial.print("TX Device Info Request... ");

    // TRANSMIT
    digitalWrite(PIN_BUS1_RE_DE, HIGH);
    Serial1.write(frame, 8);
    Serial1.flush(); // CRITICAL: Wait for last bit to leave UART
    digitalWrite(PIN_BUS1_RE_DE, LOW);
    
    Serial.print("Sent. Waiting for RX... ");

    // Expected Response: Slave(1) + FC(1) + ByteCount(1) + Data(20) + CRC(2) = 25 bytes
    uint8_t expectedBytes = 25;
    uint32_t startTime = millis();
    
    while (Serial1.available() < expectedBytes) {
        if (millis() - startTime > 100) { // 100ms Timeout
            Serial.println("TIMEOUT!");
            return false;
        }
    }

    Serial.println("Received!");

    // Read payload
    uint8_t response[25];
    for (int i = 0; i < expectedBytes; i++) {
        response[i] = Serial1.read();
    }

    // Validate CRC
    uint16_t receivedCrc = response[23] | (response[24] << 8);
    uint16_t calculatedCrc = calculateCRC(response, 23);

    if (receivedCrc != calculatedCrc) {
        Serial.println("CRC ERROR! Data corrupted by noise.");
        return false;
    }

    // Parse Device Info
    // Register 1 (Bytes 3,4) = Protocol Version
    // Register 2 (Bytes 5,6) = Device Version
    // Register 3 (Bytes 7,8) = Number of Parameters
    uint16_t protocolVer = (response[3] << 8) | response[4];
    uint16_t deviceVer = (response[5] << 8) | response[6];
    uint16_t paramCount = (response[7] << 8) | response[8];

    Serial.println("--- DEVICE INFO ---");
    Serial.print("Protocol Version: "); Serial.println(protocolVer);
    Serial.print("Device Version:   "); Serial.println(deviceVer);
    Serial.print("Parameter Count:  "); Serial.println(paramCount);

    return true;
}

bool readAndDisplayParameters(uint8_t paramCount) {
    flushReceiveBuffer();
    
    uint8_t frame[8];
    frame[0] = TARGET_SLAVE_ID;
    frame[1] = FC_READ_HOLDING;
    // Address 40101 (Offset 0x0064) - Parameter Block
    frame[2] = 0x00; 
    frame[3] = 0x64;
    // Quantity to read
    frame[4] = 0x00; 
    frame[5] = paramCount;
    
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    Serial.print("TX Parameter Request...   ");

    // TRANSMIT
    digitalWrite(PIN_BUS1_RE_DE, HIGH);
    Serial1.write(frame,
            } else {
                Serial.println("\n[!] COMMUNICATION FAILED.");
                Serial.println("Troubleshooting Checklist:");
                Serial.println(" 1. Is the EM-282D powered on (12V-48V)?");
                Serial.println(" 2. Are RS-485 'A' and 'B' wires swapped on the breadboard?");
                Serial.println(" 3. Is the Teensy GND connected to the MAX3485 transceiver GND?");
                Serial.println(" 4. Did the controller ID or Baud rate get changed previously?");
            }
            Serial.println("\nSend 'r' to try again...");
        }
    }
}

// =========================================================================
// MODBUS IMPLEMENTATION FUNCTIONS
// =========================================================================

bool readAndDisplayDeviceInfo() {
    flushReceiveBuffer();
    
    uint8_t frame[8];
    frame[0] = TARGET_SLAVE_ID;
    frame[1] = FC_READ_HOLDING;
    // Address 40001 (Offset 0x0000) - Device Info Block
    frame[2] = 0x00; 
    frame[3] = 0x00;
    // Quantity to read: 10 registers (0x0A)
    frame[4] = 0x00; 
    frame[5] = 0x0A;
    
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    Serial.print("TX Device Info Request... ");

    // TRANSMIT
    digitalWrite(PIN_BUS1_RE_DE, HIGH);
    Serial1.write(frame, 8);
    Serial1.flush(); // CRITICAL: Wait for last bit to leave UART
    digitalWrite(PIN_BUS1_RE_DE, LOW);
    
    Serial.print("Sent. Waiting for RX... ");

    // Expected Response: Slave(1) + FC(1) + ByteCount(1) + Data(20) + CRC(2) = 25 bytes
    uint8_t expectedBytes = 25;
    uint32_t startTime = millis();
    
    while (Serial1.available() < expectedBytes) {
        if (millis() - startTime > 100) { // 100ms Timeout
            Serial.println("TIMEOUT!");
            return false;
        }
    }

    Serial.println("Received!");

    // Read payload
    uint8_t response[25];
    for (int i = 0; i < expectedBytes; i++) {
        response[i] = Serial1.read();
    }

    // Validate CRC
    uint16_t receivedCrc = response[23] | (response[24] << 8);
    uint16_t calculatedCrc = calculateCRC(response, 23);

    if (receivedCrc != calculatedCrc) {
        Serial.println("CRC ERROR! Data corrupted by noise.");
        return false;
    }

    // Parse Device Info
    // Register 1 (Bytes 3,4) = Protocol Version
    // Register 2 (Bytes 5,6) = Device Version
    // Register 3 (Bytes 7,8) = Number of Parameters
    uint16_t protocolVer = (response[3] << 8) | response[4];
    uint16_t deviceVer = (response[5] << 8) | response[6];
    uint16_t paramCount = (response[7] << 8) | response[8];

    Serial.println("--- DEVICE INFO ---");
    Serial.print("Protocol Version: "); Serial.println(protocolVer);
    Serial.print("Device Version:   "); Serial.println(deviceVer);
    Serial.print("Parameter Count:  "); Serial.println(paramCount);

    return true;
}

bool readAndDisplayParameters(uint8_t paramCount) {
    flushReceiveBuffer();
    
    uint8_t frame[8];
    frame[0] = TARGET_SLAVE_ID;
    frame[1] = FC_READ_HOLDING;
    // Address 40101 (Offset 0x0064) - Parameter Block
    frame[2] = 0x00; 
    frame[3] = 0x64;
    // Quantity to read
    frame[4] = 0x00; 
    frame[5] = paramCount;
    
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    Serial.print("TX Parameter Request...   ");

    // TRANSMIT
    digitalWrite(PIN_BUS1_RE_DE, HIGH);
    Serial1.write(frame, 8);
    Serial1.flush(); 
    digitalWrite(PIN_BUS1_RE_DE, LOW);
    
    Serial.print("Sent. Waiting for RX... ");

    // Expected Response: Slave(1) + FC(1) + ByteCount(1) + Data(paramCount * 2) + CRC(2)
    uint8_t expectedBytes = 3 + (paramCount * 2) + 2;
    uint32_t startTime = millis();
    
    while (Serial1.available() < expectedBytes) {
        if (millis() - startTime > 150) { // 150ms Timeout
            Serial.println("TIMEOUT!");
            return false;
        }
    }

    Serial.println("Received!\n");

    // Read payload
    uint8_t response[100]; // Buffer safe up to ~48 parameters
    for (int i = 0; i < expectedBytes; i++) {
        response[i] = Serial1.read();
    }

    // Validate CRC
    uint16_t receivedCrc = response[expectedBytes - 2] | (response[expectedBytes - 1] << 8);
    uint16_t calculatedCrc = calculateCRC(response, expectedBytes - 2);

    if (receivedCrc != calculatedCrc) {
        Serial.println("CRC ERROR! Data corrupted by noise.");
        return false;
    }

    Serial.println("--- PARAMETER DUMP ---");
    // Parse Parameters (Data starts at byte 3, 2 bytes per register)
    for (int i = 0; i < paramCount; i++) {
        uint16_t val = (response[3 + (i * 2)] << 8) | response[4 + (i * 2)];
        Serial.print("Parameter ");
        if (i + 1 < 10) Serial.print(" "); // Formatting alignment
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(val);
    }

    return true;
}

void flushReceiveBuffer() {
    // Clear out the hardware UART buffer before starting a new transaction
    while (Serial1.available()) {
        Serial1.read();
    }
}

uint16_t calculateCRC(const uint8_t *buffer, uint16_t length) {
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
