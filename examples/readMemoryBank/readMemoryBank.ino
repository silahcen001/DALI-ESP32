// ============================================================
//  ReadMemory.ino
//
//  Demonstrates how to read data from a DALI device's 
//  internal memory banks using readMemoryBank().
//
//  Wiring:
//    DALI_TX_PIN (GPIO 2)  -> DALI transceiver TX input
//    DALI_RX_PIN (GPIO 15) <- DALI transceiver RX output
// ============================================================

#include "DaliController.h"

#define DALI_TX_PIN 2
#define DALI_RX_PIN 15

// The DALI short address of the device to query (0 to 63)
const uint8_t TARGET_ADDRESS = 0; 

DaliController dali;

void setup() {
    Serial.begin(115200);
    delay(1000); 
    
    Serial.println("Starting DALI Memory Reading Example...");

    if (!dali.begin(DALI_TX_PIN, DALI_RX_PIN)) {
        Serial.println("[ERROR] Failed to initialize DALI hardware!");
        while (true) { }
    }
    
    Serial.println("DALI initialized.\n");

    // Give the bus a moment to settle
    delay(1000);

    Serial.print("Attempting to read Memory Bank 0 for Address ");
    Serial.println(TARGET_ADDRESS);
    Serial.println("------------------------------------------------");

    // Memory Bank 0 holds standard device info. 
    // Location 0x02 tells us the last accessible location in this bank.
    uint8_t lastLocation = dali.readMemoryBank(TARGET_ADDRESS, 0, 0x02);
    
    if (lastLocation == 0xFF) {
        Serial.println("[ERROR] Device did not respond or read failed.");
        Serial.println("Check your wiring, power, and ensure the target address is correct.");
    } else {
        Serial.print("Last accessible location in Bank 0: 0x");
        Serial.println(lastLocation, HEX);
        
        // Read Firmware Version (Locations 0x09 and 0x0A in Bank 0)
        uint8_t fwMajor = dali.readMemoryBank(TARGET_ADDRESS, 0, 0x09);
        uint8_t fwMinor = dali.readMemoryBank(TARGET_ADDRESS, 0, 0x0A);
        
        if (fwMajor != 0xFF && fwMinor != 0xFF) {
            Serial.print("Device Firmware Version: ");
            Serial.print(fwMajor);
            Serial.print(".");
            Serial.println(fwMinor);
        }

        // Loop through and print the first 10 bytes of Memory Bank 0
        Serial.println("\nHex Dump of Bank 0 (First 10 Bytes):");
        for (uint8_t loc = 0; loc < 10; loc++) {
            uint8_t val = dali.readMemoryBank(TARGET_ADDRESS, 0, loc);
            
            Serial.print("Location 0x");
            if (loc < 0x10) Serial.print("0"); // Leading zero for neatness
            Serial.print(loc, HEX);
            Serial.print(" : ");
            
            if (val == 0xFF) {
                Serial.println("ERR / NO RESP");
            } else {
                Serial.print("0x");
                if (val < 0x10) Serial.print("0");
                Serial.println(val, HEX);
            }
            
            // DALI protocol recommends a small delay between consecutive commands
            delay(50); 
        }
    }
}

void loop() {
    // Nothing to do in the loop for a simple read example
}