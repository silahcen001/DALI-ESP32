// ============================================================
//  SingleAddressControl.ino
//
//  An example demonstrating how to control a specific
//  DALI device using its short address (0-63).
// 
//  Use setAllBrightness(value (in %)) for broadcasting
//
//  Wiring:
//    DALI_TX_PIN (GPIO 2)  -> DALI transceiver TX input
//    DALI_RX_PIN (GPIO 15) <- DALI transceiver RX output
// ============================================================

#include "DaliController.h"

// Pin definitions
#define DALI_TX_PIN  2
#define DALI_RX_PIN  15

// The DALI short address of the LED you want to control (0 to 63)
const uint8_t TARGET_ADDRESS = 0; 

DaliController dali;

void setup() {
    Serial.begin(115200);
    
    Serial.println("Starting DALI Single Address Example...");

    // Initialize the DALI bus
    dali.begin(DALI_TX_PIN, DALI_RX_PIN)
}

void loop() {
    // 1. Turn the specific LED ON to 100%
    Serial.print("Turning ON address ");
    Serial.println(TARGET_ADDRESS);
    dali.setBrightness(TARGET_ADDRESS, 100);
    delay(3000); // Wait 3 seconds

    // 2. Dim the specific LED to 25%
    Serial.print("Dimming address ");
    Serial.println(TARGET_ADDRESS);
    dali.setBrightness(TARGET_ADDRESS, 25);
    delay(3000); // Wait 3 seconds

    // 3. Turn the specific LED OFF (0%)
    Serial.print("Turning OFF address ");
    Serial.println(TARGET_ADDRESS);
    dali.setBrightness(TARGET_ADDRESS, 0);
    delay(3000); // Wait 3 seconds
}