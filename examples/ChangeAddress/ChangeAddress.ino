// ============================================================
//  ChangeAddress.ino
//
//  Demonstrates how to reassign a DALI device to a new 
//  short address using the changeShortAddress() method.
//
//  Wiring:
//    DALI_TX_PIN (GPIO 2)  -> DALI transceiver TX input
//    DALI_RX_PIN (GPIO 15) <- DALI transceiver RX output
// ============================================================

#include "DaliController.h"

#define DALI_TX_PIN 2
#define DALI_RX_PIN 15

// Define the addresses we want to use
int OLD_ADDRESS = 0;  // The address the device currently has
int NEW_ADDRESS = 5;  // The address we want to assign to it

DaliController dali;

// We use this variable to track which address to use in the loop
int activeAddress; 

void setup() {
    Serial.begin(115200);
    delay(1000); // Give the serial monitor time to open
    
    Serial.println("Starting DALI Address Change Example...");

    if (!dali.begin(DALI_TX_PIN, DALI_RX_PIN)) {
        Serial.println("[ERROR] Failed to initialize DALI hardware!");
        while (true) { }
    }
    
    Serial.println("DALI initialized. Waiting 3 seconds...");
    delay(3000);

    // 1. Test the light at its OLD address first
    Serial.println("Testing OLD address: turning ON");
    dali.setBrightness(OLD_ADDRESS, 100);
    delay(2000);
    
    Serial.println("Testing OLD address: turning OFF");
    dali.setBrightness(OLD_ADDRESS, 0);
    // Note: Many DALI drivers ignore dimming levels
    // below their hardcoded physical minimum (e.g., 6%)
    // so sending 0% will not turn them off.use this:
    //dali.turnOffAddress(OLD_ADDRESS);
    delay(2000);

    // 2. Attempt to change the address
    Serial.println("\n--- Attempting to change address ---");
    bool success = dali.changeShortAddress(OLD_ADDRESS, NEW_ADDRESS);

    // 3. Check the result
    if (success) {
        Serial.println("Success! The device has been reprogrammed.");
        activeAddress = NEW_ADDRESS; // Update our tracker to use the new address
    } else {
        Serial.println("Failed! The device did not accept the new address.");
        activeAddress = OLD_ADDRESS; // Fallback to the old address
    }
    
    Serial.println("Entering main loop to blink the active address...\n");
}

void loop() {
    // Continuously blink the light using whichever address is currently active
    
    Serial.print("Turning ON address ");
    Serial.println(activeAddress);
    dali.setBrightness(activeAddress, 100);
    delay(2000);

    Serial.print("Turning OFF address ");
    Serial.println(activeAddress);
    dali.setBrightness(activeAddress, 0);
    // Note: Many DALI drivers ignore dimming levels
    // below their hardcoded physical minimum (e.g., 6%)
    // so sending 0% will not turn them off.use this:
    //dali.turnOffAddress(OLD_ADDRESS);
    delay(2000);
}
