// ============================================================
//  ScanAddresses.ino
//
//  Scans all 64 possible DALI short addresses (0-63) and
//  reports which ones have a responding device on the bus.
//
//  Wiring:
//    DALI_TX_PIN (GPIO 2)  -> DALI transceiver TX input
//    DALI_RX_PIN (GPIO 15) <- DALI transceiver RX output
//
//  Open the Serial Monitor at 115200 baud to see results.
// ============================================================

#include "DaliController.h"

// ----- Pin definitions (change to match your wiring) --------
#define DALI_TX_PIN  2
#define DALI_RX_PIN  15

// ------------------------------------------------------------

DaliController dali;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }   // Wait for Serial on boards that need it

    Serial.println();
    Serial.println(F("================================================"));
    Serial.println(F("  DaliController – Address Scanner"));
    Serial.println(F("================================================"));

    // Initialise RMT TX + RX with default channels
    if (!dali.begin(DALI_TX_PIN, DALI_RX_PIN)) {
        Serial.println(F("[ERROR] Failed to initialise DALI hardware. Halting."));
        while (true) { ; }
    }

    Serial.println(F("Hardware ready. Starting scan of addresses 0-63…"));
    Serial.println();

    int foundCount = 0;

    for (int addr = 0; addr <= 63; addr++) {

        // Query Status returns the 8-bit status byte, or -1 on no response.
        // isDevicePresent() wraps this and returns true when a device replies.
        bool present = dali.isDevicePresent(addr);

        if (present) {
            // Device found – read a bit more info
            int statusByte = dali.queryStatus(addr);
            int devType    = dali.queryDeviceType(addr);
            int actLevel   = dali.queryActualLevel(addr);

            Serial.print(F("[FOUND] Address "));
            if (addr < 10) Serial.print(' ');   // align single digits
            Serial.print(addr);
            Serial.print(F("  |  Status: 0x"));
            if (statusByte < 0x10) Serial.print('0');
            Serial.print(statusByte, HEX);
            Serial.print(F("  |  Device Type: "));
            if (devType < 0) {
                Serial.print(F("N/A"));
            } else {
                Serial.print(devType);
            }
            Serial.print(F("  |  Actual Level: "));
            if (actLevel < 0) {
                Serial.println(F("N/A"));
            } else {
                Serial.println(actLevel);
            }

            foundCount++;
        } else {
            Serial.print(F("[ -- ]  Address "));
            if (addr < 10) Serial.print(' ');
            Serial.print(addr);
            Serial.println(F("  (no response)"));
        }

        // DALI spec recommends a short gap between back-to-back queries
        delay(50);
    }

    // ---- Summary ----
    Serial.println();
    Serial.println(F("================================================"));
    Serial.print(F("  Scan complete. Found "));
    Serial.print(foundCount);
    Serial.println(F(" device(s) on the DALI bus."));
    Serial.println(F("================================================"));
}

void loop() {
    // Nothing to do – scan runs once in setup().
    // Reset the board or re-upload to scan again.
}
