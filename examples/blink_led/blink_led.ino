#include "DaliController.h"

#define DALI_TX_PIN  2
#define DALI_RX_PIN  15

DaliController dali;

int address = 10;

void setup() {

    Serial.begin(9600);
    dali.begin(DALI_TX_PIN, DALI_RX_PIN);
    
    
}
void loop() {
    Serial.println("Turning on the light at address " + String(address));
    dali.turnOnAddress(address);
    delay(2000);
    Serial.println("Turning off the light at address " + String(address));
    dali.turnOffAddress(address);
    delay(2000);
}