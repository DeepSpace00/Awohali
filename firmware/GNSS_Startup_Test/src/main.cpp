#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

SFE_UBLOX_GNSS myGNSS;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    Wire.begin();
    
    if (!myGNSS.begin()) {
        Serial.println("u-blox GNSS not detected. Check wiring.");
        while (1);
    }
    Serial.println("u-blox GNSS detected!");

    // Enable the antenna supervisor using UBX_CFG_HW_ANT (key ID: 0x10710001)
    if (myGNSS.setVal8(0x10710001, 0b101)) {
        Serial.println("Antenna supervisor enabled.");
    } else {
        Serial.println("Failed to enable antenna supervisor.");
    }
}

void loop() {
    // Read antenna status
    uint8_t antennaStatus = myGNSS.getAntennaStatus();
    Serial.print("Antenna status: ");
    switch (antennaStatus) {
        case 0: Serial.println("Unknown"); break;
        case 1: Serial.println("Antenna present"); break;
        case 2: Serial.println("Antenna short circuit"); break;
        case 3: Serial.println("Antenna open circuit"); break;
        default: Serial.println("Invalid status"); break;
    }
    delay(1000);
}