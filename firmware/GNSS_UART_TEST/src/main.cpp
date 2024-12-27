#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// Create a GPS object
SFE_UBLOX_GNSS myGNSS;

void setup() {
  // Initialize Serial for Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial Monitor to connect
  }
  Serial.println("Initializing GPS...");

  // Initialize Serial1 for GPS communication
  Serial1.begin(38400); // ZED-F9P default baud rate

  // Begin communication with the GPS using Serial1
  if (myGNSS.begin(Serial1)) {
    Serial.println("GPS Initialized!");
  } else {
    Serial.println("Failed to initialize GPS. Check connections.");
    while (1); // Stop execution if GPS initialization fails
  }

  // Optional: Configure GPS settings
  myGNSS.setI2COutput(COM_TYPE_UBX); // Disable I2C output (if not needed)
  myGNSS.setUART1Output(COM_TYPE_UBX + COM_TYPE_NMEA); // Enable UBX and NMEA on UART1
  //myGNSS.setUART1BaudRate(115200); // Change baud rate if needed
}

void loop() {
  // Check if new GNSS data is available
  if (myGNSS.getFixType() > 2) { // 3D Fix
    Serial.print("Latitude: ");
    Serial.print(myGNSS.getLatitude() / 1e7, 7); // Convert to decimal degrees
    Serial.print(" Longitude: ");
    Serial.print(myGNSS.getLongitude() / 1e7, 7); // Convert to decimal degrees
    Serial.print(" Altitude: ");
    Serial.println(myGNSS.getAltitude() / 1000.0, 3); // Altitude in meters
  } else {
    Serial.println("No GNSS fix.");
  }
}
