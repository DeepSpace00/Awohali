#include <Wire.h>

#define WIRE Wire

#define CARD_EN 3
#define CARD_RST 7
#define CARD_IO1 6
#define CARD_IO2 5
#define CARD_IO3 4

void setup() {
  pinMode(CARD_EN, OUTPUT);
  pinMode(CARD_IO1, OUTPUT);
  WIRE.begin();

  Serial.begin(9600);
  while (!Serial)
     delay(10);
  Serial.println("\nI2C Scanner");
  digitalWrite(CARD_EN, HIGH);
  digitalWrite(CARD_IO3, LOW);
}


void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);
}
