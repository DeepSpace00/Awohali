#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
SFE_UBLOX_GNSS myGNSS;

#define CARD_EN 3

void setup() {
  pinMode(CARD_EN, OUTPUT);
  digitalWrite(CARD_EN, HIGH);
  delay(1000);
  
  Serial.begin(115200);
  while(!Serial);
  Serial.println("u-blox GNSS UBX Command Test");

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // Enable the antenna supervisor using UBX_CFG_HW_ANT (key ID: 0x10710001)
  if (myGNSS.setVal8(0x10710001, 0b101)) {
      Serial.println("Antenna supervisor enabled.");
  } else {
      Serial.println("Failed to enable antenna supervisor.");
  }

  if (myGNSS.getModuleInfo())
  {
    Serial.print(F("FWVER: "));
    Serial.print(myGNSS.getFirmwareVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getFirmwareVersionLow()); // Returns uint8_t
    
    Serial.print(F("Firmware: "));
    Serial.println(myGNSS.getFirmwareType()); // Returns HPG, SPG etc. as (const char *)

    Serial.print(F("PROTVER: "));
    Serial.print(myGNSS.getProtocolVersionHigh()); // Returns uint8_t
    Serial.print(F("."));
    Serial.println(myGNSS.getProtocolVersionLow()); // Returns uint8_t
    
    Serial.print(F("MOD: "));
    Serial.println(myGNSS.getModuleName()); // Returns ZED-F9P, MAX-M10S etc. as (const char *)
  }
  else
    Serial.println(F("Error: could not read module info!"));

  // Use the helper method to read the unique chip ID as a string
  // Returns "000000000000" if the read fails
  Serial.print(F("Unique chip ID: 0x"));
  Serial.println(myGNSS.getUniqueChipIdStr());

  UBX_MON_RF_data_t rfInformation;
  if (myGNSS.getRFinformation(&rfInformation, 2000))
  {
    Serial.print(F("The UBX_MON_RF message contains "));
    Serial.print(rfInformation.header.nBlocks); // Print how many information blocks were returned. Should be 0, 1 or 2
    Serial.println(F(" information blocks"));

    for (uint8_t block = 0; block < rfInformation.header.nBlocks; block++)
    {
      Serial.print(F("Block ID: "));
      Serial.print(rfInformation.blocks[block].blockId);
      if (rfInformation.blocks[block].blockId == 0)
        Serial.println(F(" = L1"));
      else // if (rfInformation.blocks[block].blockId == 1)
        Serial.println(F(" = L2 / L5"));
        
      Serial.print(F("Jamming state: "));
      Serial.print(rfInformation.blocks[block].flags.bits.jammingState);
      if (rfInformation.blocks[block].flags.bits.jammingState == 0)
        Serial.println(F(" = unknown / disabled"));
      else if (rfInformation.blocks[block].flags.bits.jammingState == 1)
        Serial.println(F(" = ok"));
      else if (rfInformation.blocks[block].flags.bits.jammingState == 2)
        Serial.println(F(" = warning"));
      else // if (rfInformation.blocks[block].flags.bits.jammingState == 3)
        Serial.println(F(" = critical!"));

      Serial.print(F("Antenna STS: "));
      if (rfInformation.blocks[block].antStatus == 0)
        Serial.println(F("INIT"));
      else if (rfInformation.blocks[block].antStatus == 1)
        Serial.println(F("DONTKNOW"));
      else if (rfInformation.blocks[block].antStatus == 2)
        Serial.println(F("OK"));
      else if (rfInformation.blocks[block].antStatus == 3)
        Serial.println(F("SHORT"));
      else
        Serial.println(F("OPEN"));

      Serial.print(F("Antenna PWR: "));
      if (rfInformation.blocks[block].antStatus == 0)
        Serial.println(F("OFF"));
      else if (rfInformation.blocks[block].antStatus == 1)
        Serial.println(F("ON"));
      else
        Serial.println(F("DONTKNOW"));

      Serial.print(F("Noise level: "));
      Serial.println(rfInformation.blocks[block].noisePerMS);
      
      Serial.print(F("AGC monitor: "));
      Serial.println(rfInformation.blocks[block].agcCnt);
      
      Serial.print(F("CW jamming indicator: "));
      Serial.println(rfInformation.blocks[block].jamInd);      
    }

    Serial.println();
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
