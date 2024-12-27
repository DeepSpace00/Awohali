#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;

void printFractional(int32_t fractional, uint8_t places);
void printRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct);

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  while(!Serial); // wait for serial port to be opened

  Wire.begin();

  //myGNSS.enableDebugging(Serial);

  if (!myGNSS.setPacketCfgPayloadSize(3000))
  {
    Serial.println(F("setPacketCfgPayloadSize failed. You will not be able to poll RAWX data. Freezing."));
    while (1); // Do nothing more
  }

  if (myGNSS.begin(Wire) == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setNavigationFrequency(1); //Produce one solution per second (RAWX produces a _lot_ of data!)

  Serial.println("Time,Lat (deg),Lon (deg),Ellipsoid (m),Mean Sea Level (m),Accuracy (m),GNSS ID,SV ID,Pseudorange (m),Carrier Phase (cycles)");
}

void loop()
{
  //Query module. The module only responds when a new position is available
  if (myGNSS.getHPPOSLLH())
  {
    // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getElipsoid: returns the height above ellipsoid as an int32_t in mm
    // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
    // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
    // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
    // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

    // If you want to use the high precision latitude and longitude with the full 9 decimal places
    // you will need to use a 64-bit double - which is not supported on all platforms

    // To allow this example to run on standard platforms, we cheat by converting lat and lon to integer and fractional degrees

    // The high resolution altitudes can be converted into standard 32-bit float

    // First, let's collect the position data
    int32_t latitude = myGNSS.getHighResLatitude();
    int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
    int32_t longitude = myGNSS.getHighResLongitude();
    int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
    int32_t ellipsoid = myGNSS.getElipsoid();
    int8_t ellipsoidHp = myGNSS.getElipsoidHp();
    int32_t msl = myGNSS.getMeanSeaLevel();
    int8_t mslHp = myGNSS.getMeanSeaLevelHp();
    uint32_t accuracy = myGNSS.getHorizontalAccuracy();

    // Defines storage for the lat and lon units integer and fractional parts
    int32_t lat_int; // Integer part of the latitude in degrees
    int32_t lat_frac; // Fractional part of the latitude
    int32_t lon_int; // Integer part of the longitude in degrees
    int32_t lon_frac; // Fractional part of the longitude

    // Calculate the latitude and longitude integer and fractional parts
    lat_int = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
    lat_frac = (lat_frac * 100) + latitudeHp; // Now add the high resolution component
    if (lat_frac < 0) // If the fractional part is negative, remove the minus sign
    {
      lat_frac = 0 - lat_frac;
    }
    lon_int = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
    lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component
    if (lon_frac < 0) // If the fractional part is negative, remove the minus sign
    {
      lon_frac = 0 - lon_frac;
    }
    Serial.print(",");
    // Print the lat and lon
    Serial.print(lat_int); // Print the integer part of the latitude
    Serial.print(".");
    printFractional(lat_frac, 9); // Print the fractional part of the latitude with leading zeros
    Serial.print(",");
    Serial.print(lon_int); // Print the integer part of the latitude
    Serial.print(".");
    printFractional(lon_frac, 9); // Print the fractional part of the latitude with leading zeros
    Serial.print(",");

    // Now define float storage for the heights and accuracy
    float f_ellipsoid;
    float f_msl;
    float f_accuracy;

    // Calculate the height above ellipsoid in mm * 10^-1
    f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
    // Now convert to m
    f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

    // Calculate the height above mean sea level in mm * 10^-1
    f_msl = (msl * 10) + mslHp;
    // Now convert to m
    f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

    // Convert the horizontal accuracy (mm * 10^-1) to a float
    f_accuracy = accuracy;
    // Now convert to m
    f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

    // Finally, do the printing
    Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

    Serial.print(",");
    Serial.print(f_msl, 4); // Print the mean sea level with 4 decimal places

    Serial.print(",");
    Serial.print(f_accuracy, 4); // Print the accuracy with 4 decimal places
  }

  if(myGNSS.getRXMRAWX())
  {
    printRAWX(&myGNSS.packetUBXRXMRAWX->data);
  }
}

// Pretty-print the fractional part with leading zeros - without using printf
// (Only works with positive numbers)
void printFractional(int32_t fractional, uint8_t places)
{
  if (places > 1)
  {
    for (uint8_t place = places - 1; place > 0; place--)
    {
      if (fractional < pow(10, place))
      {
        Serial.print("0");
      }
    }
  }
  Serial.print(fractional);
}

void printRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  for (uint8_t block = 0; block < ubxDataStruct->header.numMeas; block++) // For each block
  {
    Serial.print(",");
    Serial.print(ubxDataStruct->blocks[block].gnssId);
    Serial.print(",");
    Serial.print(ubxDataStruct->blocks[block].svId);
    
    if (sizeof(double) == 8) // Check if our processor supports 64-bit double
    {
      // Convert prMes from uint8_t[8] to 64-bit double
      // prMes is little-endian
      double pseudorange;
      memcpy(&pseudorange, &ubxDataStruct->blocks[block].prMes, 8);
      Serial.print(",");
      Serial.print(pseudorange, 3);
      Serial.print(",");

      // Convert cpMes from uint8_t[8] to 64-bit double
      // cpMes is little-endian
      double carrierPhase;
      memcpy(&carrierPhase, &ubxDataStruct->blocks[block].cpMes, 8);
      Serial.print(carrierPhase, 3);
    }
  }
  Serial.println();
}