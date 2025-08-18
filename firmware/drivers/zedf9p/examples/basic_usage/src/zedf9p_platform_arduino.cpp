//
// Created by deepspace on 8/18/25.
//

#include <Arduino.h>
#include "../lib/zedf9p/zedf9p_platform.h"

// Initialize the I2C port
bool SFE_UBLOX_GNSS::begin(TwoWire &wirePort, uint8_t deviceAddress, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_I2C;
  _i2cPort = &wirePort; // Grab which port the user wants us to use
  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // We expect caller to begin their I2C port, with the speed of their choice external to the library
  // But if they forget, we start the hardware here.

  // We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
  // ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
  // hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
  //_i2cPort->begin();

  _gpsI2Caddress = deviceAddress; // Store the I2C address from user

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Initialize the Serial port
bool SFE_UBLOX_GNSS::begin(Stream &serialPort, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_SERIAL;
  _serialPort = &serialPort; // Grab which port the user wants us to use
  _signsOfLife = false;      // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Get rid of any stale serial data already in the processor's RX buffer
  while (_serialPort->available())
    _serialPort->read();

  // If assumeSuccess is true, the user must really want begin to succeed. So, let's empty the module's serial transmit buffer too!
  // Keep discarding new serial data until we see a gap of 2ms - hopefully indicating that the module's TX buffer is empty.
  if (assumeSuccess)
  {
    unsigned long startTime = millis();
    unsigned long lastActivity = startTime;
    bool keepGoing = true;
    while (keepGoing && (millis() < (startTime + (unsigned long)maxWait)))
    {
      while (_serialPort->available()) // Discard any new data
      {
        _serialPort->read();
        lastActivity = millis();
      }

      if (millis() > (lastActivity + (unsigned long)2)) // Check if we have seen no new data for at least 2ms
        keepGoing = false;
    }
  }

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Initialize for SPI
bool SFE_UBLOX_GNSS::begin(SPIClass &spiPort, uint8_t csPin, uint32_t spiSpeed, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_SPI;
  _spiPort = &spiPort;
  _csPin = csPin;
  _spiSpeed = spiSpeed;
  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // Initialize the chip select pin
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  createFileBuffer();

  // Create the SPI buffer
  if (spiBuffer == NULL) // Memory has not yet been allocated - so use new
  {
    spiBuffer = new uint8_t[getSpiTransactionSize()];
  }

  if (spiBuffer == NULL)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->print(F("begin (SPI): memory allocation failed for SPI Buffer!"));
      return (false);
    }
  }
  else
  {
    // Initialize/clear the SPI buffer - fill it with 0xFF as this is what is received from the UBLOX module if there's no data to be processed
    for (uint8_t i = 0; i < getSpiTransactionSize(); i++)
    {
      spiBuffer[i] = 0xFF;
    }
  }

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Allow the user to change I2C polling wait (the minimum interval between I2C data requests - to avoid pounding the bus)
// i2cPollingWait defaults to 100ms and is adjusted automatically when setNavigationFrequency()
// or setHNRNavigationRate() are called. But if the user is using callbacks, it might be advantageous
// to be able to set the polling wait manually.
void SFE_UBLOX_GNSS::setI2CpollingWait(uint8_t newPollingWait_ms)
{
  i2cPollingWait = newPollingWait_ms;
}

// Allow the user to change SPI polling wait
// (the minimum interval between SPI data requests when no data is available - to avoid pounding the bus)
void SFE_UBLOX_GNSS::setSPIpollingWait(uint8_t newPollingWait_ms)
{
  spiPollingWait = newPollingWait_ms;
}

// Sets the global size for I2C transactions
// Most platforms use 32 bytes (the default) but this allows users to increase the transaction
// size if the platform supports it
// Note: If the transaction size is set larger than the platforms buffer size, bad things will happen.
void SFE_UBLOX_GNSS::setI2CTransactionSize(uint8_t transactionSize)
{
  if (transactionSize < 8)
    transactionSize = 8; // Ensure transactionSize is at least 8 bytes otherwise sendI2cCommand will have problems!

  i2cTransactionSize = transactionSize;
}
uint8_t SFE_UBLOX_GNSS::getI2CTransactionSize(void)
{
  return (i2cTransactionSize);
}

// Sets the global size for the SPI buffer/transactions.
// Call this **before** begin()!
// Note: if the buffer size is too small, incoming characters may be lost if the message sent
// is larger than this buffer. If too big, you may run out of SRAM on constrained architectures!
void SFE_UBLOX_GNSS::setSpiTransactionSize(uint8_t transactionSize)
{
  if (spiBuffer == NULL)
  {
    spiTransactionSize = transactionSize;
  }
  else
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("setSpiTransactionSize: you need to call setSpiTransactionSize _before_ begin!"));
    }
#endif
  }
}
uint8_t SFE_UBLOX_GNSS::getSpiTransactionSize(void)
{
  return (spiTransactionSize);
}