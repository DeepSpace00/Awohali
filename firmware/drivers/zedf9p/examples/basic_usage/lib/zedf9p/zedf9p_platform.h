/**
* @file zedf9p_platform.h
 * @brief Platform abstraction layer for ZED-F9P GNSS module driver
 * @author Generated Driver
 * @version 1.0
 *
 * Platform-specific functions that need to be implemented by the user
 * for their specific hardware platform (Arduino, STM32, Zephyr, etc.)
 */

#ifndef ZEDF9P_PLATFORM_H
#define ZEDF9P_PLATFORM_H

#include <stdint.h>

  // Begin communication with the GNSS. Advanced users can assume success if required. Useful if the port is already outputting messages at high navigation rate.
  // Begin will then return true if "signs of life" have been seen: reception of _any_ valid UBX packet or _any_ valid NMEA header.
  // By default use the default I2C address, and use Wire port
  bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x42, uint16_t maxWait = defaultMaxWait, bool assumeSuccess = false); // Returns true if module is detected
  // serialPort needs to be perviously initialized to correct baud rate
  bool begin(Stream &serialPort, uint16_t maxWait = defaultMaxWait, bool assumeSuccess = false); // Returns true if module is detected
  // SPI - supply instance of SPIClass, chip select pin and SPI speed (in Hz)
  bool begin(SPIClass &spiPort, uint8_t csPin, uint32_t spiSpeed, uint16_t maxWait = defaultMaxWait, bool assumeSuccess = false);

  void end(void); // Stop all automatic message processing. Free all used RAM

  void setI2CpollingWait(uint8_t newPollingWait_ms); // Allow the user to change the I2C polling wait if required
  void setSPIpollingWait(uint8_t newPollingWait_ms); // Allow the user to change the SPI polling wait if required

  // Set the max number of bytes set in a given I2C transaction
  uint8_t i2cTransactionSize = 32; // Default to ATmega328 limit

  // Control the size of the internal I2C transaction amount
  void setI2CTransactionSize(uint8_t bufferSize);
  uint8_t getI2CTransactionSize(void);

  // Support for platforms like ESP32 which do not support multiple I2C restarts
  // If _i2cStopRestart is true, endTransmission will always use a stop. If false, a restart will be used where needed.
  // The default value for _i2cStopRestart is set in the class instantiation code.
  void setI2cStopRestart(bool stop) { _i2cStopRestart = stop; };
  bool getI2cStopRestart(void) { return (_i2cStopRestart); };

  // Control the size of the spi buffer. If the buffer isn't big enough, we'll start to lose bytes
  // That we receive if the buffer is full!
  void setSpiTransactionSize(uint8_t bufferSize);
  uint8_t getSpiTransactionSize(void);

#endif //ZEDF9P_PLATFORM_H