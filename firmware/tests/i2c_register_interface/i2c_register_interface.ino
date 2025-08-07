#include <Wire.h>

// Configuration
#define MAX_BUFFER_SIZE 128
#define I2C_TIMEOUT 1000
#define DEFAULT_CLOCK_SPEED 100000L

// Error codes
enum I2CErrorCode {
  I2C_ERR_SUCCESS = 0,
  I2C_ERR_DATA_TOO_LONG = 1,
  I2C_ERR_NACK_ON_ADDRESS = 2,
  I2C_ERR_NACK_ON_DATA = 3,
  I2C_ERR_OTHER_ERROR = 4,
  I2C_ERR_TIMEOUT = 5
};

// Device profiles for known quirks
struct DeviceProfile {
  uint8_t address;
  bool supports_bulk_read;
  bool requires_stop_bit;
  bool has_crc;
  uint8_t max_read_size;
  uint16_t read_delay_ms;
  const char* name;
};

// Known device profiles (expandable)
DeviceProfile known_i2c_devices[] = {
  {0x2C, false, true, false, 32, 1, "USB2422 Hub Controller"},
  {0x44, false, true, true, 6, 10, "SHT41 Temp/Humidity"},
  {0x45, false, true, true, 6, 10, "SHT41 Temp/Humidity Alt"},
  {0x48, true, false, false, 32, 0, "Generic Sensor"},
  {0x68, true, false, false, 16, 1, "RTC/IMU"},
  {0x76, false, true, false, 1, 10, "Pressure Sensor"},
  {0x77, false, true, false, 1, 10, "Pressure Sensor Alt"},
  // Add more profiles as needed
};

// Global settings
uint32_t i2c_clock = DEFAULT_CLOCK_SPEED;
bool verbose_mode = false;
bool auto_detect_profile = true;

// Function declarations
void printPrompt();
void processCommand(String cmd);
void printHelp();
void scanBus();
void checkKnownDevices();
bool isDevicePresent(uint8_t addr);
void handleReadCommand(String cmd);
bool parseReadCommand(String cmd, uint8_t &addr, uint8_t &reg, uint16_t &len);
void handleWriteCommand(String cmd);
bool parseWriteCommand(String cmd, uint8_t &addr, uint8_t &reg, uint8_t values[], uint8_t &count);
void handleDumpCommand(String cmd);
bool parseDumpCommand(String cmd, uint8_t &addr, uint16_t &start, uint16_t &end);
void handleBulkReadCommand(String cmd);
bool parseBulkCommand(String cmd, uint8_t &addr, uint8_t &start, uint16_t &len);
void handleClockCommand(String cmd);
void handleProfileCommand(String cmd);
void handleSHT41Command(String cmd);
void handleUSB2422Command(String cmd);
I2CErrorCode readRegisters(uint8_t addr, uint8_t reg, uint8_t* buffer, uint16_t len);
I2CErrorCode writeRegisters(uint8_t addr, uint8_t reg, uint8_t* values, uint8_t len);
void dumpRegisters(uint8_t addr, uint16_t start, uint16_t end);
void bulkRead(uint8_t addr, uint8_t start, uint16_t len);
DeviceProfile* getDeviceProfile(uint8_t addr);
void showDeviceProfile(uint8_t addr);
void printReadResult(uint8_t addr, uint8_t reg, uint8_t* buffer, uint16_t len);
void printI2CError(I2CErrorCode error_code, uint8_t addr);
uint32_t parseNumber(String str);
void resetI2C();
void sht41Measure(uint8_t addr);
void sht41ReadSerial(uint8_t addr);
void sht41SoftReset(uint8_t addr);
void sht41Heater(uint8_t addr, String cmd);
bool verifySHT41CRC(uint8_t data1, uint8_t data2, uint8_t expected_crc);
void usb2422BlockRead(uint8_t addr, uint8_t reg, uint8_t len);
void usb2422BlockWrite(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len);
I2CErrorCode usb2422SMBusBlockRead(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t* received_len);
I2CErrorCode usb2422SMBusBlockWrite(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len);
void usb2422RawTest(uint8_t addr, uint8_t reg);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Wire.begin();
  Wire.setClock(i2c_clock);
  Wire.setTimeout(I2C_TIMEOUT);
  
  Serial.println(F("\n=== Arduino I2C Debug CLI Tool ==="));
  Serial.println(F("Type 'help' for commands"));
  printPrompt();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
    printPrompt();
  }
}

void printPrompt() {
  Serial.print(F("I2C> "));
}

void processCommand(String cmd) {
  cmd.toLowerCase();
  
  if (cmd.startsWith("help")) {
    printHelp();
  }
  else if (cmd.startsWith("scan")) {
    scanBus();
  }
  else if (cmd.startsWith("read ")) {
    handleReadCommand(cmd);
  }
  else if (cmd.startsWith("write ")) {
    handleWriteCommand(cmd);
  }
  else if (cmd.startsWith("dump ")) {
    handleDumpCommand(cmd);
  }
  else if (cmd.startsWith("bulk ")) {
    handleBulkReadCommand(cmd);
  }
  else if (cmd.startsWith("clock ")) {
    handleClockCommand(cmd);
  }
  else if (cmd.startsWith("profile ")) {
    handleProfileCommand(cmd);
  }
  else if (cmd.startsWith("sht41 ")) {
    handleSHT41Command(cmd);
  }
  else if (cmd.startsWith("usb2422 ")) {
    handleUSB2422Command(cmd);
  }
  else if (cmd.startsWith("verbose")) {
    verbose_mode = !verbose_mode;
    Serial.print(F("Verbose mode: "));
    Serial.println(verbose_mode ? F("ON") : F("OFF"));
  }
  else if (cmd.startsWith("sht41 ")) {
    handleSHT41Command(cmd);
  }
  else if (cmd.startsWith("reset")) {
    resetI2C();
  }
}

void printHelp() {
  Serial.println(F("\n=== I2C Debug Commands ==="));
  Serial.println(F("scan                     - Scan for devices on bus"));
  Serial.println(F("read <addr> <reg>        - Read single register"));
  Serial.println(F("read <addr> <reg> <len>  - Read multiple registers"));
  Serial.println(F("write <addr> <reg> <val> - Write single register"));
  Serial.println(F("write <addr> <reg> <val1> <val2> ... - Write multiple"));
  Serial.println(F("dump <addr>              - Dump all registers (0-255)"));
  Serial.println(F("dump <addr> <start> <end> - Dump register range"));
  Serial.println(F("bulk <addr> <start> <len> - Bulk read with error handling"));
  Serial.println(F("clock <speed>            - Set I2C clock speed (Hz)"));
  Serial.println(F("profile <addr>           - Show/set device profile"));
  Serial.println(F("sht41 <addr> <cmd>       - SHT41 specific commands"));
  Serial.println(F("usb2422 <addr> <cmd>     - USB2422 SMBus commands"));
  Serial.println(F("verbose                  - Toggle verbose output"));
  Serial.println(F("reset                    - Reset I2C bus"));
  Serial.println(F("\nAddresses and values in hex (0x48) or decimal (72)"));
  Serial.println(F("Examples:"));
  Serial.println(F("  scan"));
  Serial.println(F("  read 0x48 0x00"));
  Serial.println(F("  write 0x48 0x01 0xFF"));
  Serial.println(F("  dump 0x48 0x00 0x10"));
  Serial.println(F("  sht41 0x44 measure"));
  Serial.println(F("  sht41 0x44 serial"));
  Serial.println(F("  usb2422 0x2C read 0x00 4"));
  Serial.println(F("  usb2422 0x2C write 0x00 0x01 0x02"));
}

void scanBus() {
  Serial.println(F("\nScanning I2C bus..."));
  Serial.println(F("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  
  int device_count = 0;
  for (uint8_t row = 0; row < 8; row++) {
    Serial.print(row, HEX);
    Serial.print(F("0: "));
    
    for (uint8_t col = 0; col < 16; col++) {
      uint8_t addr = (row << 4) | col;
      
      if (addr < 0x08 || addr > 0x77) {
        Serial.print(F("   "));
        continue;
      }
      
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();
      
      if (error == 0) {
        if (addr < 16) Serial.print(F("0"));
        Serial.print(addr, HEX);
        Serial.print(F(" "));
        device_count++;
      } else {
        Serial.print(F("-- "));
      }
    }
    Serial.println();
  }
  
  Serial.print(F("\nFound "));
  Serial.print(device_count);
  Serial.println(F(" device(s)"));
  
  if (device_count > 0 && auto_detect_profile) {
    Serial.println(F("\nChecking known device profiles..."));
    checkKnownDevices();
  }
}

void checkKnownDevices() {
  for (int i = 0; i < sizeof(known_i2c_devices) / sizeof(DeviceProfile); i++) {
    if (isDevicePresent(known_i2c_devices[i].address)) {
      Serial.print(F("Device at 0x"));
      Serial.print(known_i2c_devices[i].address, HEX);
      Serial.print(F(": "));
      Serial.println(known_i2c_devices[i].name);
    }
  }
}

bool isDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void handleReadCommand(String cmd) {
  uint8_t addr, reg;
  uint16_t len = 1;
  
  if (!parseReadCommand(cmd, addr, reg, len)) {
    Serial.println(F("Usage: read <addr> <reg> [length]"));
    return;
  }
  
  if (verbose_mode) {
    Serial.print(F("Reading "));
    Serial.print(len);
    Serial.print(F(" byte(s) from device 0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" register 0x"));
    Serial.println(reg, HEX);
  }
  
  uint8_t buffer[MAX_BUFFER_SIZE];
  I2CErrorCode result = readRegisters(addr, reg, buffer, len);
  
  if (result == I2C_ERR_SUCCESS) {
    printReadResult(addr, reg, buffer, len);
  } else {
    printI2CError(result, addr);
  }
}

bool parseReadCommand(String cmd, uint8_t &addr, uint8_t &reg, uint16_t &len) {
  int first_space = cmd.indexOf(' ');
  int second_space = cmd.indexOf(' ', first_space + 1);
  int third_space = cmd.indexOf(' ', second_space + 1);
  
  if (second_space == -1) return false;
  
  String addr_str = cmd.substring(first_space + 1, second_space);
  String reg_str = cmd.substring(second_space + 1, third_space == -1 ? cmd.length() : third_space);
  
  addr = parseNumber(addr_str);
  reg = parseNumber(reg_str);
  
  if (third_space != -1) {
    String len_str = cmd.substring(third_space + 1);
    len = parseNumber(len_str);
    len = constrain(len, 1, MAX_BUFFER_SIZE);
  }
  
  return true;
}

void handleWriteCommand(String cmd) {
  uint8_t addr, reg;
  uint8_t values[MAX_BUFFER_SIZE];
  uint8_t value_count = 0;
  
  if (!parseWriteCommand(cmd, addr, reg, values, value_count)) {
    Serial.println(F("Usage: write <addr> <reg> <val1> [val2] [val3] ..."));
    return;
  }
  
  if (verbose_mode) {
    Serial.print(F("Writing "));
    Serial.print(value_count);
    Serial.print(F(" byte(s) to device 0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" register 0x"));
    Serial.println(reg, HEX);
  }
  
  I2CErrorCode result = writeRegisters(addr, reg, values, value_count);
  
  if (result == I2C_ERR_SUCCESS) {
    Serial.print(F("Write successful: "));
    for (uint8_t i = 0; i < value_count; i++) {
      Serial.print(F("0x"));
      if (values[i] < 16) Serial.print(F("0"));
      Serial.print(values[i], HEX);
      if (i < value_count - 1) Serial.print(F(" "));
    }
    Serial.println();
  } else {
    printI2CError(result, addr);
  }
}

bool parseWriteCommand(String cmd, uint8_t &addr, uint8_t &reg, uint8_t values[], uint8_t &count) {
  count = 0;
  int pos = cmd.indexOf(' ') + 1;
  
  // Parse address
  int next_pos = cmd.indexOf(' ', pos);
  if (next_pos == -1) return false;
  addr = parseNumber(cmd.substring(pos, next_pos));
  pos = next_pos + 1;
  
  // Parse register
  next_pos = cmd.indexOf(' ', pos);
  if (next_pos == -1) return false;
  reg = parseNumber(cmd.substring(pos, next_pos));
  pos = next_pos + 1;
  
  // Parse values
  while (pos < cmd.length() && count < MAX_BUFFER_SIZE) {
    next_pos = cmd.indexOf(' ', pos);
    if (next_pos == -1) next_pos = cmd.length();
    
    values[count] = parseNumber(cmd.substring(pos, next_pos));
    count++;
    pos = next_pos + 1;
  }
  
  return count > 0;
}

void handleDumpCommand(String cmd) {
  uint8_t addr;
  uint16_t start = 0, end = 255;
  
  if (!parseDumpCommand(cmd, addr, start, end)) {
    Serial.println(F("Usage: dump <addr> [start] [end]"));
    return;
  }
  
  Serial.print(F("Dumping registers 0x"));
  Serial.print(start, HEX);
  Serial.print(F(" to 0x"));
  Serial.print(end, HEX);
  Serial.print(F(" from device 0x"));
  Serial.println(addr, HEX);
  
  dumpRegisters(addr, start, end);
}

bool parseDumpCommand(String cmd, uint8_t &addr, uint16_t &start, uint16_t &end) {
  int first_space = cmd.indexOf(' ');
  int second_space = cmd.indexOf(' ', first_space + 1);
  int third_space = cmd.indexOf(' ', second_space + 1);
  
  if (first_space == -1) return false;
  
  String addr_str = cmd.substring(first_space + 1, second_space == -1 ? cmd.length() : second_space);
  addr = parseNumber(addr_str);
  
  if (second_space != -1) {
    String start_str = cmd.substring(second_space + 1, third_space == -1 ? cmd.length() : third_space);
    start = parseNumber(start_str);
    
    if (third_space != -1) {
      String end_str = cmd.substring(third_space + 1);
      end = parseNumber(end_str);
    }
  }
  
  return true;
}

void handleBulkReadCommand(String cmd) {
  uint8_t addr, start;
  uint16_t len;
  
  if (!parseBulkCommand(cmd, addr, start, len)) {
    Serial.println(F("Usage: bulk <addr> <start> <length>"));
    return;
  }
  
  Serial.print(F("Bulk reading "));
  Serial.print(len);
  Serial.print(F(" bytes from 0x"));
  Serial.print(addr, HEX);
  Serial.print(F(" starting at 0x"));
  Serial.println(start, HEX);
  
  bulkRead(addr, start, len);
}

bool parseBulkCommand(String cmd, uint8_t &addr, uint8_t &start, uint16_t &len) {
  int first_space = cmd.indexOf(' ');
  int second_space = cmd.indexOf(' ', first_space + 1);
  int third_space = cmd.indexOf(' ', second_space + 1);
  
  if (third_space == -1) return false;
  
  String addr_str = cmd.substring(first_space + 1, second_space);
  String start_str = cmd.substring(second_space + 1, third_space);
  String len_str = cmd.substring(third_space + 1);
  
  addr = parseNumber(addr_str);
  start = parseNumber(start_str);
  len = parseNumber(len_str);
  
  return true;
}

void handleClockCommand(String cmd) {
  int space_pos = cmd.indexOf(' ');
  if (space_pos == -1) {
    Serial.print(F("Current I2C clock: "));
    Serial.print(i2c_clock);
    Serial.println(F(" Hz"));
    return;
  }
  
  String speed_str = cmd.substring(space_pos + 1);
  uint32_t new_speed = parseNumber(speed_str);
  
  if (new_speed >= 1000 && new_speed <= 1000000) {
    i2c_clock = new_speed;
    Wire.setClock(i2c_clock);
    Serial.print(F("I2C clock set to "));
    Serial.print(i2c_clock);
    Serial.println(F(" Hz"));
  } else {
    Serial.println(F("Invalid speed. Range: 1000-1000000 Hz"));
  }
}

void handleProfileCommand(String cmd) {
  int space_pos = cmd.indexOf(' ');
  if (space_pos == -1) {
    Serial.println(F("Usage: profile <addr>"));
    return;
  }
  
  String addr_str = cmd.substring(space_pos + 1);
  uint8_t addr = parseNumber(addr_str);
  
  showDeviceProfile(addr);
}

I2CErrorCode readRegisters(uint8_t addr, uint8_t reg, uint8_t* buffer, uint16_t len) {
  // Get device profile
  DeviceProfile* profile = getDeviceProfile(addr);
  
  // Write register address
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(profile ? profile->requires_stop_bit : true);
  
  if (error != 0) {
    return (I2CErrorCode)error;
  }
  
  // Add delay if required by device profile
  if (profile && profile->read_delay_ms > 0) {
    delay(profile->read_delay_ms);
  }
  
  // Read data
  uint16_t bytes_read = Wire.requestFrom(addr, (uint8_t)min(len, 32));
  
  if (bytes_read == 0) {
    return I2C_ERR_NACK_ON_ADDRESS;
  }
  
  for (uint16_t i = 0; i < bytes_read && i < len; i++) {
    buffer[i] = Wire.read();
  }
  
  return I2C_ERR_SUCCESS;
}

I2CErrorCode writeRegisters(uint8_t addr, uint8_t reg, uint8_t* values, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(values[i]);
  }
  
  uint8_t error = Wire.endTransmission();
  return (I2CErrorCode)error;
}

void usb2422RawTest(uint8_t addr, uint8_t reg) {
  Serial.print(F("USB2422 Raw I2C Test - Addr: 0x"));
  Serial.print(addr, HEX);
  Serial.print(F(" Reg: 0x"));
  Serial.println(reg, HEX);
  
  // Test 1: Simple register write then read (like standard I2C)
  Serial.println(F("Test 1: Standard I2C read"));
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(false); // Repeated start
  
  Serial.print(F("Write result: "));
  Serial.println(error);
  
  if (error == 0) {
    uint8_t bytes_available = Wire.requestFrom(addr, (uint8_t)8);
    Serial.print(F("Bytes available: "));
    Serial.println(bytes_available);
    
    if (bytes_available > 0) {
      Serial.print(F("Data: "));
      for (uint8_t i = 0; i < bytes_available; i++) {
        uint8_t data = Wire.read();
        Serial.print(F("0x"));
        if (data < 16) Serial.print(F("0"));
        Serial.print(data, HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }
  }
  
  delay(10);
  
  // Test 2: Write some data first, then try to read it
  Serial.println(F("Test 2: Write then read"));
  
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(2); // Byte count
  Wire.write(0xAA);
  Wire.write(0xBB);
  error = Wire.endTransmission();
  
  Serial.print(F("Write result: "));
  Serial.println(error);
  
  if (error == 0) {
    delay(10);
    
    // Now try to read it back
    Wire.beginTransmission(addr);
    Wire.write(reg);
    error = Wire.endTransmission(false);
    
    if (error == 0) {
      uint8_t bytes_available = Wire.requestFrom(addr, (uint8_t)8);
      Serial.print(F("Read back bytes available: "));
      Serial.println(bytes_available);
      
      if (bytes_available > 0) {
        Serial.print(F("Read back data: "));
        for (uint8_t i = 0; i < bytes_available; i++) {
          uint8_t data = Wire.read();
          Serial.print(F("0x"));
          if (data < 16) Serial.print(F("0"));
          Serial.print(data, HEX);
          Serial.print(F(" "));
        }
        Serial.println();
      }
    }
  }
  
  // Test 3: Try without repeated start
  Serial.println(F("Test 3: Full stop/start cycle"));
  
  Wire.beginTransmission(addr);
  Wire.write(reg);
  error = Wire.endTransmission(true); // Full stop
  
  if (error == 0) {
    delay(1);
    uint8_t bytes_available = Wire.requestFrom(addr, (uint8_t)8);
    Serial.print(F("Bytes available (full stop): "));
    Serial.println(bytes_available);
    
    if (bytes_available > 0) {
      Serial.print(F("Data (full stop): "));
      for (uint8_t i = 0; i < bytes_available; i++) {
        uint8_t data = Wire.read();
        Serial.print(F("0x"));
        if (data < 16) Serial.print(F("0"));
        Serial.print(data, HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }
  }
}

void dumpRegisters(uint8_t addr, uint16_t start, uint16_t end) {
  Serial.println(F("\n     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F"));
  Serial.println(F("    ================================================"));
  
  DeviceProfile* profile = getDeviceProfile(addr);
  uint8_t read_size = profile ? profile->max_read_size : 1;
  
  for (uint16_t row_start = start & 0xFFF0; row_start <= end; row_start += 16) {
    bool row_has_data = false;
    uint8_t row_data[16];
    bool row_valid[16];
    
    // Read entire row
    for (uint8_t col = 0; col < 16; col++) {
      uint16_t reg = row_start + col;
      row_valid[col] = false;
      
      if (reg >= start && reg <= end) {
        I2CErrorCode result = readRegisters(addr, reg, &row_data[col], 1);
        if (result == I2C_ERR_SUCCESS) {
          row_valid[col] = true;
          row_has_data = true;
        } else if (verbose_mode) {
          Serial.print(F("Error reading 0x"));
          Serial.print(reg, HEX);
          Serial.print(F(": "));
          printI2CError(result, addr);
        }
      }
    }
    
    // Print row if it has valid data
    if (row_has_data) {
      Serial.print(row_start >> 4, HEX);
      Serial.print(F("0: "));
      
      for (uint8_t col = 0; col < 16; col++) {
        uint16_t reg = row_start + col;
        
        if (reg >= start && reg <= end && row_valid[col]) {
          if (row_data[col] < 16) Serial.print(F("0"));
          Serial.print(row_data[col], HEX);
        } else {
          Serial.print(F("--"));
        }
        Serial.print(F(" "));
      }
      Serial.println();
    }
  }
}

void bulkRead(uint8_t addr, uint8_t start, uint16_t len) {
  DeviceProfile* profile = getDeviceProfile(addr);
  uint8_t chunk_size = 32; // Arduino Wire library limit
  
  if (profile && !profile->supports_bulk_read) {
    Serial.println(F("Device profile indicates no bulk read support. Using single reads..."));
    chunk_size = 1;
  }
  
  uint8_t buffer[32];
  uint16_t bytes_read = 0;
  uint16_t errors = 0;
  
  Serial.println(F("\nOffset   Data"));
  Serial.println(F("------   ----"));
  
  while (bytes_read < len) {
    uint16_t current_chunk = min(chunk_size, len - bytes_read);
    uint8_t current_reg = start + bytes_read;
    
    I2CErrorCode result = readRegisters(addr, current_reg, buffer, current_chunk);
    
    if (result == I2C_ERR_SUCCESS) {
      Serial.print(F("0x"));
      if (current_reg < 16) Serial.print(F("0"));
      Serial.print(current_reg, HEX);
      Serial.print(F("    "));
      
      for (uint16_t i = 0; i < current_chunk; i++) {
        if (buffer[i] < 16) Serial.print(F("0"));
        Serial.print(buffer[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
      
      bytes_read += current_chunk;
    } else {
      errors++;
      if (verbose_mode) {
        Serial.print(F("Error at 0x"));
        Serial.print(current_reg, HEX);
        Serial.print(F(": "));
        printI2CError(result, addr);
      }
      
      // Try single byte read as fallback
      if (chunk_size > 1) {
        chunk_size = 1;
      } else {
        bytes_read++; // Skip this byte
      }
    }
    
    if (errors > 10) {
      Serial.println(F("Too many errors, stopping bulk read"));
      break;
    }
  }
  
  Serial.print(F("\nRead "));
  Serial.print(bytes_read);
  Serial.print(F("/"));
  Serial.print(len);
  Serial.print(F(" bytes with "));
  Serial.print(errors);
  Serial.println(F(" errors"));
}

DeviceProfile* getDeviceProfile(uint8_t addr) {
  for (int i = 0; i < sizeof(known_i2c_devices) / sizeof(DeviceProfile); i++) {
    if (known_i2c_devices[i].address == addr) {
      return &known_i2c_devices[i];
    }
  }
  return nullptr;
}

void showDeviceProfile(uint8_t addr) {
  DeviceProfile* profile = getDeviceProfile(addr);
  
  Serial.print(F("Device 0x"));
  Serial.println(addr, HEX);
  
  if (profile) {
    Serial.print(F("Name: "));
    Serial.println(profile->name);
    Serial.print(F("Supports bulk read: "));
    Serial.println(profile->supports_bulk_read ? F("Yes") : F("No"));
    Serial.print(F("Requires stop bit: "));
    Serial.println(profile->requires_stop_bit ? F("Yes") : F("No"));
    Serial.print(F("Has CRC: "));
    Serial.println(profile->has_crc ? F("Yes") : F("No"));
    Serial.print(F("Max read size: "));
    Serial.println(profile->max_read_size);
    Serial.print(F("Read delay: "));
    Serial.print(profile->read_delay_ms);
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("No profile found - using defaults"));
    Serial.println(F("Supports bulk read: Unknown"));
    Serial.println(F("Requires stop bit: Yes (default)"));
    Serial.println(F("Has CRC: No"));
    Serial.println(F("Max read size: 32"));
    Serial.println(F("Read delay: 0 ms"));
  }
}

void printReadResult(uint8_t addr, uint8_t reg, uint8_t* buffer, uint16_t len) {
  Serial.print(F("0x"));
  Serial.print(addr, HEX);
  Serial.print(F(":0x"));
  Serial.print(reg, HEX);
  Serial.print(F(" = "));
  
  for (uint16_t i = 0; i < len; i++) {
    Serial.print(F("0x"));
    if (buffer[i] < 16) Serial.print(F("0"));
    Serial.print(buffer[i], HEX);
    Serial.print(F(" ("));
    Serial.print(buffer[i]);
    Serial.print(F(")"));
    if (i < len - 1) Serial.print(F(", "));
  }
  Serial.println();
}

void printI2CError(I2CErrorCode error_code, uint8_t addr) {
  Serial.print(F("I2C Error "));
  Serial.print(error_code);
  Serial.print(F(" on device 0x"));
  Serial.print(addr, HEX);
  Serial.print(F(": "));
  
  switch (error_code) {
    case I2C_ERR_SUCCESS:
      Serial.println(F("Success"));
      break;
    case I2C_ERR_DATA_TOO_LONG:
      Serial.println(F("Data too long for transmit buffer"));
      break;
    case I2C_ERR_NACK_ON_ADDRESS:
      Serial.println(F("NACK received on address"));
      break;
    case I2C_ERR_NACK_ON_DATA:
      Serial.println(F("NACK received on data"));
      break;
    case I2C_ERR_OTHER_ERROR:
      Serial.println(F("Other error"));
      break;
    case I2C_ERR_TIMEOUT:
      Serial.println(F("Timeout"));
      break;
    default:
      Serial.println(F("Unknown error"));
      break;
  }
}

uint32_t parseNumber(String str) {
  str.trim();
  if (str.startsWith("0x") || str.startsWith("0X")) {
    return strtoul(str.substring(2).c_str(), NULL, 16);
  }
  return strtoul(str.c_str(), NULL, 10);
}

void resetI2C() {
  Serial.println(F("Resetting I2C bus..."));
  
  // Reset Wire library
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(i2c_clock);
  Wire.setTimeout(I2C_TIMEOUT);
  
  Serial.println(F("I2C bus reset complete"));
}

void handleSHT41Command(String cmd) {
  int first_space = cmd.indexOf(' ');
  int second_space = cmd.indexOf(' ', first_space + 1);
  
  if (second_space == -1) {
    Serial.println(F("Usage: sht41 <addr> <command>"));
    Serial.println(F("Commands: measure, serial, reset, heater"));
    return;
  }
  
  String addr_str = cmd.substring(first_space + 1, second_space);
  String sht_cmd = cmd.substring(second_space + 1);
  uint8_t addr = parseNumber(addr_str);
  
  sht_cmd.toLowerCase();
  
  if (sht_cmd == "measure" || sht_cmd == "m") {
    sht41Measure(addr);
  } else if (sht_cmd == "serial" || sht_cmd == "s") {
    sht41ReadSerial(addr);
  } else if (sht_cmd == "reset" || sht_cmd == "r") {
    sht41SoftReset(addr);
  } else if (sht_cmd.startsWith("heater")) {
    sht41Heater(addr, sht_cmd);
  } else {
    Serial.println(F("Unknown SHT41 command. Available: measure, serial, reset, heater"));
  }
}

void sht41Measure(uint8_t addr) {
  if (verbose_mode) {
    Serial.print(F("SHT41 measurement on device 0x"));
    Serial.println(addr, HEX);
  }
  
  // Send high precision measurement command (0xFD)
  Wire.beginTransmission(addr);
  Wire.write(0xFD);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    printI2CError((I2CErrorCode)error, addr);
    return;
  }
  
  // Wait for measurement to complete (max 8.3ms for high precision)
  delay(10);
  
  // Read 6 bytes: T_high, T_low, T_CRC, RH_high, RH_low, RH_CRC
  uint8_t data[6];
  uint8_t bytes_received = Wire.requestFrom(addr, 6);
  
  if (bytes_received != 6) {
    Serial.print(F("Expected 6 bytes, received "));
    Serial.println(bytes_received);
    return;
  }
  
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }
  
  // Parse temperature
  uint16_t temp_raw = (data[0] << 8) | data[1];
  uint8_t temp_crc = data[2];
  
  // Parse humidity
  uint16_t hum_raw = (data[3] << 8) | data[4];
  uint8_t hum_crc = data[5];
  
  // Verify CRC if verbose
  if (verbose_mode) {
    bool temp_crc_ok = verifySHT41CRC(data[0], data[1], temp_crc);
    bool hum_crc_ok = verifySHT41CRC(data[3], data[4], hum_crc);
    Serial.print(F("Temperature CRC: "));
    Serial.println(temp_crc_ok ? F("OK") : F("FAIL"));
    Serial.print(F("Humidity CRC: "));
    Serial.println(hum_crc_ok ? F("OK") : F("FAIL"));
  }
  
  // Convert to physical values
  float temperature = -45.0 + 175.0 * temp_raw / 65535.0;
  float humidity = -6.0 + 125.0 * hum_raw / 65535.0;
  
  // Crop humidity to valid range
  if (humidity > 100.0) humidity = 100.0;
  if (humidity < 0.0) humidity = 0.0;
  
  Serial.print(F("SHT41 Temperature: "));
  Serial.print(temperature, 2);
  Serial.print(F(" °C, Humidity: "));
  Serial.print(humidity, 2);
  Serial.println(F(" %RH"));
  
  if (verbose_mode) {
    Serial.print(F("Raw data: "));
    for (int i = 0; i < 6; i++) {
      Serial.print(F("0x"));
      if (data[i] < 16) Serial.print(F("0"));
      Serial.print(data[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
  }
}

void sht41ReadSerial(uint8_t addr) {
  Serial.print(F("Reading SHT41 serial number from 0x"));
  Serial.println(addr, HEX);
  
  // Send read serial number command (0x89)
  Wire.beginTransmission(addr);
  Wire.write(0x89);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    printI2CError((I2CErrorCode)error, addr);
    return;
  }
  
  // Wait a bit
  delay(1);
  
  // Read 6 bytes: Serial_high, Serial_low, CRC1, Serial_high2, Serial_low2, CRC2
  uint8_t data[6];
  uint8_t bytes_received = Wire.requestFrom(addr, 6);
  
  if (bytes_received != 6) {
    Serial.print(F("Expected 6 bytes, received "));
    Serial.println(bytes_received);
    return;
  }
  
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read();
  }
  
  // Combine serial number parts
  uint32_t serial = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | 
                    ((uint32_t)data[3] << 8) | data[4];
  
  Serial.print(F("SHT41 Serial Number: 0x"));
  Serial.print(serial, HEX);
  Serial.print(F(" ("));
  Serial.print(serial);
  Serial.println(F(")"));
  
  if (verbose_mode) {
    Serial.print(F("Raw serial data: "));
    for (int i = 0; i < 6; i++) {
      Serial.print(F("0x"));
      if (data[i] < 16) Serial.print(F("0"));
      Serial.print(data[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
  }
}

void sht41SoftReset(uint8_t addr) {
  Serial.print(F("Sending soft reset to SHT41 at 0x"));
  Serial.println(addr, HEX);
  
  // Send soft reset command (0x94)
  Wire.beginTransmission(addr);
  Wire.write(0x94);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println(F("Soft reset sent successfully"));
    delay(1); // Wait for reset to complete
  } else {
    printI2CError((I2CErrorCode)error, addr);
  }
}

void sht41Heater(uint8_t addr, String cmd) {
  uint8_t heater_cmd = 0x39; // Default: 200mW for 1s
  
  if (cmd.indexOf("200") >= 0) {
    if (cmd.indexOf("0.1") >= 0) {
      heater_cmd = 0x32; // 200mW for 0.1s
    } else {
      heater_cmd = 0x39; // 200mW for 1s
    }
  } else if (cmd.indexOf("110") >= 0) {
    if (cmd.indexOf("0.1") >= 0) {
      heater_cmd = 0x24; // 110mW for 0.1s
    } else {
      heater_cmd = 0x2F; // 110mW for 1s
    }
  } else if (cmd.indexOf("20") >= 0) {
    if (cmd.indexOf("0.1") >= 0) {
      heater_cmd = 0x15; // 20mW for 0.1s
    } else {
      heater_cmd = 0x1E; // 20mW for 1s
    }
  }
  
  Serial.print(F("Activating SHT41 heater (cmd: 0x"));
  Serial.print(heater_cmd, HEX);
  Serial.println(F(")"));
  
  // Send heater command
  Wire.beginTransmission(addr);
  Wire.write(heater_cmd);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    printI2CError((I2CErrorCode)error, addr);
    return;
  }
  
  // Wait for heater cycle to complete
  if (heater_cmd == 0x32 || heater_cmd == 0x24 || heater_cmd == 0x15) {
    delay(150); // 0.1s heater + measurement time
  } else {
    delay(1100); // 1s heater + measurement time
  }
  
  // Read the measurement data after heating
  uint8_t data[6];
  uint8_t bytes_received = Wire.requestFrom(addr, 6);
  
  if (bytes_received == 6) {
    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }
    
    // Parse and display results
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];
    
    float temperature = -45.0 + 175.0 * temp_raw / 65535.0;
    float humidity = -6.0 + 125.0 * hum_raw / 65535.0;
    
    if (humidity > 100.0) humidity = 100.0;
    if (humidity < 0.0) humidity = 0.0;
    
    Serial.print(F("Post-heater reading - Temp: "));
    Serial.print(temperature, 2);
    Serial.print(F(" °C, Humidity: "));
    Serial.print(humidity, 2);
    Serial.println(F(" %RH"));
  } else {
    Serial.println(F("Failed to read post-heater measurement"));
  }
}

bool verifySHT41CRC(uint8_t data1, uint8_t data2, uint8_t expected_crc) {
  // SHT41 uses CRC-8 with polynomial 0x31, init 0xFF
  uint8_t crc = 0xFF;
  
  // Process first byte
  crc ^= data1;
  for (int i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0x31;
    } else {
      crc = crc << 1;
    }
  }
  
  // Process second byte
  crc ^= data2;
  for (int i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0x31;
    } else {
      crc = crc << 1;
    }
  }
  
  return crc == expected_crc;
}

void handleUSB2422Command(String cmd) {
  int first_space = cmd.indexOf(' ');
  int second_space = cmd.indexOf(' ', first_space + 1);
  
  if (second_space == -1) {
    Serial.println(F("Usage: usb2422 <addr> <command>"));
    Serial.println(F("Commands:"));
    Serial.println(F("  read <reg> <len>    - SMBus block read"));
    Serial.println(F("  write <reg> <data>  - SMBus block write"));
    Serial.println(F("  raw <reg>           - Raw I2C debug test"));
    Serial.println(F("  status              - Check device status"));
    return;
  }
  
  String addr_str = cmd.substring(first_space + 1, second_space);
  String usb_cmd = cmd.substring(second_space + 1);
  uint8_t addr = parseNumber(addr_str);
  
  usb_cmd.toLowerCase();
  
  if (usb_cmd.startsWith("read ")) {
    int reg_start = usb_cmd.indexOf(' ') + 1;
    int len_start = usb_cmd.indexOf(' ', reg_start) + 1;
    
    if (len_start > reg_start) {
      uint8_t reg = parseNumber(usb_cmd.substring(reg_start, len_start - 1));
      uint8_t len = parseNumber(usb_cmd.substring(len_start));
      len = constrain(len, 1, 32);
      usb2422BlockRead(addr, reg, len);
    } else {
      Serial.println(F("Usage: read <reg> <length>"));
    }
  } 
  else if (usb_cmd.startsWith("write ")) {
    // Parse write command: write <reg> <data1> <data2> ...
    int pos = usb_cmd.indexOf(' ') + 1;
    int next_pos = usb_cmd.indexOf(' ', pos);
    
    if (next_pos == -1) {
      Serial.println(F("Usage: write <reg> <data1> [data2] [data3] ..."));
      return;
    }
    
    uint8_t reg = parseNumber(usb_cmd.substring(pos, next_pos));
    pos = next_pos + 1;
    
    uint8_t data[32];
    uint8_t data_len = 0;
    
    // Parse data bytes
    while (pos < usb_cmd.length() && data_len < 32) {
      next_pos = usb_cmd.indexOf(' ', pos);
      if (next_pos == -1) next_pos = usb_cmd.length();
      
      data[data_len] = parseNumber(usb_cmd.substring(pos, next_pos));
      data_len++;
      pos = next_pos + 1;
    }
    
    if (data_len > 0) {
      usb2422BlockWrite(addr, reg, data, data_len);
    } else {
      Serial.println(F("No data specified"));
    }
  }
  else if (usb_cmd == "status") {
    // Try to read a basic register to check communication
    usb2422BlockRead(addr, 0x00, 4);
  }
  else if (usb_cmd.startsWith("raw ")) {
    // Raw I2C test - try different read approaches
    int reg_pos = usb_cmd.indexOf(' ') + 1;
    if (reg_pos > 4) {
      uint8_t reg = parseNumber(usb_cmd.substring(reg_pos));
      usb2422RawTest(addr, reg);
    }
  }
  else {
    Serial.println(F("Unknown USB2422 command"));
    Serial.println(F("Available: read, write, status"));
  }
}

void usb2422BlockRead(uint8_t addr, uint8_t reg, uint8_t len) {
  if (verbose_mode) {
    Serial.print(F("USB2422 SMBus block read: addr=0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" reg=0x"));
    Serial.print(reg, HEX);
    Serial.print(F(" len="));
    Serial.println(len);
  }
  
  uint8_t buffer[32];
  uint8_t received_len = 0;
  
  I2CErrorCode result = usb2422SMBusBlockRead(addr, reg, buffer, &received_len);
  
  if (result == I2C_ERR_SUCCESS) {
    Serial.print(F("USB2422 Block Read Success: "));
    Serial.print(received_len);
    Serial.println(F(" bytes"));
    
    // Print data in hex format
    Serial.print(F("Data: "));
    for (uint8_t i = 0; i < received_len; i++) {
      Serial.print(F("0x"));
      if (buffer[i] < 16) Serial.print(F("0"));
      Serial.print(buffer[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println();
    
    // Print as hex dump if more than 8 bytes
    if (received_len > 8) {
      Serial.println(F("Hex dump:"));
      for (uint8_t i = 0; i < received_len; i += 16) {
        Serial.print(F("0x"));
        if (reg + i < 16) Serial.print(F("0"));
        Serial.print(reg + i, HEX);
        Serial.print(F(": "));
        
        for (uint8_t j = 0; j < 16 && (i + j) < received_len; j++) {
          if (buffer[i + j] < 16) Serial.print(F("0"));
          Serial.print(buffer[i + j], HEX);
          Serial.print(F(" "));
        }
        Serial.println();
      }
    }
  } else {
    printI2CError(result, addr);
  }
}

void usb2422BlockWrite(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
  if (verbose_mode) {
    Serial.print(F("USB2422 SMBus block write: addr=0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" reg=0x"));
    Serial.print(reg, HEX);
    Serial.print(F(" len="));
    Serial.println(len);
  }
  
  I2CErrorCode result = usb2422SMBusBlockWrite(addr, reg, data, len);
  
  if (result == I2C_ERR_SUCCESS) {
    Serial.print(F("USB2422 Block Write Success: "));
    Serial.print(len);
    Serial.println(F(" bytes written"));
    
    if (verbose_mode) {
      Serial.print(F("Data written: "));
      for (uint8_t i = 0; i < len; i++) {
        Serial.print(F("0x"));
        if (data[i] < 16) Serial.print(F("0"));
        Serial.print(data[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }
  } else {
    printI2CError(result, addr);
  }
}

I2CErrorCode usb2422SMBusBlockRead(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t* received_len) {
  *received_len = 0;
  
  if (verbose_mode) {
    Serial.print(F("USB2422 Block Read Debug - Addr: 0x"));
    Serial.print(addr, HEX);
    Serial.print(F(" Reg: 0x"));
    Serial.println(reg, HEX);
  }
  
  // USB2422 requires full stop/start cycle, not repeated start
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t error = Wire.endTransmission(true); // FULL STOP - this is key!
  
  if (verbose_mode) {
    Serial.print(F("Write phase result: "));
    Serial.println(error);
  }
  
  if (error != 0) {
    return (I2CErrorCode)error;
  }
  
  // Small delay for USB2422 to process
  delay(1);
  
  // Now read the data with a fresh start
  uint8_t bytes_available = Wire.requestFrom(addr, (uint8_t)32); // Try reading up to 32 bytes
  
  if (verbose_mode) {
    Serial.print(F("Bytes available from requestFrom: "));
    Serial.println(bytes_available);
    Serial.print(F("Wire.available(): "));
    Serial.println(Wire.available());
  }
  
  // Read all available bytes
  uint8_t actual_bytes = 0;
  while (Wire.available() && actual_bytes < 32) {
    buffer[actual_bytes] = Wire.read();
    actual_bytes++;
  }
  
  if (verbose_mode) {
    Serial.print(F("Actually read "));
    Serial.print(actual_bytes);
    Serial.println(F(" bytes"));
    
    if (actual_bytes > 0) {
      Serial.print(F("First few bytes: "));
      for (uint8_t i = 0; i < min(8, actual_bytes); i++) {
        Serial.print(F("0x"));
        if (buffer[i] < 16) Serial.print(F("0"));
        Serial.print(buffer[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }
  }
  
  *received_len = actual_bytes;
  return I2C_ERR_SUCCESS;
}

I2CErrorCode usb2422SMBusBlockWrite(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
  // SMBus Block Write Protocol:
  // Start + Addr + W + Reg + ByteCount + Data[0..N] + Stop
  
  Wire.beginTransmission(addr);
  Wire.write(reg);        // Register address
  Wire.write(len);        // Byte count
  
  // Write data bytes
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  
  uint8_t error = Wire.endTransmission();
  return (I2CErrorCode)error;
}