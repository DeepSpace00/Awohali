#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  while(!Serial) delay(10);

  Serial.println("I2C Register Tool Ready");
  showHelp();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
}

void processCommand(String cmd) {
  cmd.toLowerCase();
  
  if (cmd.startsWith("read ")) {
    handleRead(cmd);
  } else if (cmd.startsWith("write ")) {
    handleWrite(cmd);
  } else if (cmd.startsWith("dump ")) {
    handleDump(cmd);
  } else if (cmd.equals("scan")) {
    handleScan();
  } else if (cmd.equals("help")) {
    showHelp();
  } else {
    Serial.print("\r\n");
    Serial.println("Invalid command. Type 'help' for command list");
  }
}

void handleRead(String cmd) {
  int addr, reg, count = 1;
  if (parseReadCommand(cmd, &addr, &reg, &count)) {
    Serial.print("\r\n");
    Serial.print("Reading ");
    Serial.print(count);
    Serial.print(" byte");
    if (count > 1) Serial.print("s");
    Serial.print(" from 0x");
    Serial.print(addr, HEX);
    Serial.print(" starting at reg 0x");
    Serial.print(reg, HEX);
    Serial.print(": ");
    
    for (int i = 0; i < count; i++) {
      uint8_t value = readRegister(addr, reg + i);
      
      if (i > 0) Serial.print(", ");
      Serial.print("0x");
      if (value < 0x10) Serial.print("0");
      Serial.print(value, HEX);
    }
    Serial.println();
  }
}

void handleWrite(String cmd) {
  int addr, reg;
  uint8_t values[32];  // Max 32 bytes
  int valueCount = 0;
  
  if (parseWriteCommand(cmd, &addr, &reg, values, &valueCount)) {
    Serial.print("\r\n");
    Serial.print("Writing ");
    Serial.print(valueCount);
    Serial.print(" byte");
    if (valueCount > 1) Serial.print("s");
    Serial.print(" to 0x");
    Serial.print(addr, HEX);
    Serial.print(" starting at reg 0x");
    Serial.print(reg, HEX);
    Serial.print(": ");
    
    // Display what we're writing
    for (int i = 0; i < valueCount; i++) {
      if (i > 0) Serial.print(", ");
      Serial.print("0x");
      if (values[i] < 0x10) Serial.print("0");
      Serial.print(values[i], HEX);
    }
    Serial.println();
    
    // Write the values
    writeMultipleRegisters(addr, reg, values, valueCount);
    Serial.println("Write complete");
  }
}

void handleDump(String cmd) {
  int addr;
  if (parseDumpCommand(cmd, &addr)) {
    Serial.print("\r\n");
    Serial.print("Dumping registers from device 0x");
    Serial.print(addr, HEX);
    Serial.println(":");
    
    for (int reg = 0; reg <= 0xFF; reg++) {
      uint8_t value = readRegister(addr, reg);
      
      if (reg % 16 == 0) {
        Serial.println();
        Serial.print("0x");
        if (reg < 0x10) Serial.print("0");
        Serial.print(reg, HEX);
        Serial.print(": ");
      }
      
      if (value < 0x10) Serial.print("0");
      Serial.print(value, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void handleScan() {
  Serial.print("\r\n");
  Serial.println("Scanning I2C bus...");
  
  bool foundDevices = false;
  String deviceList = "";
  
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      if (foundDevices) {
        deviceList += ", ";
      }
      deviceList += "0x";
      if (addr < 0x10) deviceList += "0";
      deviceList += String(addr, HEX);
      foundDevices = true;
    }
  }
  
  if (foundDevices) {
    Serial.print("Devices found: ");
    Serial.println(deviceList);
  } else {
    Serial.println("No devices found");
  }
}

void showHelp() {
  Serial.print("\r\n");
  Serial.println("Commands:");
  Serial.println("  read <addr> <reg> [count]    - Read register(s) (count defaults to 1)");
  Serial.println("  write <addr> <reg> <val>     - Write single value to register");
  Serial.println("  write <addr> <reg> [v1,v2..] - Write multiple values starting at register");
  Serial.println("  dump <addr>                  - Dump all registers (0x00-0xFF) of device");
  Serial.println("  scan                         - Scan I2C bus for devices");
  Serial.println("  help                         - Show this help message");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  read 0x48 0x00");
  Serial.println("  read 0x48 0x00 4");
  Serial.println("  write 0x48 0x01 0xFF");
  Serial.println("  write 0x48 0x01 [0x00,0x01,0x02]");
  Serial.println("  dump 0x48");
  Serial.println("  scan");
}

bool parseReadCommand(String cmd, int* addr, int* reg, int* count) {
  // Format: "read <addr> <reg> [count]"
  int firstSpace = cmd.indexOf(' ', 5);
  if (firstSpace == -1) {
    Serial.print("\r\n");
    Serial.println("Usage: read <addr> <reg> [count]");
    return false;
  }
  
  int secondSpace = cmd.indexOf(' ', firstSpace + 1);
  
  String addrStr = cmd.substring(5, firstSpace);
  String regStr;
  String countStr = "";
  
  if (secondSpace == -1) {
    // No count provided, use default
    regStr = cmd.substring(firstSpace + 1);
    *count = 1;
  } else {
    regStr = cmd.substring(firstSpace + 1, secondSpace);
    countStr = cmd.substring(secondSpace + 1);
    *count = parseNumber(countStr);
  }
  
  *addr = parseNumber(addrStr);
  *reg = parseNumber(regStr);
  
  if (*addr < 0 || *reg < 0 || *count <= 0) {
    Serial.print("\r\n");
    Serial.println("Invalid address, register, or count");
    return false;
  }
  
  if (*count > 32) {
    Serial.print("\r\n");
    Serial.println("Maximum 32 bytes can be read at once");
    return false;
  }
  
  return true;
}

bool parseWriteCommand(String cmd, int* addr, int* reg, uint8_t* values, int* valueCount) {
  // Format: "write <addr> <reg> <val>" or "write <addr> <reg> [val1,val2,...]"
  int firstSpace = cmd.indexOf(' ', 6);
  int secondSpace = cmd.indexOf(' ', firstSpace + 1);
  
  if (firstSpace == -1 || secondSpace == -1) {
    Serial.print("\r\n");
    Serial.println("Usage: write <addr> <reg> <val> or write <addr> <reg> [val1,val2,...]");
    return false;
  }
  
  String addrStr = cmd.substring(6, firstSpace);
  String regStr = cmd.substring(firstSpace + 1, secondSpace);
  String valStr = cmd.substring(secondSpace + 1);
  
  *addr = parseNumber(addrStr);
  *reg = parseNumber(regStr);
  
  if (*addr < 0 || *reg < 0) {
    Serial.println("Invalid address or register");
    return false;
  }
  
  valStr.trim();
  
  // Check if it's an array format [val1,val2,...]
  if (valStr.startsWith("[") && valStr.endsWith("]")) {
    // Parse array
    valStr = valStr.substring(1, valStr.length() - 1); // Remove brackets
    valStr.replace(" ", ""); // Remove spaces
    
    *valueCount = 0;
    int startPos = 0;
    int commaPos = 0;
    
    while (commaPos >= 0 && *valueCount < 32) {
      commaPos = valStr.indexOf(',', startPos);
      String valueStr;
      
      if (commaPos >= 0) {
        valueStr = valStr.substring(startPos, commaPos);
        startPos = commaPos + 1;
      } else {
        valueStr = valStr.substring(startPos);
      }
      
      if (valueStr.length() > 0) {
        int val = parseNumber(valueStr);
        if (val < 0 || val > 255) {
          Serial.print("\r\n");
          Serial.println("Invalid value in array (must be 0-255)");
          return false;
        }
        values[*valueCount] = (uint8_t)val;
        (*valueCount)++;
      }
    }
    
    if (*valueCount == 0) {
      Serial.print("\r\n");
      Serial.println("Empty array provided");
      return false;
    }
    
  } else {
    // Single value
    int val = parseNumber(valStr);
    if (val < 0 || val > 255) {
      Serial.print("\r\n");
      Serial.println("Invalid value (must be 0-255)");
      return false;
    }
    values[0] = (uint8_t)val;
    *valueCount = 1;
  }
  
  return true;
}

bool parseDumpCommand(String cmd, int* addr) {
  // Format: "dump <addr>"
  if (cmd.length() < 6) {
    Serial.print("\r\n");
    Serial.println("Usage: dump <addr>");
    return false;
  }
  
  String addrStr = cmd.substring(5);
  *addr = parseNumber(addrStr);
  
  if (*addr < 0) {
    Serial.print("\r\n");
    Serial.println("Invalid address");
    return false;
  }
  
  return true;
}

int parseNumber(String str) {
  str.trim();
  
  // Hex format (0x or 0X)
  if (str.startsWith("0x") || str.startsWith("0X")) {
    return strtol(str.c_str(), NULL, 16);
  }
  
  // Binary format (0b or 0B)
  if (str.startsWith("0b") || str.startsWith("0B")) {
    return strtol(str.substring(2).c_str(), NULL, 2);
  }
  
  // Decimal format
  return str.toInt();
}

uint8_t readRegister(uint8_t deviceAddr, uint8_t regAddr) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("\r\n");
    Serial.print("I2C Error ");
    Serial.print(error);
    Serial.println(" on write");
    return 0;
  }
  
  Wire.requestFrom(deviceAddr, (uint8_t)1);
  
  if (Wire.available()) {
    return Wire.read();
  } else {
    Serial.print("\r\n");
    Serial.println("No data received");
    return 0;
  }
}

void writeMultipleRegisters(uint8_t deviceAddr, uint8_t regAddr, uint8_t* values, int count) {
  // Write each register sequentially
  for (int i = 0; i < count; i++) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(regAddr + i);
    Wire.write(values[i]);
    uint8_t error = Wire.endTransmission();
    
    if (error != 0) {
      Serial.print("\r\n");
      Serial.print("I2C Error ");
      Serial.print(error);
      Serial.print(" on write to register 0x");
      Serial.println(regAddr + i, HEX);
      return;
    }
    
    delay(5); // Small delay between writes for some devices
  }
}

void writeRegister(uint8_t deviceAddr, uint8_t regAddr, uint8_t value) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  Wire.write(value);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("\r\n");
    Serial.print("I2C Error ");
    Serial.println(error);
  }
}