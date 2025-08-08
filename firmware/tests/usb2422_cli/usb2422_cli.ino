#include <Wire.h>

// USB2422 I2C address (can be changed via CLI)
uint8_t deviceAddress = 0x2C;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Wait for serial port to connect
    }
    
    // Initialize I2C with default pins and clock
    Wire.begin();
    Wire.setClock(100000); // 100kHz standard mode
    
    Serial.println("USB2422 I2C CLI Interface");
    Serial.println("========================");
    printHelp();
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        processCommand(command);
    }
}

void printHelp() {
    Serial.println("\nAvailable commands:");
    Serial.println("help                     - Show this help");
    Serial.println("addr <hex>               - Set device address (e.g., addr 2D)");
    Serial.println("scan                     - Scan for I2C devices");
    Serial.println("bw <cmd> <data...>       - Block write (e.g., bw 10 01 02 03)");
    Serial.println("br <cmd> <maxlen>        - Block read (e.g., br 11 10)");
    Serial.println("wb <cmd> <data>          - Write single byte (e.g., wb 20 AB)");
    Serial.println("rb <cmd>                 - Read single byte (e.g., rb 21)");
    Serial.println("status                   - Show current settings");
    Serial.println("\nNotes:");
    Serial.println("- All values in hexadecimal (without 0x prefix)");
    Serial.println("- Block operations limited to 1-32 bytes");
    Serial.println("- Current device address: 0x" + String(deviceAddress, HEX));
    Serial.println();
}

void processCommand(String cmd) {
    cmd.toUpperCase();
    
    if (cmd == "HELP" || cmd == "?") {
        printHelp();
    }
    else if (cmd.startsWith("ADDR ")) {
        setAddress(cmd);
    }
    else if (cmd == "SCAN") {
        scanDevices();
    }
    else if (cmd.startsWith("BW ")) {
        blockWrite(cmd);
    }
    else if (cmd.startsWith("BR ")) {
        blockRead(cmd);
    }
    else if (cmd.startsWith("WB ")) {
        writeByte(cmd);
    }
    else if (cmd.startsWith("RB ")) {
        readByte(cmd);
    }
    else if (cmd == "STATUS") {
        showStatus();
    }
    else if (cmd == "") {
        // Empty command, do nothing
    }
    else {
        Serial.println("Unknown command. Type 'help' for available commands.");
    }
    
    Serial.print("> ");
}

void setAddress(String cmd) {
    String addrStr = cmd.substring(5);
    addrStr.trim();
    
    if (addrStr.length() == 0) {
        Serial.println("Error: Address required");
        return;
    }
    
    long addr = strtol(addrStr.c_str(), NULL, 16);
    if (addr < 0 || addr > 0x7F) {
        Serial.println("Error: Invalid address (must be 00-7F)");
        return;
    }
    
    deviceAddress = (uint8_t)addr;
    Serial.println("Device address set to 0x" + String(deviceAddress, HEX));
}

void scanDevices() {
    Serial.println("Scanning I2C bus...");
    int found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found device at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found++;
        }
    }
    
    if (found == 0) {
        Serial.println("No I2C devices found");
    } else {
        Serial.println("Scan complete. Found " + String(found) + " device(s)");
    }
}

void blockWrite(String cmd) {
    // Parse: BW <cmd> <data1> <data2> ...
    String params = cmd.substring(3);
    params.trim();
    
    if (params.length() == 0) {
        Serial.println("Error: Command code and data required");
        return;
    }
    
    // Split parameters
    String tokens[34]; // Max 32 data bytes + command + length
    int tokenCount = 0;
    int start = 0;
    
    while (start < params.length() && tokenCount < 34) {
        int space = params.indexOf(' ', start);
        if (space == -1) {
            tokens[tokenCount++] = params.substring(start);
            break;
        }
        tokens[tokenCount++] = params.substring(start, space);
        start = space + 1;
        while (start < params.length() && params.charAt(start) == ' ') start++;
    }
    
    if (tokenCount < 2) {
        Serial.println("Error: Command code and at least one data byte required");
        return;
    }
    
    // Parse command code
    long cmdCode = strtol(tokens[0].c_str(), NULL, 16);
    if (cmdCode < 0 || cmdCode > 0xFF) {
        Serial.println("Error: Invalid command code");
        return;
    }
    
    // Parse data bytes
    uint8_t data[32];
    uint8_t dataLength = tokenCount - 1;
    
    if (dataLength > 32) {
        Serial.println("Error: Maximum 32 data bytes allowed");
        return;
    }
    
    for (int i = 0; i < dataLength; i++) {
        long val = strtol(tokens[i + 1].c_str(), NULL, 16);
        if (val < 0 || val > 0xFF) {
            Serial.println("Error: Invalid data byte at position " + String(i + 1));
            return;
        }
        data[i] = (uint8_t)val;
    }
    
    // Execute block write
    if (usb2422_blockWrite((uint8_t)cmdCode, data, dataLength)) {
        Serial.println("Block write successful (" + String(dataLength) + " bytes)");
    } else {
        Serial.println("Block write failed");
    }
}

void blockRead(String cmd) {
    // Parse: BR <cmd> <maxlen>
    String params = cmd.substring(3);
    params.trim();
    
    int space = params.indexOf(' ');
    if (space == -1) {
        Serial.println("Error: Command code and max length required");
        return;
    }
    
    String cmdStr = params.substring(0, space);
    String lenStr = params.substring(space + 1);
    
    long cmdCode = strtol(cmdStr.c_str(), NULL, 16);
    long maxLen = strtol(lenStr.c_str(), NULL, 16);
    
    if (cmdCode < 0 || cmdCode > 0xFF) {
        Serial.println("Error: Invalid command code");
        return;
    }
    
    if (maxLen < 1 || maxLen > 32) {
        Serial.println("Error: Max length must be 1-32");
        return;
    }
    
    // Execute block read
    uint8_t data[32];
    uint8_t actualLength;
    
    if (usb2422_blockRead((uint8_t)cmdCode, data, (uint8_t)maxLen, &actualLength)) {
        Serial.print("Block read successful, received " + String(actualLength) + " bytes: ");
        for (uint8_t i = 0; i < actualLength; i++) {
            if (data[i] < 0x10) Serial.print("0");
            Serial.print(data[i], HEX);
            if (i < actualLength - 1) Serial.print(" ");
        }
        Serial.println();
    } else {
        Serial.println("Block read failed");
    }
}

void writeByte(String cmd) {
    // Parse: WB <cmd> <data>
    String params = cmd.substring(3);
    params.trim();
    
    int space = params.indexOf(' ');
    if (space == -1) {
        Serial.println("Error: Command code and data byte required");
        return;
    }
    
    String cmdStr = params.substring(0, space);
    String dataStr = params.substring(space + 1);
    
    long cmdCode = strtol(cmdStr.c_str(), NULL, 16);
    long data = strtol(dataStr.c_str(), NULL, 16);
    
    if (cmdCode < 0 || cmdCode > 0xFF || data < 0 || data > 0xFF) {
        Serial.println("Error: Invalid command code or data");
        return;
    }
    
    if (usb2422_writeByte((uint8_t)cmdCode, (uint8_t)data)) {
        Serial.println("Write byte successful");
    } else {
        Serial.println("Write byte failed");
    }
}

void readByte(String cmd) {
    // Parse: RB <cmd>
    String cmdStr = cmd.substring(3);
    cmdStr.trim();
    
    if (cmdStr.length() == 0) {
        Serial.println("Error: Command code required");
        return;
    }
    
    long cmdCode = strtol(cmdStr.c_str(), NULL, 16);
    if (cmdCode < 0 || cmdCode > 0xFF) {
        Serial.println("Error: Invalid command code");
        return;
    }
    
    uint8_t data;
    if (usb2422_readByte((uint8_t)cmdCode, &data)) {
        Serial.print("Read byte successful: ");
        if (data < 0x10) Serial.print("0");
        Serial.println(data, HEX);
    } else {
        Serial.println("Read byte failed");
    }
}

void showStatus() {
    Serial.println("Current Settings:");
    Serial.println("- Device Address: 0x" + String(deviceAddress, HEX));
    Serial.println("- I2C Clock: 100kHz");
    
    // Test device communication
    Wire.beginTransmission(deviceAddress);
    if (Wire.endTransmission() == 0) {
        Serial.println("- Device Status: Connected");
    } else {
        Serial.println("- Device Status: Not responding");
    }
}

// Block write function
bool usb2422_blockWrite(uint8_t commandCode, uint8_t* data, uint8_t length) {
    if (length == 0 || length > 32) {
        return false;
    }
    
    Wire.beginTransmission(deviceAddress);
    Wire.write(commandCode);
    Wire.write(length);
    
    for (uint8_t i = 0; i < length; i++) {
        Wire.write(data[i]);
    }
    
    uint8_t result = Wire.endTransmission();
    return (result == 0);
}

// Block read function
bool usb2422_blockRead(uint8_t commandCode, uint8_t* data, uint8_t maxLength, uint8_t* actualLength) {
    if (maxLength == 0 || maxLength > 32) {
        return false;
    }
    
    Wire.beginTransmission(deviceAddress);
    Wire.write(commandCode);
    uint8_t result = Wire.endTransmission(false);
    
    if (result != 0) {
        return false;
    }
    
    uint8_t bytesReceived = Wire.requestFrom(deviceAddress, (uint8_t)(maxLength + 1));
    
    if (bytesReceived < 1) {
        return false;
    }
    
    uint8_t byteCount = Wire.read();
    *actualLength = byteCount;
    bytesReceived--;
    
    if (byteCount == 0 || byteCount > maxLength || byteCount > bytesReceived) {
        while (Wire.available()) {
            Wire.read();
        }
        return false;
    }
    
    for (uint8_t i = 0; i < byteCount && Wire.available(); i++) {
        data[i] = Wire.read();
    }
    
    while (Wire.available()) {
        Wire.read();
    }
    
    return true;
}

// Single byte write
bool usb2422_writeByte(uint8_t commandCode, uint8_t value) {
    return usb2422_blockWrite(commandCode, &value, 1);
}

// Single byte read
bool usb2422_readByte(uint8_t commandCode, uint8_t* value) {
    uint8_t length;
    return usb2422_blockRead(commandCode, value, 1, &length) && (length == 1);
}