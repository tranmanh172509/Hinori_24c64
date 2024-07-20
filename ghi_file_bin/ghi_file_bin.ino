#include <Wire.h>
#include "SPIFFS.h"

#define EEPROM_BASE_ADDRESS 0x50
#define PAGE_SIZE 32

// Pin definitions for EEPROM address pins
#define A0_PIN 25
#define A1_PIN 26
#define A2_PIN 27

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  // Initialize address pins as outputs
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);

  // Open the binary file from SPIFFS
  File file = SPIFFS.open("/test.bin", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Get the file size
  size_t fileSize = file.size();
  Serial.print("File size: ");
  Serial.println(fileSize);

  uint8_t buffer[PAGE_SIZE];
  uint16_t address = 0;

  // Write data to EEPROM
  while (file.available()) {
    size_t bytesRead = file.read(buffer, PAGE_SIZE);
    writeEEPROM(EEPROM_BASE_ADDRESS, address, buffer, bytesRead);
    address += bytesRead;
    delay(10); // Ensure the write cycle completes
  }

  file.close();
  Serial.println("File write completed");
}

void loop() {
  // Nothing to do here
}

// Function to set the EEPROM address pins
void setEEPROMAddress(uint8_t address) {
  digitalWrite(A0_PIN, (address & 0x01) ? HIGH : LOW);
  digitalWrite(A1_PIN, (address & 0x02) ? HIGH : LOW);
  digitalWrite(A2_PIN, (address & 0x04) ? HIGH : LOW);
}

// Function to write data to EEPROM
void writeEEPROM(uint8_t i2cAddress, uint16_t memoryAddress, uint8_t* data, size_t length) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((memoryAddress >> 8) & 0xFF); // MSB
  Wire.write(memoryAddress & 0xFF);        // LSB

  for (size_t i = 0; i < length; i++) {
    Wire.write(data[i]);
  }

  Wire.endTransmission();
  delay(5); // Wait for write cycle to complete
}

// Function to read data from EEPROM
void readEEPROM(uint8_t i2cAddress, uint16_t memoryAddress, uint8_t* buffer, size_t length) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((memoryAddress >> 8) & 0xFF); // MSB
  Wire.write(memoryAddress & 0xFF);        // LSB
  Wire.endTransmission();

  Wire.requestFrom(i2cAddress, length);

  for (size_t i = 0; i < length; i++) {
    if (Wire.available()) {
      buffer[i] = Wire.read();
    }
  }
}
