/*
 * Connect the pins of the EEPROM 24C64 to the ESP32 as follows:
 * 24C64  |  ESP32
 * VCC(8) -> 3.3V
 * GND(4) -> GND
 * SCL(6) -> GPIO 22(or any GPIO pin that can be used for I2C)
 * SDA(5) -> GPIO 21(or any GPIO pin that can be used for I2C)
 * WP(7)  -> GND (to turn off Write Protect mode, allowing data to be written)
 * Optional connection to set the I2C address 0x52 
 * A0(1) -> GND
 * A1(2) -> 3V3(D2)
 * A2(3) -> GND
 */


#include <Wire.h>
#include "SPIFFS.h"

#define EEPROM_BASE_ADDRESS 0x52
#define PAGE_SIZE 32

// Pin definitions for EEPROM address pins
#define A1_PIN 2

// Define buttom pin.

#define buttonPin 4

void setup() {
  Serial.begin(115200);
  
  // Initialize address pins as outputs          
  pinMode(A1_PIN, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  
  // set the EEPROM address pins
 
  digitalWrite(A1_PIN, HIGH);
  
  // Initialize I2C
  Wire.begin();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    delay(1000);
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

    Serial.println("Start the file recording process");
    
    // Write data to EEPROM
    while (file.available()) {
      size_t bytesRead = file.read(buffer, PAGE_SIZE);
      writeEEPROM(EEPROM_BASE_ADDRESS, address, buffer, bytesRead);
      address += bytesRead;
      delay(10); // Ensure the write cycle completes
    }

    file.close();
    Serial.println("File write successfull");
    }
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
