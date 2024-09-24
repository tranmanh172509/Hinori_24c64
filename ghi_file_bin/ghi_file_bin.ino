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
 * A1(2) -> 3V3
 * A2(3) -> GND
 * GND -> Buttom pin 1 | Buttom pin 2 -> GPIO 4
 * GND -> Buzzer pin 1 | Buzzer pin 2 -> GPIO 12
 */


#include <Wire.h>
#include "SPIFFS.h"

#define EEPROM_BASE_ADDRESS 0x52
#define PAGE_SIZE 32

// Define buttom pin.
#define buttonPin 4

// Define Buzzer pin.
int buzzerPin = 12;
void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  
  // Initialize I2C
  Wire.begin();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }
}

void loop() {
  //Serial.println(digitalRead(buttonPin));
  if (digitalRead(buttonPin) == LOW) {
    int foundDevice = 0;
    for (byte address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);

      // Kiểm tra nếu địa chỉ là 0x52
        if (address == 0x52) {
          Serial.println("EEPROM 24C64 detected at 0x52.");
          foundDevice = 1;
          break;
        }
      }
    }
    if (foundDevice == 0) {
      Serial.println("Error: EEPROM 24C64 not found at address 0x52.");
      alarmSound();  // Play alarm sound
      return;
    }
    // Open the binary file from SPIFFS
    File file = SPIFFS.open("/test.bin", "r");
    if (!file) {
       Serial.println("Failed to open file for reading");
       alarmSound(); // Play alarm sound
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
    successSound();
    }
}

// Function alarm sound
void alarmSound() {
  for (int freq = 500; freq <= 2000; freq += 50) {
    tone(buzzerPin, freq);
    delay(50);
  }

  for (int freq = 2000; freq >= 500; freq -= 50) {
    tone(buzzerPin, freq);
    delay(50);
  }

  noTone(buzzerPin);
}

// Play success sound
void successSound() {
  // Play a series of increasing tones for a "success" sound
  tone(buzzerPin, 500);  // Play sound at 500 Hz
  delay(200);
  tone(buzzerPin, 700);  // Play sound at 700 Hz
  delay(200);
  tone(buzzerPin, 900);  // Play sound at 900 Hz
  delay(200);

  noTone(buzzerPin);  // Turn off sound
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
