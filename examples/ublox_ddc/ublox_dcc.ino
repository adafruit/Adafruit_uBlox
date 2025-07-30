/*!
 * @file ublox_ddc_example.ino
 * 
 * Example sketch demonstrating the use of the Adafruit_UBloxDDC library
 * with u-blox GPS/RTK modules over I2C (DDC) interface.
 * 
 * This example simply streams all raw bytes from the GPS module to the
 * Serial port so you can see the NMEA sentences in their original format.
 * 
 * Written by Ladyada for Adafruit Industries.
 * 
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_UBloxDDC.h"

// Create Adafruit_UBloxDDC object with default I2C address (0x42)
Adafruit_UBloxDDC gps;

// Buffer for reading chunks of data
const size_t BUFFER_SIZE = 64;
uint8_t buffer[BUFFER_SIZE];

void setup() {
  // Initialize serial port for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("Adafruit UBlox DDC Raw NMEA Stream Example");
  
  // Initialize GPS module
  if (gps.begin()) {
    Serial.println("GPS module found!");
  } else {
    Serial.println("Failed to connect to GPS module!");
    while (1); // Don't proceed if we couldn't connect to the module
  }
  
  Serial.println("Streaming raw data from GPS module...");
  Serial.println("------------------------------------");
}

void loop() {
  // Check how many bytes are available
  int bytesAvailable = gps.available();
  
  if (bytesAvailable > 0) {
    // Read up to BUFFER_SIZE bytes at a time
    size_t bytesToRead = min(bytesAvailable, (int)BUFFER_SIZE);
    size_t bytesRead = gps.readBytes(buffer, bytesToRead);
    
    // Stream the bytes directly to Serial
    // This will show raw NMEA sentences
    for (size_t i = 0; i < bytesRead; i++) {
      Serial.write(buffer[i]);
    }
  }
  
  // Short delay to prevent overwhelming the serial monitor
  delay(10);
}
