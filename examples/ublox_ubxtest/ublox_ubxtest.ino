/*!
 * @file ubx_mode_test.ino
 * 
 * Test sketch to verify switching to UBX protocol mode
 * This example uses I2C (DDC) to communicate with a u-blox module.
 * 
 * Written by Ladyada for Adafruit Industries.
 * 
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_UBloxDDC.h"
#include "Adafruit_UBX.h"
#include "Adafruit_uBlox_typedef.h"

// Create Adafruit_UBloxDDC object with default I2C address (0x42)
Adafruit_UBloxDDC ddc;

// Create Adafruit_UBX parser using the DDC object as input stream
Adafruit_UBX ubx(ddc);

// Variables to track message statistics
uint32_t messageCount = 0;
uint32_t lastMsgTime = 0;
bool ubxModeConfirmed = false;

// Callback function for UBX messages
void ubxMessageCallback(uint8_t msgClass, uint8_t msgId, uint16_t payloadLen, uint8_t *payload) {
  messageCount++;
  lastMsgTime = millis();
  
  // First successful UBX message confirms we're in UBX mode
  if (!ubxModeConfirmed) {
    ubxModeConfirmed = true;
    Serial.println("SUCCESS: UBX mode confirmed! Receiving UBX messages.");
  }
  
  // Print message details
  Serial.print("UBX Message: Class 0x");
  if (msgClass < 0x10) Serial.print("0");
  Serial.print(msgClass, HEX);
  
  Serial.print(", ID 0x");
  if (msgId < 0x10) Serial.print("0");
  Serial.print(msgId, HEX);
  
  Serial.print(", Length: ");
  Serial.print(payloadLen);
  Serial.println(" bytes");
}

void setup() {
  // Initialize serial port for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("\n\n------------- UBX Mode Test -------------");
  Serial.println("This test will try to switch a u-blox module to UBX protocol");
  
  // Initialize GPS module
  Serial.print("Connecting to u-blox module via I2C...");
  if (ddc.begin()) {
    Serial.println(" SUCCESS!");
  } else {
    Serial.println(" FAILED!");
    Serial.println("Check connections and try again.");
    while (1); // Don't proceed if we couldn't connect to the module
  }
  
  // Show current data format (should be NMEA)
  Serial.println("\nCurrent data format (should be NMEA sentences):");
  Serial.println("------------------------------------------------");
  
  // Show some raw data
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    if (ddc.available()) {
      int c = ddc.read();
      if (c != -1) {
        Serial.write(c);
      }
    }
  }
  
  Serial.println("\n------------------------------------------------");
  
  // Initialize UBX parser and set debug mode
  ubx.begin();
  ubx.verbose_debug = 2;  // Enable basic debug output
  
  // Set the callback function for UBX messages
  ubx.setMessageCallback(ubxMessageCallback);
  
  // Now switch to UBX mode
  Serial.println("\nSwitching to UBX-only mode on DDC port...");
  
  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  
  switch (status) {
    case UBX_SEND_SUCCESS:
      Serial.println("SUCCESS: UBX configuration command acknowledged!");
      break;
    case UBX_SEND_NAK:
      Serial.println("ERROR: UBX configuration command was rejected by the module.");
      break;
    case UBX_SEND_FAIL:
      Serial.println("ERROR: Failed to send UBX configuration command.");
      break;
    case UBX_SEND_TIMEOUT:
      Serial.println("ERROR: Timeout waiting for acknowledgment.");
      break;
  }
  
  if (status != UBX_SEND_SUCCESS) {
    Serial.println("Continuing anyway to see if we receive any UBX messages...");
  }
  
  Serial.println("\nWaiting for UBX messages (timeout: 5 seconds)...");
  lastMsgTime = millis();
}

void loop() {
  // Check for UBX messages
  ubx.checkMessages();
  
  // Display statistics every second
  static uint32_t lastStatsTime = 0;
  if (millis() - lastStatsTime >= 1000) {
    lastStatsTime = millis();
    Serial.print("Status: Messages received: ");
    Serial.print(messageCount);
    
    // Check for timeout
    if (!ubxModeConfirmed && (millis() - lastMsgTime > 5000)) {
      Serial.println("\n\nTIMEOUT: No UBX messages received within 5 seconds.");
      Serial.println("Possible issues:");
      Serial.println("1. Module doesn't support UBX protocol on this port");
      Serial.println("2. Configuration command wasn't received properly");
      Serial.println("3. Module requires different configuration");
      Serial.println("\nTry using verbose_debug = 2 for more details.");
      while(1); // Stop execution
    }
    
    Serial.println();
  }
  
  // Short delay to prevent overwhelming the processor
  delay(10);
}
