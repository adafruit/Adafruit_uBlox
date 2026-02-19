/*!
 * @file ublox_ubxtest.ino
 *
 * Test sketch to verify switching to UBX protocol mode
 * This example uses I2C (DDC) to communicate with a u-blox module.
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_UBX.h"
#include "Adafruit_UBloxDDC.h"
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
void ubxMessageCallback(uint8_t msgClass, uint8_t msgId, uint16_t payloadLen,
                        uint8_t* payload) {
  messageCount++;
  lastMsgTime = millis();

  // First successful UBX message confirms we're in UBX mode
  if (!ubxModeConfirmed) {
    ubxModeConfirmed = true;
    Serial.println(F("SUCCESS: UBX mode confirmed! Receiving UBX messages."));
  }

  // Print message details
  Serial.print(F("UBX Message: Class 0x"));
  if (msgClass < 0x10)
    Serial.print(F("0"));
  Serial.print(msgClass, HEX);

  Serial.print(F(", ID 0x"));
  if (msgId < 0x10)
    Serial.print(F("0"));
  Serial.print(msgId, HEX);

  Serial.print(F(", Length: "));
  Serial.print(payloadLen);
  Serial.println(F(" bytes"));
}

void setup() {
  // Initialize serial port for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println(F("\n\n------------- UBX Mode Test -------------"));
  Serial.println(
      F("This test will try to switch a u-blox module to UBX protocol"));

  // Initialize GPS module
  Serial.print(F("Connecting to u-blox module via I2C..."));
  if (ddc.begin()) {
    Serial.println(F(" SUCCESS!"));
  } else {
    Serial.println(F(" FAILED!"));
    Serial.println(F("Check connections and try again."));
    while (1)
      ; // Don't proceed if we couldn't connect to the module
  }

  // Show current data format (should be NMEA)
  Serial.println(F("\nCurrent data format (should be NMEA sentences):"));
  Serial.println(F("------------------------------------------------"));

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

  Serial.println(F("\n------------------------------------------------"));

  // Initialize UBX parser and set debug mode
  ubx.begin();
  ubx.verbose_debug = 2; // Enable basic debug output

  // Set the callback function for UBX messages
  ubx.setMessageCallback(ubxMessageCallback);

  // Now switch to UBX mode
  Serial.println(F("\nSwitching to UBX-only mode on DDC port..."));

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  switch (status) {
    case UBX_SEND_SUCCESS:
      Serial.println(F("SUCCESS: UBX configuration command acknowledged!"));
      break;
    case UBX_SEND_NAK:
      Serial.println(
          F("ERROR: UBX configuration command was rejected by the module."));
      break;
    case UBX_SEND_FAIL:
      Serial.println(F("ERROR: Failed to send UBX configuration command."));
      break;
    case UBX_SEND_TIMEOUT:
      Serial.println(F("ERROR: Timeout waiting for acknowledgment."));
      break;
  }

  if (status != UBX_SEND_SUCCESS) {
    Serial.println(
        F("Continuing anyway to see if we receive any UBX messages..."));
  }

  Serial.println(F("\nWaiting for UBX messages (timeout: 5 seconds)..."));
  lastMsgTime = millis();
}

void loop() {
  // Check for UBX messages
  ubx.checkMessages();

  // Display statistics every second
  static uint32_t lastStatsTime = 0;
  if (millis() - lastStatsTime >= 1000) {
    lastStatsTime = millis();
    Serial.print(F("Status: Messages received: "));
    Serial.print(messageCount);

    // Check for timeout
    if (!ubxModeConfirmed && (millis() - lastMsgTime > 5000)) {
      Serial.println(
          F("\n\nTIMEOUT: No UBX messages received within 5 seconds."));
      Serial.println(F("Possible issues:"));
      Serial.println(F("1. Module doesn't support UBX protocol on this port"));
      Serial.println(F("2. Configuration command wasn't received properly"));
      Serial.println(F("3. Module requires different configuration"));
      Serial.println(F("\nTry using verbose_debug = 2 for more details."));
      while (1)
        ; // Stop execution
    }

    Serial.println();
  }

  // Short delay to prevent overwhelming the processor
  delay(10);
}
