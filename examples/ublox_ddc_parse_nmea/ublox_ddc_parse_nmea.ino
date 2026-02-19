/*!
 * @file ublox_ddc_parse_nmea.ino
 *
 * Example sketch demonstrating parsing NMEA sentences from a u-blox GPS/RTK
 * module using the Adafruit_GPS and Adafruit_UBX libraries.
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * MIT license, all text above must be included in any redistribution
 */

#include <Adafruit_GPS.h>
#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

// Adafruit_UBloxDDC object with default I2C address (0x42)
Adafruit_UBloxDDC gps;
// Create Adafruit_UBX parser using the DDC object as input stream
Adafruit_UBX ubx(gps);
// Create Adafruit_GPS object for use as a NMEA parser only
Adafruit_GPS NMEA;

// Buffer to store NMEA sentences from the module (may be partial sentences)
const size_t SZ_NMEA_BUFFER = 128;
uint8_t buffer[SZ_NMEA_BUFFER];
uint32_t timer = millis();

/*!
 *  @brief  Sets the NMEA output to only RMC and GGA sentences and waits for an
 * acknowledgment. All other common NMEA sentences are disabled to increase
 * loop() performance by decreasing the amount of messages output by the UBlox
 * module.
 *  @return Status indicating success, failure, or timeout
 */
UBXSendStatus SetNMEAOutputRMCGGAOnly() {
  UBXSendStatus status;
  // Disable all common NMEA messages
  status = ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_MSG,
                                  UBX_CFG_MSG_NMEA_GLL_DISABLE,
                                  sizeof(UBX_CFG_MSG_NMEA_GLL_DISABLE));
  if (status != UBX_SEND_SUCCESS)
    return status;
  status = ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_MSG,
                                  UBX_CFG_MSG_NMEA_GSA_DISABLE,
                                  sizeof(UBX_CFG_MSG_NMEA_GSA_DISABLE));
  if (status != UBX_SEND_SUCCESS)
    return status;
  status = ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_MSG,
                                  UBX_CFG_MSG_NMEA_GSV_DISABLE,
                                  sizeof(UBX_CFG_MSG_NMEA_GSV_DISABLE));
  if (status != UBX_SEND_SUCCESS)
    return status;
  status = ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_MSG,
                                  UBX_CFG_MSG_NMEA_VTG_DISABLE,
                                  sizeof(UBX_CFG_MSG_NMEA_VTG_DISABLE));
  if (status != UBX_SEND_SUCCESS)
    return status;
  // Enable only GGA and RMC messages
  status = ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_MSG,
                                  UBX_CFG_MSG_NMEA_GGA_ENABLE,
                                  sizeof(UBX_CFG_MSG_NMEA_GGA_ENABLE));
  if (status != UBX_SEND_SUCCESS)
    return status;
  status = ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_MSG,
                                  UBX_CFG_MSG_NMEA_RMC_ENABLE,
                                  sizeof(UBX_CFG_MSG_NMEA_RMC_ENABLE));
  return status;
}

void setup() {
  // Initialize serial port for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Initialize GPS module
  if (!gps.begin()) {
    Serial.println(F("Failed to initialize GPS module!"));
    while (1)
      ; // Halt if initialization fails
  }
  Serial.println(F("GPS module initialized successfully!"));

  // Initialize the u-blox parser
  if (!ubx.begin()) {
    Serial.println(F("Failed to initialize u-blox parser!"));
    while (1)
      ; // Halt if initialization fails
  }
  Serial.println(F("u-blox parser initialized successfully!"));
  ubx.verbose_debug = 3; // Set debug level to verbose

  // Set NMEA output on DDC to RMC and GGA sentences only
  UBXSendStatus status = SetNMEAOutputRMCGGAOnly();
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("Failed to configure NMEA output [status code: "));
    Serial.print(status);
    Serial.println(F("]"));
    while (1)
      ; // Halt if setting NMEA output fails
  }
}

void loop() {
  static char nmeaSentenceBuf[SZ_NMEA_BUFFER];
  static size_t nmeaSentenceIdx = 0;

  // Check how many bytes are available
  int bytesAvailable = gps.available();

  if (bytesAvailable > 0) {
    // Read up to SZ_NMEA_BUFFER bytes at a time
    size_t bytesToRead = min(bytesAvailable, (int)SZ_NMEA_BUFFER);
    size_t bytesRead = gps.readBytes(buffer, bytesToRead);

    // Build NMEA sentences and parse when complete
    for (size_t i = 0; i < bytesRead; i++) {
      char c = buffer[i];

      // Add to buffer if space available
      if (nmeaSentenceIdx < SZ_NMEA_BUFFER - 1) {
        nmeaSentenceBuf[nmeaSentenceIdx++] = c;
      }

      // Check for end of NMEA sentence
      if (c == '\n') {
        nmeaSentenceBuf[nmeaSentenceIdx] = '\0';

        // Parse the NMEA sentence
        if (NMEA.parse(nmeaSentenceBuf)) {
          // Successfully parsed sentence!
          Serial.println(nmeaSentenceBuf);
          // approximately every 2 seconds or so, print out the current stats
          if (millis() - timer > 2000) {
            timer = millis(); // reset the timer
            Serial.print(F("\nTime: "));
            if (NMEA.hour < 10) {
              Serial.print('0');
            }
            Serial.print(NMEA.hour, DEC);
            Serial.print(':');
            if (NMEA.minute < 10) {
              Serial.print('0');
            }
            Serial.print(NMEA.minute, DEC);
            Serial.print(':');
            if (NMEA.seconds < 10) {
              Serial.print('0');
            }
            Serial.print(NMEA.seconds, DEC);
            Serial.print('.');
            if (NMEA.milliseconds < 10) {
              Serial.print(F("00"));
            } else if (NMEA.milliseconds > 9 && NMEA.milliseconds < 100) {
              Serial.print(F("0"));
            }
            Serial.println(NMEA.milliseconds);
            Serial.print(F("Date: "));
            Serial.print(NMEA.day, DEC);
            Serial.print('/');
            Serial.print(NMEA.month, DEC);
            Serial.print(F("/20"));
            Serial.println(NMEA.year, DEC);
            Serial.print(F("Fix: "));
            Serial.print((int)NMEA.fix);
            Serial.print(F(" quality: "));
            Serial.println((int)NMEA.fixquality);
            if (NMEA.fix) {
              Serial.print(F("Location: "));
              Serial.print(NMEA.latitude, 4);
              Serial.print(NMEA.lat);
              Serial.print(F(", "));
              Serial.print(NMEA.longitude, 4);
              Serial.println(NMEA.lon);
              Serial.print(F("Speed (knots): "));
              Serial.println(NMEA.speed);
              Serial.print(F("Angle: "));
              Serial.println(NMEA.angle);
              Serial.print(F("Altitude: "));
              Serial.println(NMEA.altitude);
              Serial.print(F("Satellites: "));
              Serial.println((int)NMEA.satellites);
            }
          }
        } else {
          Serial.println(F("Failed to parse sentence."));
        }
        nmeaSentenceIdx = 0; // Reset index for next sentence
      }
    }
  }
  // Short delay to prevent overwhelming the module
  delay(10);
}