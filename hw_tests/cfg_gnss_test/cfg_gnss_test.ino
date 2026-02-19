/*!
 * @file cfg_gnss_test.ino (hw_test)
 *
 * Hardware test for UBX-CFG-GNSS message.
 * Tests GNSS configuration polling on real hardware.
 *
 * Target: QT Py RP2040 + SAM-M8Q (I2C 0x42)
 * Reset pin: A0, Baud: 115200
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

#define RESET_PIN A0

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* getGnssName(uint8_t gnssId) {
  switch (gnssId) {
    case UBX_GNSS_ID_GPS:
      return "GPS";
    case UBX_GNSS_ID_SBAS:
      return "SBAS";
    case UBX_GNSS_ID_GALILEO:
      return "Galileo";
    case UBX_GNSS_ID_BEIDOU:
      return "BeiDou";
    case UBX_GNSS_ID_QZSS:
      return "QZSS";
    case UBX_GNSS_ID_GLONASS:
      return "GLONASS";
    default:
      return "Unknown";
  }
}

void resetGPS() {
  Serial.println(F("Resetting GPS module..."));
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(RESET_PIN, INPUT);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW TEST: CFG-GNSS ==="));

  resetGPS();

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS connected"));

  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  // Run tests
  uint8_t passed = 0;
  const uint8_t total = 2;

  // Test 1: Poll GNSS configuration
  UBX_CFG_GNSS_header_t header;
  UBX_CFG_GNSS_block_t blocks[7];
  uint8_t numBlocks = ubx.pollCfgGnss(&header, blocks, 7);
  bool test1 = (numBlocks > 0);
  Serial.print(F("["));
  Serial.print(test1 ? F("PASS") : F("FAIL"));
  Serial.print(F("] pollCfgGnss: "));
  Serial.print(numBlocks);
  Serial.println(F(" systems"));
  if (test1) {
    passed++;
    Serial.print(F("  HW channels: "));
    Serial.println(header.numTrkChHw);
    for (uint8_t i = 0; i < numBlocks; i++) {
      Serial.print(F("  "));
      Serial.print(getGnssName(blocks[i].gnssId));
      Serial.print(F(": "));
      Serial.println((blocks[i].flags & UBX_GNSS_FLAG_ENABLE) ? F("ON") : F("off"));
    }
  }

  // Test 2: Verify GPS is enabled
  bool gpsEnabled = false;
  for (uint8_t i = 0; i < numBlocks; i++) {
    if (blocks[i].gnssId == UBX_GNSS_ID_GPS) {
      gpsEnabled = (blocks[i].flags & UBX_GNSS_FLAG_ENABLE) != 0;
      break;
    }
  }
  Serial.print(F("["));
  Serial.print(gpsEnabled ? F("PASS") : F("FAIL"));
  Serial.println(F("] GPS enabled"));
  if (gpsEnabled)
    passed++;

  Serial.println();
  Serial.print(F("RESULT: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(passed == total ? F(" PASS") : F(" FAIL"));
}

void loop() {
  delay(1000);
}
