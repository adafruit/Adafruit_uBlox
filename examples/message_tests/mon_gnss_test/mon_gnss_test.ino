/*!
 * @file mon_gnss_test.ino
 *
 * Message test: Poll UBX-MON-GNSS (GNSS System Information)
 * Displays supported, default, and enabled GNSS systems.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.println(name);
}

void printGnssBits(uint8_t bits) {
  if (bits & UBX_MON_GNSS_GPS) Serial.print(F("GPS "));
  if (bits & UBX_MON_GNSS_GLONASS) Serial.print(F("GLONASS "));
  if (bits & UBX_MON_GNSS_BEIDOU) Serial.print(F("BeiDou "));
  if (bits & UBX_MON_GNSS_GALILEO) Serial.print(F("Galileo "));
  if (bits == 0) Serial.print(F("(none)"));
}

void printGnssDetails(UBX_MON_GNSS_t* gnss) {
  Serial.println(F("\nMON-GNSS Details:"));

  Serial.print(F("  Version: "));
  Serial.println(gnss->version);

  Serial.print(F("  Supported: "));
  printGnssBits(gnss->supported);
  Serial.print(F(" (0x"));
  Serial.print(gnss->supported, HEX);
  Serial.println(F(")"));

  Serial.print(F("  Default: "));
  printGnssBits(gnss->defaultGnss);
  Serial.print(F(" (0x"));
  Serial.print(gnss->defaultGnss, HEX);
  Serial.println(F(")"));

  Serial.print(F("  Enabled: "));
  printGnssBits(gnss->enabled);
  Serial.print(F(" (0x"));
  Serial.print(gnss->enabled, HEX);
  Serial.println(F(")"));

  Serial.print(F("  Max simultaneous: "));
  Serial.println(gnss->simultaneous);
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running MON-GNSS tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_MON_GNSS_t) == 8);
  printTestResult(F("struct_size_8"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll MON-GNSS
  UBX_MON_GNSS_t gnss;
  bool poll_ok = ubx.pollMonGnss(&gnss);
  printTestResult(F("poll_mon_gnss"), poll_ok);
  if (poll_ok) {
    passed++;
    printGnssDetails(&gnss);
  }

  // Test 3: Verify GPS is supported (all u-blox modules support GPS)
  bool gps_supported = poll_ok && (gnss.supported & UBX_MON_GNSS_GPS);
  printTestResult(F("gps_supported"), gps_supported);
  if (gps_supported) passed++;

  // Test 4: Verify at least one GNSS is enabled
  bool gnss_enabled = poll_ok && (gnss.enabled != 0);
  printTestResult(F("gnss_enabled"), gnss_enabled);
  if (gnss_enabled) passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-MON-GNSS Message Test"));
  Serial.println(F("=========================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: Could not connect to GPS module!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS module connected on I2C"));

  if (!ubx.begin()) {
    Serial.println(F("FAIL: UBX parser init failed!"));
    while (1)
      delay(10);
  }

  delay(500);

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  runTests();
  Serial.println();
  Serial.println(F("Streaming GNSS info..."));
}

void loop() {
  delay(5000);

  UBX_MON_GNSS_t gnss;
  if (ubx.pollMonGnss(&gnss)) {
    Serial.print(F("MON-GNSS: enabled="));
    printGnssBits(gnss.enabled);
    Serial.print(F(", maxSim="));
    Serial.println(gnss.simultaneous);
  } else {
    Serial.println(F("MON-GNSS poll timeout"));
  }
}
