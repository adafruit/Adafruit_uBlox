/*!
 * @file cfg_gnss_test.ino
 *
 * Message test: Poll UBX-CFG-GNSS, display all GNSS systems,
 * toggle GLONASS enable/disable, verify.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

bool tests_run = false;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

const char* getGnssName(uint8_t gnssId) {
  switch (gnssId) {
    case UBX_GNSS_ID_GPS: return "GPS";
    case UBX_GNSS_ID_SBAS: return "SBAS";
    case UBX_GNSS_ID_GALILEO: return "Galileo";
    case UBX_GNSS_ID_BEIDOU: return "BeiDou";
    case UBX_GNSS_ID_IMES: return "IMES";
    case UBX_GNSS_ID_QZSS: return "QZSS";
    case UBX_GNSS_ID_GLONASS: return "GLONASS";
    default: return "Unknown";
  }
}

void printGnssConfig(UBX_CFG_GNSS_header_t* header,
                     UBX_CFG_GNSS_block_t* blocks, uint8_t numBlocks) {
  Serial.print(F("  numTrkChHw: "));
  Serial.println(header->numTrkChHw);
  Serial.print(F("  numTrkChUse: "));
  Serial.println(header->numTrkChUse);
  Serial.print(F("  numConfigBlocks: "));
  Serial.println(header->numConfigBlocks);
  Serial.println(F("  GNSS Systems:"));
  for (uint8_t i = 0; i < numBlocks; i++) {
    Serial.print(F("    "));
    Serial.print(getGnssName(blocks[i].gnssId));
    Serial.print(F(": "));
    Serial.print((blocks[i].flags & UBX_GNSS_FLAG_ENABLE) ? F("ENABLED") : F("disabled"));
    Serial.print(F(" (resTrkCh="));
    Serial.print(blocks[i].resTrkCh);
    Serial.print(F(", maxTrkCh="));
    Serial.print(blocks[i].maxTrkCh);
    Serial.println(F(")"));
  }
}

bool isGlonassEnabled(UBX_CFG_GNSS_block_t* blocks, uint8_t numBlocks) {
  for (uint8_t i = 0; i < numBlocks; i++) {
    if (blocks[i].gnssId == UBX_GNSS_ID_GLONASS) {
      return (blocks[i].flags & UBX_GNSS_FLAG_ENABLE) != 0;
    }
  }
  return false;
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running tests..."));

  // Test 1: Poll CFG-GNSS
  UBX_CFG_GNSS_header_t header;
  UBX_CFG_GNSS_block_t blocks[8];
  uint8_t numBlocks = ubx.pollCfgGnss(&header, blocks, 8);
  bool poll_ok = (numBlocks > 0);
  printTestResult(F("poll_cfg_gnss"), poll_ok);
  Serial.print(numBlocks);
  Serial.println(F(" blocks"));
  if (poll_ok) {
    passed++;
    printGnssConfig(&header, blocks, numBlocks);
  }

  // Test 2: Check GPS is enabled (should always be)
  bool gps_enabled = false;
  for (uint8_t i = 0; i < numBlocks; i++) {
    if (blocks[i].gnssId == UBX_GNSS_ID_GPS) {
      gps_enabled = (blocks[i].flags & UBX_GNSS_FLAG_ENABLE) != 0;
      break;
    }
  }
  printTestResult(F("gps_enabled"), gps_enabled);
  Serial.println(gps_enabled ? F("yes") : F("no"));
  if (gps_enabled) passed++;

  // Test 3: Toggle GLONASS (if available)
  bool original_glonass = isGlonassEnabled(blocks, numBlocks);
  Serial.print(F("  Original GLONASS state: "));
  Serial.println(original_glonass ? F("enabled") : F("disabled"));

  // Try to toggle GLONASS
  bool toggle_ok = ubx.enableGNSS(UBX_GNSS_ID_GLONASS, !original_glonass);
  printTestResult(F("toggle_glonass"), toggle_ok);
  Serial.println(toggle_ok ? F("OK") : F("FAILED or not supported"));
  // Note: SAM-M8Q may not support GLONASS, so we count partial success
  if (toggle_ok) passed++;

  delay(600); // CFG-GNSS needs time to apply

  // Test 4: Restore original state
  bool restore_ok = ubx.enableGNSS(UBX_GNSS_ID_GLONASS, original_glonass);
  printTestResult(F("restore_glonass"), restore_ok);
  Serial.println(restore_ok ? F("OK") : F("FAILED"));
  if (restore_ok) passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));
  Serial.println(F("(Note: GLONASS tests may fail on modules that don't support it)"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-CFG-GNSS Message Test"));
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
  tests_run = true;
}

void loop() {
  delay(1000);
}
