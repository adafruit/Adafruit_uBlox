/*!
 * @file cfg_nmea_test.ino
 *
 * Message test: Poll UBX-CFG-NMEA, display settings,
 * toggle high precision mode, verify.
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

const char* getNmeaVersionName(uint8_t ver) {
  switch (ver) {
    case 0x21: return "2.1";
    case 0x23: return "2.3";
    case 0x40: return "4.0";
    case 0x41: return "4.10";
    case 0x4B: return "4.11";
    default: return "Unknown";
  }
}

void printNmeaConfig(UBX_CFG_NMEA_t* nmea) {
  Serial.print(F("  filter: 0x"));
  Serial.println(nmea->filter, HEX);
  Serial.print(F("  nmeaVersion: 0x"));
  Serial.print(nmea->nmeaVersion, HEX);
  Serial.print(F(" ("));
  Serial.print(getNmeaVersionName(nmea->nmeaVersion));
  Serial.println(F(")"));
  Serial.print(F("  numSV: "));
  Serial.println(nmea->numSV);
  Serial.print(F("  flags: 0x"));
  Serial.println(nmea->flags, HEX);
  Serial.print(F("    compat: "));
  Serial.println((nmea->flags & UBX_NMEA_FLAGS_COMPAT) ? F("yes") : F("no"));
  Serial.print(F("    consider: "));
  Serial.println((nmea->flags & UBX_NMEA_FLAGS_CONSIDER) ? F("yes") : F("no"));
  Serial.print(F("    limit82: "));
  Serial.println((nmea->flags & UBX_NMEA_FLAGS_LIMIT82) ? F("yes") : F("no"));
  Serial.print(F("    highPrec: "));
  Serial.println((nmea->flags & UBX_NMEA_FLAGS_HIGHPREC) ? F("yes") : F("no"));
  Serial.print(F("  mainTalkerId: "));
  Serial.println(nmea->mainTalkerId);
  Serial.print(F("  gsvTalkerId: "));
  Serial.println(nmea->gsvTalkerId);
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running tests..."));

  // Test 1: Poll CFG-NMEA
  UBX_CFG_NMEA_t nmea;
  bool poll_ok = ubx.pollCfgNmea(&nmea);
  printTestResult(F("poll_cfg_nmea"), poll_ok);
  Serial.println(poll_ok ? F("OK") : F("FAILED"));
  if (poll_ok) {
    passed++;
    printNmeaConfig(&nmea);
  }

  // Test 2: Verify version field
  bool ver_ok = poll_ok && (nmea.version == 0x01);
  printTestResult(F("version_v1"), ver_ok);
  Serial.print(F("0x"));
  Serial.println(nmea.version, HEX);
  if (ver_ok) passed++;

  // Save original flags
  uint8_t original_flags = nmea.flags;

  // Test 3: Toggle compat mode
  nmea.flags ^= UBX_NMEA_FLAGS_COMPAT;
  bool set_ok = ubx.setCfgNmea(&nmea);
  printTestResult(F("set_toggle_compat"), set_ok);
  Serial.println(set_ok ? F("OK") : F("FAILED"));
  if (set_ok) passed++;

  delay(100);

  // Test 4: Verify change and restore
  UBX_CFG_NMEA_t nmea_verify;
  bool verify_ok = ubx.pollCfgNmea(&nmea_verify);
  if (verify_ok) {
    bool changed = (nmea_verify.flags & UBX_NMEA_FLAGS_COMPAT) !=
                   (original_flags & UBX_NMEA_FLAGS_COMPAT);
    printTestResult(F("verify_change"), changed);
    Serial.println(changed ? F("OK") : F("no change"));
    if (changed) passed++;

    // Restore original
    nmea_verify.flags = original_flags;
    ubx.setCfgNmea(&nmea_verify);
  } else {
    printTestResult(F("verify_change"), false);
    Serial.println(F("poll failed"));
  }

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

  Serial.println(F("UBX-CFG-NMEA Message Test"));
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
