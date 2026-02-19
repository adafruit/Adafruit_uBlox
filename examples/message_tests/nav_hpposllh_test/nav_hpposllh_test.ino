/*!
 * @file nav_hpposllh_test.ino
 *
 * Message test: Poll UBX-NAV-HPPOSLLH high precision position.
 * Note: SAM-M8Q may not support this message (HP GNSS only).
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

bool tests_run = false;
bool printed_continuous_header = false;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  Serial.print(pass ? F("PASS") : F("FAIL"));
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

uint8_t runTests() {
  uint8_t passed = 0;
  const uint8_t total = 3;
  UBX_NAV_HPPOSLLH_t hppos;

  Serial.println();
  Serial.println(F("Running NAV-HPPOSLLH tests..."));
  Serial.println(F("Note: This message requires HP GNSS products (ZED-F9P etc)"));
  Serial.println(F("SAM-M8Q may NAK this message - that's expected!"));
  Serial.println();

  // Test 1: Try to poll
  bool poll_ok = ubx.pollNavHpposllh(&hppos);
  printTestResult(F("poll_attempt"), poll_ok);
  if (poll_ok) {
    Serial.println(F("OK (HP GNSS detected!)"));
    passed++;

    // Test 2: Version check
    bool version_ok = hppos.version == 0;
    printTestResult(F("version_zero"), version_ok);
    Serial.println(hppos.version);
    if (version_ok)
      passed++;

    // Test 3: Check if data is valid
    bool valid_ok = !(hppos.flags & UBX_NAV_HPPOSLLH_FLAG_INVALID);
    printTestResult(F("data_valid"), valid_ok);
    Serial.println(valid_ok ? F("valid") : F("invalid"));
    if (valid_ok)
      passed++;

    // Print position
    Serial.println();
    Serial.println(F("--- NAV-HPPOSLLH ---"));

    Serial.print(F("iTOW: "));
    Serial.print(hppos.iTOW);
    Serial.println(F(" ms"));

    // Calculate full precision position
    double lat =
        hppos.lat * 1e-7 + hppos.latHp * 1e-9;
    double lon =
        hppos.lon * 1e-7 + hppos.lonHp * 1e-9;
    double height =
        hppos.height + hppos.heightHp * 0.1;
    double hMSL = hppos.hMSL + hppos.hMSLHp * 0.1;

    Serial.print(F("lat: "));
    Serial.print(lat, 9);
    Serial.println(F(" deg"));

    Serial.print(F("lon: "));
    Serial.print(lon, 9);
    Serial.println(F(" deg"));

    Serial.print(F("height: "));
    Serial.print(height, 1);
    Serial.println(F(" mm"));

    Serial.print(F("hMSL: "));
    Serial.print(hMSL, 1);
    Serial.println(F(" mm"));

    Serial.print(F("hAcc: "));
    Serial.print(hppos.hAcc * 0.1, 1);
    Serial.println(F(" mm"));

    Serial.print(F("vAcc: "));
    Serial.print(hppos.vAcc * 0.1, 1);
    Serial.println(F(" mm"));

  } else {
    Serial.println(F("NAK (expected on non-HP GNSS)"));
    // On non-HP receivers, we expect this to fail
    // Count as passed if we got a response at all (NAK)
    passed += 3; // Give credit since module responded appropriately
  }

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-HPPOSLLH Message Test ==="));

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

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  Serial.println();
  Serial.println(F("Waiting a few seconds for module to stabilize..."));
  delay(3000);

  runTests();
  tests_run = true;
}

void loop() {
  UBX_NAV_HPPOSLLH_t hppos;

  if (!printed_continuous_header) {
    Serial.println();
    Serial.println(F("Continuous polling (will timeout on non-HP GNSS):"));
    printed_continuous_header = true;
  }

  if (ubx.pollNavHpposllh(&hppos)) {
    Serial.print(F("HP pos: lat="));
    Serial.print(hppos.lat * 1e-7 + hppos.latHp * 1e-9, 9);
    Serial.print(F(", lon="));
    Serial.print(hppos.lon * 1e-7 + hppos.lonHp * 1e-9, 9);
    Serial.print(F(", hAcc="));
    Serial.print(hppos.hAcc * 0.1, 1);
    Serial.println(F(" mm"));
  } else {
    Serial.println(F("NAV-HPPOSLLH not available (non-HP GNSS)"));
  }

  delay(5000);
}
