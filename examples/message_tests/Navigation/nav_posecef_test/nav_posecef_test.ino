/*!
 * @file nav_posecef_test.ino
 *
 * Message test: Poll UBX-NAV-POSECEF, wait for fix, validate fields,
 * then stream parsed data for inspection.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

static const uint32_t FIX_TIMEOUT_MS = 120000;

bool tests_run = false;
bool fix_acquired = false;
bool printed_continuous_header = false;
uint32_t attempt = 0;
unsigned long fix_start_ms = 0;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  Serial.print(pass ? F("PASS") : F("FAIL"));
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

uint8_t runTests(const UBX_NAV_POSECEF_t& posecef) {
  uint8_t passed = 0;
  const uint8_t total = 5;

  Serial.println();
  Serial.println(F("Running NAV-POSECEF tests..."));

  // iTOW should be non-zero
  bool itow_ok = posecef.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.println(posecef.iTOW);
  if (itow_ok)
    passed++;

  // ECEF X should be in Earth range (approx -6.4e8 to 6.4e8 cm)
  // For continental US, X is roughly -150e6 to -50e6 cm
  bool ecefX_ok =
      posecef.ecefX > -700000000L && posecef.ecefX < 700000000L;
  printTestResult(F("ecefX_range"), ecefX_ok);
  Serial.print(posecef.ecefX / 100.0, 2);
  Serial.println(F(" m"));
  if (ecefX_ok)
    passed++;

  // ECEF Y should be in Earth range
  bool ecefY_ok =
      posecef.ecefY > -700000000L && posecef.ecefY < 700000000L;
  printTestResult(F("ecefY_range"), ecefY_ok);
  Serial.print(posecef.ecefY / 100.0, 2);
  Serial.println(F(" m"));
  if (ecefY_ok)
    passed++;

  // ECEF Z should be in Earth range
  bool ecefZ_ok =
      posecef.ecefZ > -700000000L && posecef.ecefZ < 700000000L;
  printTestResult(F("ecefZ_range"), ecefZ_ok);
  Serial.print(posecef.ecefZ / 100.0, 2);
  Serial.println(F(" m"));
  if (ecefZ_ok)
    passed++;

  // Position accuracy should be < 100m
  double pAcc_m = posecef.pAcc / 100.0;
  bool pAcc_ok = posecef.pAcc < 10000;
  printTestResult(F("pAcc_reasonable"), pAcc_ok);
  Serial.print(pAcc_m, 2);
  Serial.println(F(" m"));
  if (pAcc_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printPOSECEF(const UBX_NAV_POSECEF_t& posecef) {
  Serial.println(F("--- NAV-POSECEF ---"));

  Serial.print(F("iTOW: "));
  Serial.print(posecef.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("ecefX: "));
  Serial.print(posecef.ecefX / 100.0, 2);
  Serial.println(F(" m"));

  Serial.print(F("ecefY: "));
  Serial.print(posecef.ecefY / 100.0, 2);
  Serial.println(F(" m"));

  Serial.print(F("ecefZ: "));
  Serial.print(posecef.ecefZ / 100.0, 2);
  Serial.println(F(" m"));

  Serial.print(F("pAcc: "));
  Serial.print(posecef.pAcc / 100.0, 2);
  Serial.println(F(" m"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-POSECEF Message Test ==="));

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
  fix_start_ms = millis();
}

void loop() {
  UBX_NAV_PVT_t pvt;
  UBX_NAV_POSECEF_t posecef;

  if (!tests_run) {
    attempt++;
    bool got = ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt));

    if (attempt == 1) {
      Serial.println(F("Waiting for 3D fix..."));
    } else {
      Serial.print(F("Waiting for fix... (attempt "));
      Serial.print(attempt);
      if (got) {
        Serial.print(F(", fixType="));
        Serial.print(pvt.fixType);
        Serial.print(F(", sats="));
        Serial.print(pvt.numSV);
        Serial.println(F(")"));
      } else {
        Serial.println(F(", poll failed)"));
      }
    }

    if (got && pvt.fixType == 3 && (pvt.flags & 0x01)) {
      fix_acquired = true;
      Serial.print(F("Fix acquired! fixType="));
      Serial.print(pvt.fixType);
      Serial.print(F(", sats="));
      Serial.println(pvt.numSV);

      if (ubx.pollNAVPOSECEF(&posecef)) {
        runTests(posecef);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-POSECEF after fix!"));
      }
      tests_run = true;
      printed_continuous_header = false;
      delay(2000);
      return;
    }

    if (millis() - fix_start_ms > FIX_TIMEOUT_MS) {
      Serial.println(F("Timeout waiting for 3D fix after 120 seconds."));
      tests_run = true;
    }

    delay(1000);
    return;
  }

  if (!printed_continuous_header) {
    Serial.println();
    Serial.println(F("Continuous output:"));
    printed_continuous_header = true;
  }

  if (ubx.pollNAVPOSECEF(&posecef)) {
    printPOSECEF(posecef);
  } else {
    Serial.println(F("NAV-POSECEF poll failed (timeout)"));
  }

  delay(2000);
}
