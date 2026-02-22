/*!
 * @file nav_velecef_test.ino
 *
 * Message test: Poll UBX-NAV-VELECEF, wait for fix, validate fields,
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

uint8_t runTests(const UBX_NAV_VELECEF_t& velecef) {
  uint8_t passed = 0;
  const uint8_t total = 5;

  Serial.println();
  Serial.println(F("Running NAV-VELECEF tests..."));

  // iTOW should be non-zero
  bool itow_ok = velecef.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.println(velecef.iTOW);
  if (itow_ok)
    passed++;

  // ECEF VX should be reasonable for stationary (< 1000 m/s = 100000 cm/s)
  bool ecefVX_ok = velecef.ecefVX > -100000 && velecef.ecefVX < 100000;
  printTestResult(F("ecefVX_reasonable"), ecefVX_ok);
  Serial.print(velecef.ecefVX / 100.0, 3);
  Serial.println(F(" m/s"));
  if (ecefVX_ok)
    passed++;

  // ECEF VY should be reasonable
  bool ecefVY_ok = velecef.ecefVY > -100000 && velecef.ecefVY < 100000;
  printTestResult(F("ecefVY_reasonable"), ecefVY_ok);
  Serial.print(velecef.ecefVY / 100.0, 3);
  Serial.println(F(" m/s"));
  if (ecefVY_ok)
    passed++;

  // ECEF VZ should be reasonable
  bool ecefVZ_ok = velecef.ecefVZ > -100000 && velecef.ecefVZ < 100000;
  printTestResult(F("ecefVZ_reasonable"), ecefVZ_ok);
  Serial.print(velecef.ecefVZ / 100.0, 3);
  Serial.println(F(" m/s"));
  if (ecefVZ_ok)
    passed++;

  // Speed accuracy should be < 50 m/s
  double sAcc_ms = velecef.sAcc / 100.0;
  bool sAcc_ok = velecef.sAcc < 5000;
  printTestResult(F("sAcc_reasonable"), sAcc_ok);
  Serial.print(sAcc_ms, 3);
  Serial.println(F(" m/s"));
  if (sAcc_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printVELECEF(const UBX_NAV_VELECEF_t& velecef) {
  Serial.println(F("--- NAV-VELECEF ---"));

  Serial.print(F("iTOW: "));
  Serial.print(velecef.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("ecefVX: "));
  Serial.print(velecef.ecefVX / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("ecefVY: "));
  Serial.print(velecef.ecefVY / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("ecefVZ: "));
  Serial.print(velecef.ecefVZ / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("sAcc: "));
  Serial.print(velecef.sAcc / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-VELECEF Message Test ==="));

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
  UBX_NAV_VELECEF_t velecef;

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

      if (ubx.pollNAVVELECEF(&velecef)) {
        runTests(velecef);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-VELECEF after fix!"));
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

  if (ubx.pollNAVVELECEF(&velecef)) {
    printVELECEF(velecef);
  } else {
    Serial.println(F("NAV-VELECEF poll failed (timeout)"));
  }

  delay(2000);
}
