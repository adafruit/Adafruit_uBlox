/*!
 * @file nav_velned_test.ino
 *
 * Message test: Poll UBX-NAV-VELNED, wait for fix, validate fields,
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

uint8_t runTests(const UBX_NAV_VELNED_t& velned) {
  uint8_t passed = 0;
  const uint8_t total = 6;

  Serial.println();
  Serial.println(F("Running NAV-VELNED tests..."));

  // iTOW should be non-zero when we have a fix
  bool itow_ok = velned.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.println(velned.iTOW);
  if (itow_ok)
    passed++;

  // Ground speed should be reasonable (< 1000 m/s for stationary)
  double gSpeed_ms = velned.gSpeed / 100.0;  // cm/s to m/s
  bool gspeed_ok = velned.gSpeed < 100000;   // < 1000 m/s
  printTestResult(F("gSpeed_reasonable"), gspeed_ok);
  Serial.print(gSpeed_ms, 3);
  Serial.println(F(" m/s"));
  if (gspeed_ok)
    passed++;

  // 3D speed should be reasonable
  double speed_ms = velned.speed / 100.0;
  bool speed_ok = velned.speed < 100000;
  printTestResult(F("speed_reasonable"), speed_ok);
  Serial.print(speed_ms, 3);
  Serial.println(F(" m/s"));
  if (speed_ok)
    passed++;

  // Heading should be 0-360 degrees
  double heading_deg = velned.heading * 1e-5;
  bool heading_ok = heading_deg >= 0.0 && heading_deg <= 360.0;
  printTestResult(F("heading_range"), heading_ok);
  Serial.print(heading_deg, 2);
  Serial.println(F(" deg"));
  if (heading_ok)
    passed++;

  // Speed accuracy should be < 50 m/s
  double sAcc_ms = velned.sAcc / 100.0;
  bool sAcc_ok = velned.sAcc < 5000;
  printTestResult(F("sAcc_reasonable"), sAcc_ok);
  Serial.print(sAcc_ms, 3);
  Serial.println(F(" m/s"));
  if (sAcc_ok)
    passed++;

  // Course accuracy should be < 180 degrees
  double cAcc_deg = velned.cAcc * 1e-5;
  bool cAcc_ok = velned.cAcc < 18000000;
  printTestResult(F("cAcc_reasonable"), cAcc_ok);
  Serial.print(cAcc_deg, 2);
  Serial.println(F(" deg"));
  if (cAcc_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printVELNED(const UBX_NAV_VELNED_t& velned) {
  Serial.println(F("--- NAV-VELNED ---"));

  Serial.print(F("iTOW: "));
  Serial.print(velned.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("velN: "));
  Serial.print(velned.velN / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("velE: "));
  Serial.print(velned.velE / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("velD: "));
  Serial.print(velned.velD / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("speed (3D): "));
  Serial.print(velned.speed / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("gSpeed (2D): "));
  Serial.print(velned.gSpeed / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("heading: "));
  Serial.print(velned.heading * 1e-5, 2);
  Serial.println(F(" deg"));

  Serial.print(F("sAcc: "));
  Serial.print(velned.sAcc / 100.0, 3);
  Serial.println(F(" m/s"));

  Serial.print(F("cAcc: "));
  Serial.print(velned.cAcc * 1e-5, 2);
  Serial.println(F(" deg"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-VELNED Message Test ==="));

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
  UBX_NAV_VELNED_t velned;

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

      if (ubx.pollNAVVELNED(&velned)) {
        runTests(velned);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-VELNED after fix!"));
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

  if (ubx.pollNAVVELNED(&velned)) {
    printVELNED(velned);
  } else {
    Serial.println(F("NAV-VELNED poll failed (timeout)"));
  }

  delay(2000);
}
