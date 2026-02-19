/*!
 * @file nav_timegps_test.ino
 *
 * Message test: Poll UBX-NAV-TIMEGPS, wait for fix, validate fields,
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

uint8_t runTests(const UBX_NAV_TIMEGPS_t& timegps) {
  uint8_t passed = 0;
  const uint8_t total = 5;

  Serial.println();
  Serial.println(F("Running NAV-TIMEGPS tests..."));

  // iTOW should be non-zero
  bool itow_ok = timegps.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.println(timegps.iTOW);
  if (itow_ok)
    passed++;

  // fTOW should be in range +/- 500000 ns
  bool ftow_ok = timegps.fTOW >= -500000 && timegps.fTOW <= 500000;
  printTestResult(F("fTOW_range"), ftow_ok);
  Serial.print(timegps.fTOW);
  Serial.println(F(" ns"));
  if (ftow_ok)
    passed++;

  // GPS week should be > 2300 (we're well past that)
  // GPS week 0 was Jan 6, 1980. Week 2300 was around late 2024.
  bool week_ok = timegps.week > 2300;
  printTestResult(F("week_reasonable"), week_ok);
  Serial.println(timegps.week);
  if (week_ok)
    passed++;

  // Leap seconds should be around 18-20 (as of 2024)
  bool leaps_ok = timegps.leapS >= 15 && timegps.leapS <= 25;
  printTestResult(F("leapS_range"), leaps_ok);
  Serial.print(timegps.leapS);
  Serial.println(F(" s"));
  if (leaps_ok)
    passed++;

  // Time accuracy should be < 1 second
  bool tAcc_ok = timegps.tAcc < 1000000000;
  printTestResult(F("tAcc_reasonable"), tAcc_ok);
  Serial.print(timegps.tAcc);
  Serial.println(F(" ns"));
  if (tAcc_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printTIMEGPS(const UBX_NAV_TIMEGPS_t& timegps) {
  Serial.println(F("--- NAV-TIMEGPS ---"));

  Serial.print(F("iTOW: "));
  Serial.print(timegps.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("fTOW: "));
  Serial.print(timegps.fTOW);
  Serial.println(F(" ns"));

  Serial.print(F("week: "));
  Serial.println(timegps.week);

  Serial.print(F("leapS: "));
  Serial.print(timegps.leapS);
  Serial.println(F(" s"));

  Serial.print(F("valid: 0x"));
  Serial.print(timegps.valid, HEX);
  Serial.print(F(" (TOW="));
  Serial.print(timegps.valid & 0x01);
  Serial.print(F(", week="));
  Serial.print((timegps.valid >> 1) & 0x01);
  Serial.print(F(", leapS="));
  Serial.print((timegps.valid >> 2) & 0x01);
  Serial.println(F(")"));

  Serial.print(F("tAcc: "));
  Serial.print(timegps.tAcc);
  Serial.println(F(" ns"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-TIMEGPS Message Test ==="));

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
  UBX_NAV_TIMEGPS_t timegps;

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

      if (ubx.pollNAVTIMEGPS(&timegps)) {
        runTests(timegps);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-TIMEGPS after fix!"));
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

  if (ubx.pollNAVTIMEGPS(&timegps)) {
    printTIMEGPS(timegps);
  } else {
    Serial.println(F("NAV-TIMEGPS poll failed (timeout)"));
  }

  delay(2000);
}
