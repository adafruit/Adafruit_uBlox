/*!
 * @file nav_timeutc_test.ino
 *
 * Message test: Poll UBX-NAV-TIMEUTC, wait for fix, validate fields,
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

void printDate(uint16_t year, uint8_t month, uint8_t day) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u", year, month, day);
  Serial.print(buf);
}

void printTime(uint8_t hour, uint8_t min, uint8_t sec) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u", hour, min, sec);
  Serial.print(buf);
}

uint8_t runTests(const UBX_NAV_TIMEUTC_t& timeutc) {
  uint8_t passed = 0;
  const uint8_t total = 7;

  Serial.println();
  Serial.println(F("Running NAV-TIMEUTC tests..."));

  // iTOW should be non-zero
  bool itow_ok = timeutc.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.println(timeutc.iTOW);
  if (itow_ok)
    passed++;

  // Year should be 2024-2035
  bool year_ok = timeutc.year >= 2024 && timeutc.year <= 2035;
  printTestResult(F("year_range"), year_ok);
  Serial.println(timeutc.year);
  if (year_ok)
    passed++;

  // Month 1-12
  bool month_ok = timeutc.month >= 1 && timeutc.month <= 12;
  printTestResult(F("month_range"), month_ok);
  Serial.println(timeutc.month);
  if (month_ok)
    passed++;

  // Day 1-31
  bool day_ok = timeutc.day >= 1 && timeutc.day <= 31;
  printTestResult(F("day_range"), day_ok);
  Serial.println(timeutc.day);
  if (day_ok)
    passed++;

  // Hour 0-23
  bool hour_ok = timeutc.hour <= 23;
  printTestResult(F("hour_range"), hour_ok);
  Serial.println(timeutc.hour);
  if (hour_ok)
    passed++;

  // Minute 0-59
  bool min_ok = timeutc.min <= 59;
  printTestResult(F("min_range"), min_ok);
  Serial.println(timeutc.min);
  if (min_ok)
    passed++;

  // Second 0-60 (60 for leap second)
  bool sec_ok = timeutc.sec <= 60;
  printTestResult(F("sec_range"), sec_ok);
  Serial.println(timeutc.sec);
  if (sec_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printTIMEUTC(const UBX_NAV_TIMEUTC_t& timeutc) {
  Serial.println(F("--- NAV-TIMEUTC ---"));

  Serial.print(F("iTOW: "));
  Serial.print(timeutc.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("tAcc: "));
  Serial.print(timeutc.tAcc);
  Serial.println(F(" ns"));

  Serial.print(F("nano: "));
  Serial.print(timeutc.nano);
  Serial.println(F(" ns"));

  Serial.print(F("UTC: "));
  printDate(timeutc.year, timeutc.month, timeutc.day);
  Serial.print(F(" "));
  printTime(timeutc.hour, timeutc.min, timeutc.sec);
  Serial.println();

  Serial.print(F("valid: 0x"));
  Serial.print(timeutc.valid, HEX);
  Serial.print(F(" (TOW="));
  Serial.print(timeutc.valid & 0x01);
  Serial.print(F(", WKN="));
  Serial.print((timeutc.valid >> 1) & 0x01);
  Serial.print(F(", UTC="));
  Serial.print((timeutc.valid >> 2) & 0x01);
  Serial.println(F(")"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-TIMEUTC Message Test ==="));

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
  UBX_NAV_TIMEUTC_t timeutc;

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

      if (ubx.pollNAVTIMEUTC(&timeutc)) {
        runTests(timeutc);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-TIMEUTC after fix!"));
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

  if (ubx.pollNAVTIMEUTC(&timeutc)) {
    printTIMEUTC(timeutc);
  } else {
    Serial.println(F("NAV-TIMEUTC poll failed (timeout)"));
  }

  delay(2000);
}
