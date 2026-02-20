/*!
 * @file nav_timels_test.ino
 *
 * Message test: Poll UBX-NAV-TIMELS for leap second information.
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

const char* getSourceName(uint8_t src) {
  switch (src) {
    case 0:
      return "Default";
    case 1:
      return "GPS";
    case 2:
      return "SBAS";
    case 3:
      return "BeiDou";
    case 4:
      return "Galileo";
    case 5:
      return "GLONASS";
    case 255:
      return "Unknown";
    default:
      return "Reserved";
  }
}

uint8_t runTests(const UBX_NAV_TIMELS_t& timels) {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running NAV-TIMELS tests..."));

  // Test 1: Version should be 0
  bool version_ok = timels.version == 0;
  printTestResult(F("version_zero"), version_ok);
  Serial.println(timels.version);
  if (version_ok)
    passed++;

  // Test 2: iTOW should be non-zero
  bool itow_ok = timels.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.print(timels.iTOW);
  Serial.println(F(" ms"));
  if (itow_ok)
    passed++;

  // Test 3: Current leap seconds should be reasonable (15-30 range as of 2024)
  bool currLs_ok = timels.currLs >= 0 && timels.currLs < 50;
  printTestResult(F("currLs_reasonable"), currLs_ok);
  Serial.print(timels.currLs);
  Serial.println(F(" s"));
  if (currLs_ok)
    passed++;

  // Test 4: Source should be valid enum
  bool src_ok = timels.srcOfCurrLs <= 5 || timels.srcOfCurrLs == 255;
  printTestResult(F("source_valid"), src_ok);
  Serial.println(getSourceName(timels.srcOfCurrLs));
  if (src_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printTIMELS(const UBX_NAV_TIMELS_t& timels) {
  Serial.println(F("--- NAV-TIMELS ---"));

  Serial.print(F("iTOW: "));
  Serial.print(timels.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("srcOfCurrLs: "));
  Serial.print(timels.srcOfCurrLs);
  Serial.print(F(" ("));
  Serial.print(getSourceName(timels.srcOfCurrLs));
  Serial.println(F(")"));

  Serial.print(F("currLs: "));
  Serial.print(timels.currLs);
  Serial.println(F(" s (GPS-UTC)"));

  Serial.print(F("srcOfLsChange: "));
  Serial.print(timels.srcOfLsChange);
  Serial.print(F(" ("));
  Serial.print(getSourceName(timels.srcOfLsChange));
  Serial.println(F(")"));

  Serial.print(F("lsChange: "));
  Serial.print(timels.lsChange);
  Serial.println(F(" s (upcoming)"));

  Serial.print(F("timeToLsEvent: "));
  Serial.print(timels.timeToLsEvent);
  Serial.println(F(" s"));

  if (timels.dateOfLsGpsWn > 0) {
    Serial.print(F("dateOfLsEvent: GPS week "));
    Serial.print(timels.dateOfLsGpsWn);
    Serial.print(F(", day "));
    Serial.println(timels.dateOfLsGpsDn);
  }

  Serial.print(F("valid: 0x"));
  Serial.print(timels.valid, HEX);
  Serial.print(F(" (currLs="));
  Serial.print((timels.valid & UBX_TIMELS_VALID_CURR_LS) ? F("Y") : F("N"));
  Serial.print(F(", timeToEvent="));
  Serial.print((timels.valid & UBX_TIMELS_VALID_TIME_TO_EVENT) ? F("Y") : F("N"));
  Serial.println(F(")"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-TIMELS Message Test ==="));

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
  UBX_NAV_TIMELS_t timels;

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

      if (ubx.pollNavTimels(&timels)) {
        runTests(timels);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-TIMELS after fix!"));
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

  if (ubx.pollNavTimels(&timels)) {
    printTIMELS(timels);
  } else {
    Serial.println(F("NAV-TIMELS poll failed (timeout)"));
  }

  delay(2000);
}
