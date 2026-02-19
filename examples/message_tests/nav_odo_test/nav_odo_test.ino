/*!
 * @file nav_odo_test.ino
 *
 * Message test: Poll UBX-NAV-ODO, test reset, validate fields.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

static const uint32_t FIX_TIMEOUT_MS = 120000;

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
  const uint8_t total = 5;
  UBX_NAV_ODO_t odo;

  Serial.println();
  Serial.println(F("Running NAV-ODO tests..."));

  // Test 1: Basic poll
  bool poll_ok = ubx.pollNavOdo(&odo);
  printTestResult(F("poll_success"), poll_ok);
  Serial.println(poll_ok ? F("OK") : F("FAIL"));
  if (poll_ok)
    passed++;

  if (!poll_ok) {
    Serial.println(F("Cannot continue without successful poll"));
    Serial.print(F("Results: "));
    Serial.print(passed);
    Serial.print(F("/"));
    Serial.print(total);
    Serial.println(F(" tests passed"));
    return passed;
  }

  // Test 2: Version should be 0
  bool version_ok = odo.version == 0;
  printTestResult(F("version_zero"), version_ok);
  Serial.println(odo.version);
  if (version_ok)
    passed++;

  // Test 3: iTOW should be non-zero (if we have time)
  bool itow_ok = odo.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.print(odo.iTOW);
  Serial.println(F(" ms"));
  if (itow_ok)
    passed++;

  // Test 4: Test reset odometer
  bool reset_ok = ubx.resetOdometer();
  printTestResult(F("reset_ack"), reset_ok);
  Serial.println(reset_ok ? F("OK") : F("FAIL"));
  if (reset_ok)
    passed++;

  // Test 5: After reset, distance should be small (< 100m)
  delay(500); // Give module time to process
  if (ubx.pollNavOdo(&odo)) {
    bool distance_reset = odo.distance < 100;
    printTestResult(F("distance_reset"), distance_reset);
    Serial.print(odo.distance);
    Serial.println(F(" m"));
    if (distance_reset)
      passed++;
  } else {
    printTestResult(F("distance_reset"), false);
    Serial.println(F("poll failed"));
  }

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printODO(const UBX_NAV_ODO_t& odo) {
  Serial.println(F("--- NAV-ODO ---"));

  Serial.print(F("iTOW: "));
  Serial.print(odo.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("distance: "));
  Serial.print(odo.distance);
  Serial.println(F(" m (since reset)"));

  Serial.print(F("totalDistance: "));
  Serial.print(odo.totalDistance);
  Serial.println(F(" m (cumulative)"));

  Serial.print(F("distanceStd: "));
  Serial.print(odo.distanceStd);
  Serial.println(F(" m (1-sigma)"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-ODO Message Test ==="));

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
  delay(2000);
}

void loop() {
  UBX_NAV_ODO_t odo;

  if (!printed_continuous_header) {
    Serial.println();
    Serial.println(F("Continuous output:"));
    printed_continuous_header = true;
  }

  if (ubx.pollNavOdo(&odo)) {
    printODO(odo);
  } else {
    Serial.println(F("NAV-ODO poll failed (timeout)"));
  }

  delay(2000);
}
