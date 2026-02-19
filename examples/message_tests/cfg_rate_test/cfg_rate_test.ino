/*!
 * @file cfg_rate_test.ino
 *
 * Message test: Poll UBX-CFG-RATE, validate defaults,
 * then set 5 Hz and 10 Hz rates, restore 1 Hz, and stream NAV-PVT.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

bool tests_run = false;
uint16_t current_rate_ms = 1000;
uint32_t last_pvt_ms = 0;

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

bool getRateSafe(UBX_CFG_RATE_t* rate) {
  if (!ubx.getRate(rate)) {
    Serial.println(F("CFG-RATE poll failed"));
    return false;
  }
  return true;
}

bool setAndVerify(uint16_t rate_ms, const __FlashStringHelper* label) {
  if (!ubx.setRate(rate_ms)) {
    printTestResult(label, false);
    Serial.println(F("setRate failed"));
    return false;
  }
  delay(100);
  UBX_CFG_RATE_t rate;
  if (!getRateSafe(&rate)) {
    printTestResult(label, false);
    Serial.println(F("getRate failed"));
    return false;
  }
  bool ok = rate.measRate == rate_ms;
  printTestResult(label, ok);
  Serial.print(rate.measRate);
  Serial.println(F(" ms"));
  if (ok) {
    current_rate_ms = rate_ms;
  }
  return ok;
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 5;

  Serial.println();
  Serial.println(F("Running tests..."));

  UBX_CFG_RATE_t rate;
  bool get_rate_ok = getRateSafe(&rate);
  printTestResult(F("get_rate_ok"), get_rate_ok);
  Serial.println(get_rate_ok ? F("OK") : F("BAD"));
  if (get_rate_ok)
    passed++;

  bool default_1hz = get_rate_ok && (rate.measRate == 1000);
  printTestResult(F("default_1hz"), default_1hz);
  Serial.print(rate.measRate);
  Serial.println(F(" ms"));
  if (default_1hz)
    passed++;

  if (setAndVerify(200, F("set_5hz")))
    passed++;
  if (setAndVerify(100, F("set_10hz")))
    passed++;
  if (setAndVerify(1000, F("restore_1hz")))
    passed++;

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

  Serial.println(F("UBX-CFG-RATE Message Test"));
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

  delay(500); // Give GPS module time to stabilize after boot

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  runTests();
  tests_run = true;
  last_pvt_ms = millis();
  Serial.println();
  Serial.println(F("Streaming NAV-PVT timestamps..."));
}

void loop() {
  UBX_NAV_PVT_t pvt;
  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt))) {
    uint32_t now = millis();
    uint32_t delta = now - last_pvt_ms;
    last_pvt_ms = now;
    Serial.print(F("NAV-PVT iTOW: "));
    Serial.print(pvt.iTOW);
    Serial.print(F(" ms, delta: "));
    Serial.print(delta);
    Serial.print(F(" ms (target "));
    Serial.print(current_rate_ms);
    Serial.println(F(" ms)"));
  } else {
    Serial.println(F("NAV-PVT poll timeout"));
  }
}
