/*!
 * @file cfg_rst_test.ino
 *
 * Message test: Send UBX-CFG-RST hot start and validate time to fix.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

static const uint32_t FIX_TIMEOUT_MS = 120000;

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

bool initModule() {
  if (!ddc.begin()) {
    return false;
  }
  if (!ubx.begin()) {
    return false;
  }
  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  }
  return true;
}

bool waitForFix(uint32_t* ttff_ms) {
  uint32_t start_ms = millis();
  while (millis() - start_ms < FIX_TIMEOUT_MS) {
    UBX_NAV_STATUS_t status;
    if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_STATUS, &status, sizeof(status))) {
      bool fix_ok = (status.flags & 0x01) != 0;
      bool fix_3d = status.gpsFix == 3;
      if (fix_ok && fix_3d) {
        *ttff_ms = status.ttff;
        return true;
      }
    }
    delay(200);
  }
  return false;
}

bool reconnectModule(uint32_t timeout_ms) {
  uint32_t start_ms = millis();
  while (millis() - start_ms < timeout_ms) {
    if (initModule()) {
      return true;
    }
    delay(200);
  }
  return false;
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Waiting for initial fix..."));
  uint32_t ttff_initial = 0;
  bool initial_fix = waitForFix(&ttff_initial);
  if (!initial_fix) {
    Serial.println(F("FAIL: No initial fix"));
  }

  Serial.print(F("Initial TTFF: "));
  Serial.print(ttff_initial);
  Serial.println(F(" ms"));

  bool hot_start_sent = ubx.hotStart();
  printTestResult(F("hot_start_sent"), hot_start_sent);
  Serial.println(hot_start_sent ? F("OK") : F("BAD"));
  if (hot_start_sent)
    passed++;

  Serial.println(F("Waiting for module reset..."));
  delay(3000);

  bool module_reconnected = reconnectModule(10000);
  printTestResult(F("module_reconnected"), module_reconnected);
  Serial.println(module_reconnected ? F("OK") : F("BAD"));
  if (module_reconnected)
    passed++;

  uint32_t ttff_hot = 0;
  bool fix_reacquired = false;
  if (module_reconnected) {
    Serial.println(F("Waiting for fix after hot start..."));
    fix_reacquired = waitForFix(&ttff_hot);
  }
  printTestResult(F("fix_reacquired"), fix_reacquired);
  Serial.println(fix_reacquired ? F("OK") : F("BAD"));
  if (fix_reacquired)
    passed++;

  bool ttff_reasonable = fix_reacquired && (ttff_hot < 30000);
  printTestResult(F("ttff_reasonable"), ttff_reasonable);
  Serial.print(ttff_hot);
  Serial.println(F(" ms"));
  if (ttff_reasonable)
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

  Serial.println(F("UBX-CFG-RST Message Test"));
  Serial.println(F("========================"));

  if (!initModule()) {
    Serial.println(F("FAIL: Could not connect to GPS module!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS module connected on I2C"));

  runTests();
  tests_run = true;
}

void loop() {
  delay(1000);
}
