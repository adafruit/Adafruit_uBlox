/*!
 * @file cfg_rxm_test.ino
 *
 * Message test: Poll UBX-CFG-RXM, toggle power save mode.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

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

void printRxm(UBX_CFG_RXM_t* rxm) {
  Serial.print(F("  reserved1: "));
  Serial.println(rxm->reserved1);
  Serial.print(F("  lpMode: "));
  Serial.print(rxm->lpMode);
  Serial.print(F(" ("));
  if (rxm->lpMode == UBX_RXM_LPMODE_CONTINUOUS) {
    Serial.print(F("Continuous"));
  } else if (rxm->lpMode == UBX_RXM_LPMODE_POWERSAVE) {
    Serial.print(F("Power Save"));
  } else {
    Serial.print(F("Unknown"));
  }
  Serial.println(F(")"));
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running CFG-RXM tests..."));

  // Test 1: Poll current settings
  UBX_CFG_RXM_t rxm;
  bool poll_ok = ubx.pollCfgRxm(&rxm);
  printTestResult(F("poll_rxm"), poll_ok);
  Serial.println(poll_ok ? F("OK") : F("FAIL"));
  if (poll_ok) {
    passed++;
    Serial.println(F("Current RXM settings:"));
    printRxm(&rxm);
  }

  // Save original mode
  uint8_t originalMode = rxm.lpMode;

  // Test 2: Set continuous mode (ensure known state)
  bool set_continuous = ubx.setPowerSave(false);
  delay(100);
  UBX_CFG_RXM_t verify;
  ubx.pollCfgRxm(&verify);
  bool verify_continuous = (verify.lpMode == UBX_RXM_LPMODE_CONTINUOUS);
  printTestResult(F("set_continuous"), set_continuous && verify_continuous);
  Serial.println(verify_continuous ? F("lpMode=0") : F("failed"));
  if (set_continuous && verify_continuous)
    passed++;

  // Test 3: Set power save mode
  bool set_powersave = ubx.setPowerSave(true);
  delay(100);
  ubx.pollCfgRxm(&verify);
  bool verify_powersave = (verify.lpMode == UBX_RXM_LPMODE_POWERSAVE);
  printTestResult(F("set_powersave"), set_powersave && verify_powersave);
  Serial.println(verify_powersave ? F("lpMode=1") : F("failed"));
  if (set_powersave && verify_powersave)
    passed++;

  // Test 4: Restore original mode
  bool restore = ubx.setPowerSave(originalMode == UBX_RXM_LPMODE_POWERSAVE);
  delay(100);
  ubx.pollCfgRxm(&verify);
  bool verify_restore = (verify.lpMode == originalMode);
  printTestResult(F("restore"), restore && verify_restore);
  Serial.print(F("lpMode="));
  Serial.println(verify.lpMode);
  if (restore && verify_restore)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  Serial.println();
  Serial.println(F("NOTE: Power save mode requires additional CFG-PM2"));
  Serial.println(F("configuration for full effect."));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-CFG-RXM Message Test"));
  Serial.println(F("========================="));

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

  delay(500);

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  runTests();
  tests_run = true;
}

void loop() {
  delay(1000);
}
