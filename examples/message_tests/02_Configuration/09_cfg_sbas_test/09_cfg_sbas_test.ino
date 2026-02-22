/*!
 * @file cfg_sbas_test.ino
 *
 * Message test: Poll UBX-CFG-SBAS, display SBAS settings,
 * toggle enable, verify.
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

void printSbasConfig(UBX_CFG_SBAS_t* sbas) {
  Serial.print(F("  mode: 0x"));
  Serial.println(sbas->mode, HEX);
  Serial.print(F("    enabled: "));
  Serial.println((sbas->mode & UBX_SBAS_MODE_ENABLED) ? F("yes") : F("no"));
  Serial.print(F("    test: "));
  Serial.println((sbas->mode & UBX_SBAS_MODE_TEST) ? F("yes") : F("no"));

  Serial.print(F("  usage: 0x"));
  Serial.println(sbas->usage, HEX);
  Serial.print(F("    range: "));
  Serial.println((sbas->usage & UBX_SBAS_USAGE_RANGE) ? F("yes") : F("no"));
  Serial.print(F("    diffCorr: "));
  Serial.println((sbas->usage & UBX_SBAS_USAGE_DIFFCORR) ? F("yes") : F("no"));
  Serial.print(F("    integrity: "));
  Serial.println((sbas->usage & UBX_SBAS_USAGE_INTEGRITY) ? F("yes") : F("no"));

  Serial.print(F("  maxSBAS: "));
  Serial.println(sbas->maxSBAS);
  Serial.print(F("  scanmode2: 0x"));
  Serial.println(sbas->scanmode2, HEX);
  Serial.print(F("  scanmode1: 0x"));
  Serial.println(sbas->scanmode1, HEX);
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running tests..."));

  // Test 1: Poll CFG-SBAS
  UBX_CFG_SBAS_t sbas;
  bool poll_ok = ubx.pollCfgSbas(&sbas);
  printTestResult(F("poll_cfg_sbas"), poll_ok);
  Serial.println(poll_ok ? F("OK") : F("FAILED"));
  if (poll_ok) {
    passed++;
    printSbasConfig(&sbas);
  }

  // Test 2: Verify struct size
  bool size_ok = (sizeof(UBX_CFG_SBAS_t) == 8);
  printTestResult(F("struct_size"), size_ok);
  Serial.print(sizeof(UBX_CFG_SBAS_t));
  Serial.println(F(" bytes"));
  if (size_ok) passed++;

  // Save original state
  bool original_enabled = (sbas.mode & UBX_SBAS_MODE_ENABLED) != 0;
  Serial.print(F("  Original SBAS state: "));
  Serial.println(original_enabled ? F("enabled") : F("disabled"));

  // Test 3: Toggle SBAS
  bool toggle_ok = ubx.enableSBAS(!original_enabled);
  printTestResult(F("toggle_sbas"), toggle_ok);
  Serial.println(toggle_ok ? F("OK") : F("FAILED"));
  if (toggle_ok) passed++;

  delay(100);

  // Test 4: Verify change and restore
  UBX_CFG_SBAS_t sbas_verify;
  bool verify_ok = ubx.pollCfgSbas(&sbas_verify);
  if (verify_ok) {
    bool new_state = (sbas_verify.mode & UBX_SBAS_MODE_ENABLED) != 0;
    bool changed = (new_state != original_enabled);
    printTestResult(F("verify_toggle"), changed);
    Serial.print(F("now "));
    Serial.println(new_state ? F("enabled") : F("disabled"));
    if (changed) passed++;

    // Restore original
    ubx.enableSBAS(original_enabled);
  } else {
    printTestResult(F("verify_toggle"), false);
    Serial.println(F("poll failed"));
  }

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

  Serial.println(F("UBX-CFG-SBAS Message Test"));
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
