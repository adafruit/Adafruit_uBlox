/*!
 * @file cfg_pms_test.ino
 *
 * Message test: Poll and set UBX-CFG-PMS (Power Mode Setup)
 * Tests various power modes without actually entering power save.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.println(name);
}

const char* getPowerModeName(uint8_t mode) {
  switch (mode) {
    case UBX_PMS_FULLPOWER: return "Full Power";
    case UBX_PMS_BALANCED: return "Balanced";
    case UBX_PMS_INTERVAL: return "Interval";
    case UBX_PMS_AGGRESSIVE_1HZ: return "Aggressive 1Hz";
    case UBX_PMS_AGGRESSIVE_2HZ: return "Aggressive 2Hz";
    case UBX_PMS_AGGRESSIVE_4HZ: return "Aggressive 4Hz";
    case UBX_PMS_INVALID: return "Invalid";
    default: return "Unknown";
  }
}

void printPmsDetails(UBX_CFG_PMS_t* pms) {
  Serial.println(F("\nCFG-PMS Details:"));
  Serial.print(F("  Version: "));
  Serial.println(pms->version);
  Serial.print(F("  Power mode: "));
  Serial.print(pms->powerSetupValue);
  Serial.print(F(" ("));
  Serial.print(getPowerModeName(pms->powerSetupValue));
  Serial.println(F(")"));
  Serial.print(F("  Period: "));
  Serial.print(pms->period);
  Serial.println(F(" s"));
  Serial.print(F("  On time: "));
  Serial.print(pms->onTime);
  Serial.println(F(" s"));
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running CFG-PMS tests..."));

  // Test 1: Poll current CFG-PMS
  UBX_CFG_PMS_t pms;
  bool poll_ok = ubx.pollCfgPms(&pms);
  printTestResult(F("poll_cfg_pms"), poll_ok);
  if (poll_ok) {
    passed++;
    printPmsDetails(&pms);
  }

  // Test 2: Check struct size
  bool size_ok = (sizeof(UBX_CFG_PMS_t) == 8);
  printTestResult(F("struct_size_8"), size_ok);
  if (size_ok) passed++;

  // Test 3: Set to Full Power mode (safe, doesn't change behavior)
  bool set_full_ok = ubx.setPowerMode(UBX_PMS_FULLPOWER);
  printTestResult(F("set_full_power"), set_full_ok);
  if (set_full_ok) passed++;

  // Verify it was set
  delay(100);
  if (ubx.pollCfgPms(&pms)) {
    Serial.print(F("    Current mode after set: "));
    Serial.println(getPowerModeName(pms.powerSetupValue));
  }

  // Test 4: Try setting Balanced mode (also safe)
  bool set_balanced_ok = ubx.setPowerMode(UBX_PMS_BALANCED);
  printTestResult(F("set_balanced"), set_balanced_ok);
  if (set_balanced_ok) passed++;

  // Restore to Full Power
  ubx.setPowerMode(UBX_PMS_FULLPOWER);

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

  Serial.println(F("UBX-CFG-PMS Message Test"));
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
}

void loop() {
  // Poll periodically
  delay(5000);

  UBX_CFG_PMS_t pms;
  if (ubx.pollCfgPms(&pms)) {
    Serial.print(F("CFG-PMS: mode="));
    Serial.print(getPowerModeName(pms.powerSetupValue));
    Serial.print(F(", period="));
    Serial.print(pms.period);
    Serial.print(F("s, onTime="));
    Serial.print(pms.onTime);
    Serial.println(F("s"));
  } else {
    Serial.println(F("CFG-PMS poll timeout"));
  }
}
