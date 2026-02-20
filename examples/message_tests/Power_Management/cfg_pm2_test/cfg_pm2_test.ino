/*!
 * @file cfg_pm2_test.ino
 *
 * Message test: Poll UBX-CFG-PM2 (Extended Power Management Configuration)
 * and display current settings.
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

void printPm2Details(UBX_CFG_PM2_t* pm2) {
  Serial.println(F("\nCFG-PM2 Details:"));
  Serial.print(F("  Version: "));
  Serial.println(pm2->version);
  Serial.print(F("  Max startup state duration: "));
  Serial.print(pm2->maxStartupStateDur);
  Serial.println(F(" s"));
  Serial.print(F("  Flags: 0x"));
  Serial.println(pm2->flags, HEX);
  Serial.print(F("  Update period: "));
  Serial.print(pm2->updatePeriod);
  Serial.println(F(" ms"));
  Serial.print(F("  Search period: "));
  Serial.print(pm2->searchPeriod);
  Serial.println(F(" ms"));
  Serial.print(F("  Grid offset: "));
  Serial.print(pm2->gridOffset);
  Serial.println(F(" ms"));
  Serial.print(F("  On time: "));
  Serial.print(pm2->onTime);
  Serial.println(F(" s"));
  Serial.print(F("  Min acquisition time: "));
  Serial.print(pm2->minAcqTime);
  Serial.println(F(" s"));

  // Decode flags
  Serial.println(F("\n  Flag bits:"));
  Serial.print(F("    EXTINT select: "));
  Serial.println((pm2->flags & UBX_PM2_FLAG_EXTINTSEL) ? F("EXTINT1") : F("EXTINT0"));
  Serial.print(F("    EXTINT wake: "));
  Serial.println((pm2->flags & UBX_PM2_FLAG_EXTINTWAKE) ? F("enabled") : F("disabled"));
  Serial.print(F("    EXTINT backup: "));
  Serial.println((pm2->flags & UBX_PM2_FLAG_EXTINTBACKUP) ? F("enabled") : F("disabled"));
  Serial.print(F("    Wait for time fix: "));
  Serial.println((pm2->flags & UBX_PM2_FLAG_WAITTIMEFIX) ? F("yes") : F("no"));
  Serial.print(F("    Update RTC: "));
  Serial.println((pm2->flags & UBX_PM2_FLAG_UPDATERTC) ? F("yes") : F("no"));
  Serial.print(F("    Update EPH: "));
  Serial.println((pm2->flags & UBX_PM2_FLAG_UPDATEEPH) ? F("yes") : F("no"));
  Serial.print(F("    Mode: "));
  Serial.println((pm2->flags & UBX_PM2_FLAG_MODE_CYCLIC) ? F("Cyclic (PSMCT)") : F("ON/OFF (PSMOO)"));
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 2;

  Serial.println();
  Serial.println(F("Running CFG-PM2 tests..."));

  // Test 1: Poll CFG-PM2
  UBX_CFG_PM2_t pm2;
  bool poll_ok = ubx.pollCfgPm2(&pm2);
  printTestResult(F("poll_cfg_pm2"), poll_ok);
  if (poll_ok) {
    passed++;
    printPm2Details(&pm2);
  }

  // Test 2: Check struct size
  bool size_ok = (sizeof(UBX_CFG_PM2_t) == 44);
  printTestResult(F("struct_size_44"), size_ok);
  if (size_ok) passed++;

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

  Serial.println(F("UBX-CFG-PM2 Message Test"));
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
  // Poll periodically to show power management state
  delay(5000);

  UBX_CFG_PM2_t pm2;
  if (ubx.pollCfgPm2(&pm2)) {
    Serial.print(F("CFG-PM2: updatePeriod="));
    Serial.print(pm2.updatePeriod);
    Serial.print(F("ms, onTime="));
    Serial.print(pm2.onTime);
    Serial.println(F("s"));
  } else {
    Serial.println(F("CFG-PM2 poll timeout"));
  }
}
