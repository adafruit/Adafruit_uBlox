/*!
 * @file cfg_ant_test.ino
 *
 * Message test: Poll UBX-CFG-ANT, display antenna settings.
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

void printAntConfig(UBX_CFG_ANT_t* ant) {
  Serial.print(F("  flags: 0x"));
  Serial.println(ant->flags, HEX);
  Serial.print(F("    svcs (supply voltage control): "));
  Serial.println((ant->flags & UBX_ANT_FLAG_SVCS) ? F("enabled") : F("disabled"));
  Serial.print(F("    scd (short circuit detect): "));
  Serial.println((ant->flags & UBX_ANT_FLAG_SCD) ? F("enabled") : F("disabled"));
  Serial.print(F("    ocd (open circuit detect): "));
  Serial.println((ant->flags & UBX_ANT_FLAG_OCD) ? F("enabled") : F("disabled"));
  Serial.print(F("    pdwnOnSCD: "));
  Serial.println((ant->flags & UBX_ANT_FLAG_PDWNONSCD) ? F("enabled") : F("disabled"));
  Serial.print(F("    recovery: "));
  Serial.println((ant->flags & UBX_ANT_FLAG_RECOVERY) ? F("enabled") : F("disabled"));

  Serial.print(F("  pins: 0x"));
  Serial.println(ant->pins, HEX);
  Serial.print(F("    pinSwitch: "));
  Serial.println(ant->pins & 0x1F);
  Serial.print(F("    pinSCD: "));
  Serial.println((ant->pins >> 5) & 0x1F);
  Serial.print(F("    pinOCD: "));
  Serial.println((ant->pins >> 10) & 0x1F);
  Serial.print(F("    reconfig: "));
  Serial.println((ant->pins >> 15) & 0x01);
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 3;

  Serial.println();
  Serial.println(F("Running tests..."));

  // Test 1: Poll CFG-ANT
  UBX_CFG_ANT_t ant;
  bool poll_ok = ubx.pollCfgAnt(&ant);
  printTestResult(F("poll_cfg_ant"), poll_ok);
  Serial.println(poll_ok ? F("OK") : F("FAILED"));
  if (poll_ok) {
    passed++;
    printAntConfig(&ant);
  }

  // Test 2: Verify struct size
  bool size_ok = (sizeof(UBX_CFG_ANT_t) == 4);
  printTestResult(F("struct_size"), size_ok);
  Serial.print(sizeof(UBX_CFG_ANT_t));
  Serial.println(F(" bytes"));
  if (size_ok) passed++;

  // Test 3: Set and verify (toggle recovery flag)
  if (poll_ok) {
    uint16_t original_flags = ant.flags;
    ant.flags ^= UBX_ANT_FLAG_RECOVERY;
    bool set_ok = ubx.setCfgAnt(&ant);
    printTestResult(F("set_cfg_ant"), set_ok);
    Serial.println(set_ok ? F("OK") : F("FAILED"));
    if (set_ok) passed++;

    // Restore original
    ant.flags = original_flags;
    ubx.setCfgAnt(&ant);
  } else {
    printTestResult(F("set_cfg_ant"), false);
    Serial.println(F("skipped (poll failed)"));
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

  Serial.println(F("UBX-CFG-ANT Message Test"));
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
