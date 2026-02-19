/*!
 * @file mon_hw2_test.ino
 *
 * Message test: Poll UBX-MON-HW2 (Extended Hardware Status)
 * Displays IQ imbalance, config source, and POST status.
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

const char* getConfigSourceName(uint8_t src) {
  switch (src) {
    case UBX_MON_HW2_CFG_ROM: return "ROM";
    case UBX_MON_HW2_CFG_OTP: return "OTP";
    case UBX_MON_HW2_CFG_PINS: return "Pins";
    case UBX_MON_HW2_CFG_FLASH: return "Flash";
    default: return "Unknown";
  }
}

void printHw2Details(UBX_MON_HW2_t* hw2) {
  Serial.println(F("\nMON-HW2 Details:"));

  Serial.println(F("  IQ Imbalance:"));
  Serial.print(F("    ofsI: "));
  Serial.print(hw2->ofsI);
  Serial.print(F(", magI: "));
  Serial.println(hw2->magI);
  Serial.print(F("    ofsQ: "));
  Serial.print(hw2->ofsQ);
  Serial.print(F(", magQ: "));
  Serial.println(hw2->magQ);

  Serial.print(F("  Config source: "));
  Serial.print(hw2->cfgSource);
  Serial.print(F(" ("));
  Serial.print(getConfigSourceName(hw2->cfgSource));
  Serial.println(F(")"));

  Serial.print(F("  Low-level config: 0x"));
  Serial.println(hw2->lowLevCfg, HEX);

  Serial.print(F("  POST status: 0x"));
  Serial.println(hw2->postStatus, HEX);
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running MON-HW2 tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_MON_HW2_t) == 28);
  printTestResult(F("struct_size_28"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll MON-HW2
  UBX_MON_HW2_t hw2;
  bool poll_ok = ubx.pollMonHw2(&hw2);
  printTestResult(F("poll_mon_hw2"), poll_ok);
  if (poll_ok) {
    passed++;
    printHw2Details(&hw2);
  }

  // Test 3: Verify config source is valid (known values)
  bool cfg_valid = poll_ok && (hw2.cfgSource == UBX_MON_HW2_CFG_ROM ||
                               hw2.cfgSource == UBX_MON_HW2_CFG_OTP ||
                               hw2.cfgSource == UBX_MON_HW2_CFG_PINS ||
                               hw2.cfgSource == UBX_MON_HW2_CFG_FLASH);
  printTestResult(F("config_source_valid"), cfg_valid);
  if (cfg_valid) passed++;

  // Test 4: Verify magnitude values are sane (at least one should be > 0)
  bool mag_ok = poll_ok && (hw2.magI > 0 || hw2.magQ > 0);
  printTestResult(F("iq_magnitude_nonzero"), mag_ok);
  if (mag_ok) passed++;

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

  Serial.println(F("UBX-MON-HW2 Message Test"));
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
  Serial.println();
  Serial.println(F("Streaming extended HW status..."));
}

void loop() {
  delay(2000);

  UBX_MON_HW2_t hw2;
  if (ubx.pollMonHw2(&hw2)) {
    Serial.print(F("MON-HW2: I("));
    Serial.print(hw2.ofsI);
    Serial.print(F(","));
    Serial.print(hw2.magI);
    Serial.print(F(") Q("));
    Serial.print(hw2.ofsQ);
    Serial.print(F(","));
    Serial.print(hw2.magQ);
    Serial.print(F(") cfg="));
    Serial.println(getConfigSourceName(hw2.cfgSource));
  } else {
    Serial.println(F("MON-HW2 poll timeout"));
  }
}
