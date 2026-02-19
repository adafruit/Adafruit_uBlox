/*!
 * @file mon_hw_test.ino
 *
 * Message test: Poll UBX-MON-HW (Hardware Status)
 * Displays antenna status, noise level, AGC, and jamming indicator.
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

const char* getAntennaStatusName(uint8_t status) {
  switch (status) {
    case UBX_MON_HW_ASTATUS_INIT: return "INIT";
    case UBX_MON_HW_ASTATUS_DONTKNOW: return "UNKNOWN";
    case UBX_MON_HW_ASTATUS_OK: return "OK";
    case UBX_MON_HW_ASTATUS_SHORT: return "SHORT";
    case UBX_MON_HW_ASTATUS_OPEN: return "OPEN";
    default: return "???";
  }
}

const char* getAntennaPowerName(uint8_t power) {
  switch (power) {
    case UBX_MON_HW_APOWER_OFF: return "OFF";
    case UBX_MON_HW_APOWER_ON: return "ON";
    case UBX_MON_HW_APOWER_DONTKNOW: return "UNKNOWN";
    default: return "???";
  }
}

void printHwDetails(UBX_MON_HW_t* hw) {
  Serial.println(F("\nMON-HW Details:"));

  // Antenna info
  Serial.println(F("  Antenna:"));
  Serial.print(F("    Status: "));
  Serial.print(hw->aStatus);
  Serial.print(F(" ("));
  Serial.print(getAntennaStatusName(hw->aStatus));
  Serial.println(F(")"));
  Serial.print(F("    Power: "));
  Serial.print(hw->aPower);
  Serial.print(F(" ("));
  Serial.print(getAntennaPowerName(hw->aPower));
  Serial.println(F(")"));

  // Noise and AGC
  Serial.println(F("  Signal quality:"));
  Serial.print(F("    Noise per MS: "));
  Serial.println(hw->noisePerMS);
  Serial.print(F("    AGC count: "));
  Serial.print(hw->agcCnt);
  Serial.print(F(" ("));
  // AGC is 0-8191 representing 0-100%
  Serial.print((hw->agcCnt * 100) / 8191);
  Serial.println(F("%)"));
  Serial.print(F("    Jamming indicator: "));
  Serial.print(hw->jamInd);
  Serial.print(F(" ("));
  if (hw->jamInd == 0) {
    Serial.print(F("none"));
  } else if (hw->jamInd < 80) {
    Serial.print(F("low"));
  } else if (hw->jamInd < 160) {
    Serial.print(F("medium"));
  } else {
    Serial.print(F("HIGH"));
  }
  Serial.println(F(")"));

  // Flags
  Serial.println(F("  Flags:"));
  Serial.print(F("    RTC calibrated: "));
  Serial.println((hw->flags & UBX_MON_HW_FLAG_RTCCALIB) ? F("yes") : F("no"));
  Serial.print(F("    Safe boot: "));
  Serial.println((hw->flags & UBX_MON_HW_FLAG_SAFEBOOT) ? F("active") : F("inactive"));
  Serial.print(F("    Crystal absent: "));
  Serial.println((hw->flags & UBX_MON_HW_FLAG_XTALABSENT) ? F("yes") : F("no"));

  // Jamming state from flags (deprecated but still available)
  uint8_t jamState = (hw->flags & UBX_MON_HW_FLAG_JAMSTATE_MASK) >> 2;
  Serial.print(F("    Jamming state: "));
  switch (jamState) {
    case 0: Serial.println(F("unknown/disabled")); break;
    case 1: Serial.println(F("OK")); break;
    case 2: Serial.println(F("warning")); break;
    case 3: Serial.println(F("CRITICAL")); break;
    default: Serial.println(F("???")); break;
  }

  // Pin masks (brief summary)
  Serial.println(F("  Pin masks:"));
  Serial.print(F("    Used: 0x"));
  Serial.println(hw->usedMask, HEX);
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 3;

  Serial.println();
  Serial.println(F("Running MON-HW tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_MON_HW_t) == 60);
  printTestResult(F("struct_size_60"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll MON-HW
  UBX_MON_HW_t hw;
  bool poll_ok = ubx.pollMonHw(&hw);
  printTestResult(F("poll_mon_hw"), poll_ok);
  if (poll_ok) {
    passed++;
    printHwDetails(&hw);
  }

  // Test 3: Verify antenna status is valid (0-4)
  bool status_valid = poll_ok && (hw.aStatus <= 4);
  printTestResult(F("antenna_status_valid"), status_valid);
  if (status_valid) passed++;

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

  Serial.println(F("UBX-MON-HW Message Test"));
  Serial.println(F("========================"));

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
  Serial.println(F("Streaming hardware status..."));
}

void loop() {
  delay(2000);

  UBX_MON_HW_t hw;
  if (ubx.pollMonHw(&hw)) {
    Serial.print(F("MON-HW: ant="));
    Serial.print(getAntennaStatusName(hw.aStatus));
    Serial.print(F(", noise="));
    Serial.print(hw.noisePerMS);
    Serial.print(F(", AGC="));
    Serial.print((hw.agcCnt * 100) / 8191);
    Serial.print(F("%, jam="));
    Serial.println(hw.jamInd);
  } else {
    Serial.println(F("MON-HW poll timeout"));
  }
}
