/*!
 * @file nav_clock_test.ino
 *
 * Message test: Poll UBX-NAV-CLOCK, wait for fix, validate fields,
 * then stream parsed data for inspection.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

static const uint32_t FIX_TIMEOUT_MS = 120000;

bool tests_run = false;
bool fix_acquired = false;
bool printed_continuous_header = false;
uint32_t attempt = 0;
unsigned long fix_start_ms = 0;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  Serial.print(pass ? F("PASS") : F("FAIL"));
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

uint8_t runTests(const UBX_NAV_CLOCK_t& clk) {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running NAV-CLOCK tests..."));

  // iTOW should be non-zero
  bool itow_ok = clk.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.println(clk.iTOW);
  if (itow_ok)
    passed++;

  // Clock bias could be anything, just check it's not extreme
  // Typical values are in the microseconds range (thousands of ns)
  bool clkB_ok = clk.clkB > -1000000000L && clk.clkB < 1000000000L;
  printTestResult(F("clkB_reasonable"), clkB_ok);
  Serial.print(clk.clkB);
  Serial.println(F(" ns"));
  if (clkB_ok)
    passed++;

  // Clock drift should be reasonable (< 1 ms/s = 1e6 ns/s)
  bool clkD_ok = clk.clkD > -1000000 && clk.clkD < 1000000;
  printTestResult(F("clkD_reasonable"), clkD_ok);
  Serial.print(clk.clkD);
  Serial.println(F(" ns/s"));
  if (clkD_ok)
    passed++;

  // Time accuracy should be < 1 second (1e9 ns)
  bool tAcc_ok = clk.tAcc < 1000000000;
  printTestResult(F("tAcc_reasonable"), tAcc_ok);
  Serial.print(clk.tAcc);
  Serial.println(F(" ns"));
  if (tAcc_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printCLOCK(const UBX_NAV_CLOCK_t& clk) {
  Serial.println(F("--- NAV-CLOCK ---"));

  Serial.print(F("iTOW: "));
  Serial.print(clk.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("clkB: "));
  Serial.print(clk.clkB);
  Serial.println(F(" ns"));

  Serial.print(F("clkD: "));
  Serial.print(clk.clkD);
  Serial.println(F(" ns/s"));

  Serial.print(F("tAcc: "));
  Serial.print(clk.tAcc);
  Serial.println(F(" ns"));

  Serial.print(F("fAcc: "));
  Serial.print(clk.fAcc);
  Serial.println(F(" ps/s"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-CLOCK Message Test ==="));

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
  fix_start_ms = millis();
}

void loop() {
  UBX_NAV_PVT_t pvt;
  UBX_NAV_CLOCK_t clk;

  if (!tests_run) {
    attempt++;
    bool got = ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt));

    if (attempt == 1) {
      Serial.println(F("Waiting for 3D fix..."));
    } else {
      Serial.print(F("Waiting for fix... (attempt "));
      Serial.print(attempt);
      if (got) {
        Serial.print(F(", fixType="));
        Serial.print(pvt.fixType);
        Serial.print(F(", sats="));
        Serial.print(pvt.numSV);
        Serial.println(F(")"));
      } else {
        Serial.println(F(", poll failed)"));
      }
    }

    if (got && pvt.fixType == 3 && (pvt.flags & 0x01)) {
      fix_acquired = true;
      Serial.print(F("Fix acquired! fixType="));
      Serial.print(pvt.fixType);
      Serial.print(F(", sats="));
      Serial.println(pvt.numSV);

      if (ubx.pollNAVCLOCK(&clk)) {
        runTests(clk);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-CLOCK after fix!"));
      }
      tests_run = true;
      printed_continuous_header = false;
      delay(2000);
      return;
    }

    if (millis() - fix_start_ms > FIX_TIMEOUT_MS) {
      Serial.println(F("Timeout waiting for 3D fix after 120 seconds."));
      tests_run = true;
    }

    delay(1000);
    return;
  }

  if (!printed_continuous_header) {
    Serial.println();
    Serial.println(F("Continuous output:"));
    printed_continuous_header = true;
  }

  if (ubx.pollNAVCLOCK(&clk)) {
    printCLOCK(clk);
  } else {
    Serial.println(F("NAV-CLOCK poll failed (timeout)"));
  }

  delay(2000);
}
