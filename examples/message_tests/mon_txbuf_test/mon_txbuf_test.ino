/*!
 * @file mon_txbuf_test.ino
 *
 * Message test: Poll UBX-MON-TXBUF (Transmitter Buffer Status)
 * Displays TX buffer pending, usage, peak, and error flags.
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

const char* getPortName(uint8_t port) {
  switch (port) {
    case 0: return "DDC";
    case 1: return "UART1";
    case 2: return "UART2";
    case 3: return "USB";
    case 4: return "SPI";
    case 5: return "Port5";
    default: return "???";
  }
}

void printTxbufDetails(UBX_MON_TXBUF_t* txbuf) {
  Serial.println(F("\nMON-TXBUF Details:"));
  Serial.println(F("  Port      Pending   Usage%  Peak%"));

  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(F("  "));
    Serial.print(getPortName(i));
    if (strlen(getPortName(i)) < 5) Serial.print(F("\t"));
    Serial.print(F("\t"));

    if (txbuf->pending[i] < 10000) Serial.print(F(" "));
    if (txbuf->pending[i] < 1000) Serial.print(F(" "));
    if (txbuf->pending[i] < 100) Serial.print(F(" "));
    if (txbuf->pending[i] < 10) Serial.print(F(" "));
    Serial.print(txbuf->pending[i]);

    Serial.print(F("\t  "));
    if (txbuf->usage[i] < 100) Serial.print(F(" "));
    if (txbuf->usage[i] < 10) Serial.print(F(" "));
    Serial.print(txbuf->usage[i]);

    Serial.print(F("\t  "));
    if (txbuf->peakUsage[i] < 100) Serial.print(F(" "));
    if (txbuf->peakUsage[i] < 10) Serial.print(F(" "));
    Serial.println(txbuf->peakUsage[i]);
  }

  Serial.println(F("\n  Totals:"));
  Serial.print(F("    Total usage: "));
  Serial.print(txbuf->tUsage);
  Serial.println(F("%"));
  Serial.print(F("    Total peak: "));
  Serial.print(txbuf->tPeakUsage);
  Serial.println(F("%"));

  Serial.println(F("\n  Error flags:"));
  Serial.print(F("    Limit reached: "));
  Serial.println((txbuf->errors & UBX_MON_TXBUF_ERR_LIMIT) ? F("YES") : F("no"));
  Serial.print(F("    Memory error: "));
  Serial.println((txbuf->errors & UBX_MON_TXBUF_ERR_MEM) ? F("YES") : F("no"));
  Serial.print(F("    Alloc error: "));
  Serial.println((txbuf->errors & UBX_MON_TXBUF_ERR_ALLOC) ? F("YES") : F("no"));
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running MON-TXBUF tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_MON_TXBUF_t) == 28);
  printTestResult(F("struct_size_28"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll MON-TXBUF
  UBX_MON_TXBUF_t txbuf;
  bool poll_ok = ubx.pollMonTxbuf(&txbuf);
  printTestResult(F("poll_mon_txbuf"), poll_ok);
  if (poll_ok) {
    passed++;
    printTxbufDetails(&txbuf);
  }

  // Test 3: Verify usage values are valid (0-100%)
  bool usage_valid = true;
  if (poll_ok) {
    for (uint8_t i = 0; i < 6; i++) {
      if (txbuf.usage[i] > 100 || txbuf.peakUsage[i] > 100) {
        usage_valid = false;
        break;
      }
    }
    if (txbuf.tUsage > 100 || txbuf.tPeakUsage > 100) {
      usage_valid = false;
    }
  } else {
    usage_valid = false;
  }
  printTestResult(F("usage_valid_range"), usage_valid);
  if (usage_valid) passed++;

  // Test 4: No errors present (normal operation)
  bool no_errors = poll_ok && (txbuf.errors == 0);
  printTestResult(F("no_errors"), no_errors);
  if (no_errors) passed++;

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

  Serial.println(F("UBX-MON-TXBUF Message Test"));
  Serial.println(F("==========================="));

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
  Serial.println(F("Streaming TX buffer status..."));
}

void loop() {
  delay(2000);

  UBX_MON_TXBUF_t txbuf;
  if (ubx.pollMonTxbuf(&txbuf)) {
    Serial.print(F("MON-TXBUF: DDC pend="));
    Serial.print(txbuf.pending[0]);
    Serial.print(F(", total="));
    Serial.print(txbuf.tUsage);
    Serial.print(F("%, peak="));
    Serial.print(txbuf.tPeakUsage);
    Serial.print(F("%, err=0x"));
    Serial.println(txbuf.errors, HEX);
  } else {
    Serial.println(F("MON-TXBUF poll timeout"));
  }
}
