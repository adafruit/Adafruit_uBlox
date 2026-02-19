/*!
 * @file mon_rxbuf_test.ino
 *
 * Message test: Poll UBX-MON-RXBUF (Receiver Buffer Status)
 * Displays RX buffer pending, usage, and peak usage per port.
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

void printRxbufDetails(UBX_MON_RXBUF_t* rxbuf) {
  Serial.println(F("\nMON-RXBUF Details:"));
  Serial.println(F("  Port      Pending   Usage%  Peak%"));

  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(F("  "));
    Serial.print(getPortName(i));
    if (strlen(getPortName(i)) < 5) Serial.print(F("\t"));
    Serial.print(F("\t"));

    if (rxbuf->pending[i] < 10000) Serial.print(F(" "));
    if (rxbuf->pending[i] < 1000) Serial.print(F(" "));
    if (rxbuf->pending[i] < 100) Serial.print(F(" "));
    if (rxbuf->pending[i] < 10) Serial.print(F(" "));
    Serial.print(rxbuf->pending[i]);

    Serial.print(F("\t  "));
    if (rxbuf->usage[i] < 100) Serial.print(F(" "));
    if (rxbuf->usage[i] < 10) Serial.print(F(" "));
    Serial.print(rxbuf->usage[i]);

    Serial.print(F("\t  "));
    if (rxbuf->peakUsage[i] < 100) Serial.print(F(" "));
    if (rxbuf->peakUsage[i] < 10) Serial.print(F(" "));
    Serial.println(rxbuf->peakUsage[i]);
  }
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running MON-RXBUF tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_MON_RXBUF_t) == 24);
  printTestResult(F("struct_size_24"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll MON-RXBUF
  UBX_MON_RXBUF_t rxbuf;
  bool poll_ok = ubx.pollMonRxbuf(&rxbuf);
  printTestResult(F("poll_mon_rxbuf"), poll_ok);
  if (poll_ok) {
    passed++;
    printRxbufDetails(&rxbuf);
  }

  // Test 3: Verify usage values are valid (0-100%)
  bool usage_valid = true;
  if (poll_ok) {
    for (uint8_t i = 0; i < 6; i++) {
      if (rxbuf.usage[i] > 100 || rxbuf.peakUsage[i] > 100) {
        usage_valid = false;
        break;
      }
    }
  } else {
    usage_valid = false;
  }
  printTestResult(F("usage_valid_range"), usage_valid);
  if (usage_valid) passed++;

  // Test 4: Peak should be >= current usage
  bool peak_ok = true;
  if (poll_ok) {
    for (uint8_t i = 0; i < 6; i++) {
      if (rxbuf.peakUsage[i] < rxbuf.usage[i]) {
        peak_ok = false;
        break;
      }
    }
  } else {
    peak_ok = false;
  }
  printTestResult(F("peak_gte_usage"), peak_ok);
  if (peak_ok) passed++;

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

  Serial.println(F("UBX-MON-RXBUF Message Test"));
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
  Serial.println(F("Streaming RX buffer status..."));
}

void loop() {
  delay(2000);

  UBX_MON_RXBUF_t rxbuf;
  if (ubx.pollMonRxbuf(&rxbuf)) {
    Serial.print(F("MON-RXBUF: DDC pend="));
    Serial.print(rxbuf.pending[0]);
    Serial.print(F(", use="));
    Serial.print(rxbuf.usage[0]);
    Serial.print(F("%, peak="));
    Serial.print(rxbuf.peakUsage[0]);
    Serial.println(F("%"));
  } else {
    Serial.println(F("MON-RXBUF poll timeout"));
  }
}
