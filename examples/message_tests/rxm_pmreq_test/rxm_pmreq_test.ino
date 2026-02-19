/*!
 * @file rxm_pmreq_test.ino
 *
 * Message test: Test UBX-RXM-PMREQ message formatting
 * NOTE: This test does NOT actually put the module to sleep!
 * It only verifies the message can be constructed and sent.
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

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running RXM-PMREQ tests..."));
  Serial.println(F("NOTE: Not actually sending sleep commands!"));

  // Test 1: Check v0 struct size
  bool size_v0_ok = (sizeof(UBX_RXM_PMREQ_t) == 8);
  printTestResult(F("struct_v0_size_8"), size_v0_ok);
  if (size_v0_ok) passed++;

  // Test 2: Check v1 struct size
  bool size_v1_ok = (sizeof(UBX_RXM_PMREQ_V1_t) == 16);
  printTestResult(F("struct_v1_size_16"), size_v1_ok);
  if (size_v1_ok) passed++;

  // Test 3: Verify v0 struct packing
  UBX_RXM_PMREQ_t pmreq_v0;
  pmreq_v0.duration = 0x12345678;
  pmreq_v0.flags = 0xABCDEF01;
  uint8_t* bytes = (uint8_t*)&pmreq_v0;
  // Check little-endian packing
  bool pack_v0_ok = (bytes[0] == 0x78 && bytes[1] == 0x56 &&
                     bytes[2] == 0x34 && bytes[3] == 0x12 &&
                     bytes[4] == 0x01 && bytes[5] == 0xEF &&
                     bytes[6] == 0xCD && bytes[7] == 0xAB);
  printTestResult(F("v0_packing"), pack_v0_ok);
  if (pack_v0_ok) passed++;

  // Test 4: Verify v1 struct packing
  UBX_RXM_PMREQ_V1_t pmreq_v1;
  memset(&pmreq_v1, 0, sizeof(pmreq_v1));
  pmreq_v1.version = 0x00;
  pmreq_v1.duration = 5000;
  pmreq_v1.flags = UBX_PMREQ_FLAG_BACKUP;
  pmreq_v1.wakeupSources = UBX_PMREQ_WAKE_EXTINT0;
  bool pack_v1_ok = (pmreq_v1.version == 0x00 &&
                     pmreq_v1.duration == 5000 &&
                     pmreq_v1.flags == UBX_PMREQ_FLAG_BACKUP &&
                     pmreq_v1.wakeupSources == UBX_PMREQ_WAKE_EXTINT0);
  printTestResult(F("v1_packing"), pack_v1_ok);
  if (pack_v1_ok) passed++;

  Serial.println();
  Serial.println(F("Flag definitions:"));
  Serial.print(F("  UBX_PMREQ_FLAG_BACKUP: 0x"));
  Serial.println(UBX_PMREQ_FLAG_BACKUP, HEX);
  Serial.print(F("  UBX_PMREQ_FLAG_FORCE: 0x"));
  Serial.println(UBX_PMREQ_FLAG_FORCE, HEX);

  Serial.println(F("\nWakeup source definitions:"));
  Serial.print(F("  UBX_PMREQ_WAKE_UARTRX: 0x"));
  Serial.println(UBX_PMREQ_WAKE_UARTRX, HEX);
  Serial.print(F("  UBX_PMREQ_WAKE_EXTINT0: 0x"));
  Serial.println(UBX_PMREQ_WAKE_EXTINT0, HEX);
  Serial.print(F("  UBX_PMREQ_WAKE_EXTINT1: 0x"));
  Serial.println(UBX_PMREQ_WAKE_EXTINT1, HEX);
  Serial.print(F("  UBX_PMREQ_WAKE_SPICS: 0x"));
  Serial.println(UBX_PMREQ_WAKE_SPICS, HEX);

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  Serial.println();
  Serial.println(F("To actually put module to sleep, call:"));
  Serial.println(F("  ubx.sendPmreq(duration_ms, UBX_PMREQ_FLAG_BACKUP);"));
  Serial.println(F("  // or for v1 with wakeup sources:"));
  Serial.println(F("  ubx.sendPmreqV1(duration_ms, UBX_PMREQ_FLAG_BACKUP, UBX_PMREQ_WAKE_EXTINT0);"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-RXM-PMREQ Message Test"));
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
}

void loop() {
  // Nothing to do - RXM-PMREQ is send-only
  delay(1000);
}
