/*!
 * @file sec_uniqid_test.ino
 *
 * Message test: Poll UBX-SEC-UNIQID (Unique Chip ID)
 * Displays the 40-bit unique chip identifier.
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

void printHex(uint8_t val) {
  if (val < 0x10) Serial.print(F("0"));
  Serial.print(val, HEX);
}

void printUniqidDetails(UBX_SEC_UNIQID_t* uniqid) {
  Serial.println(F("\nSEC-UNIQID Details:"));

  Serial.print(F("  Version: "));
  Serial.println(uniqid->version);

  Serial.print(F("  Unique ID: "));
  for (uint8_t i = 0; i < 5; i++) {
    printHex(uniqid->uniqueId[i]);
    if (i < 4) Serial.print(F(":"));
  }
  Serial.println();

  // Also print as decimal for reference
  Serial.print(F("  As bytes: "));
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(uniqid->uniqueId[i]);
    if (i < 4) Serial.print(F("."));
  }
  Serial.println();
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running SEC-UNIQID tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_SEC_UNIQID_t) == 9);
  printTestResult(F("struct_size_9"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll SEC-UNIQID
  UBX_SEC_UNIQID_t uniqid;
  bool poll_ok = ubx.pollSecUniqid(&uniqid);
  printTestResult(F("poll_sec_uniqid"), poll_ok);
  if (poll_ok) {
    passed++;
    printUniqidDetails(&uniqid);
  }

  // Test 3: Verify version is 0x01
  bool version_ok = poll_ok && (uniqid.version == 0x01);
  printTestResult(F("version_0x01"), version_ok);
  if (version_ok) passed++;

  // Test 4: Verify unique ID is not all zeros
  bool nonzero = false;
  if (poll_ok) {
    for (uint8_t i = 0; i < 5; i++) {
      if (uniqid.uniqueId[i] != 0) {
        nonzero = true;
        break;
      }
    }
  }
  printTestResult(F("id_nonzero"), nonzero);
  if (nonzero) passed++;

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

  Serial.println(F("UBX-SEC-UNIQID Message Test"));
  Serial.println(F("============================"));

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
  Serial.println(F("Test complete. Unique ID is fixed per chip."));
}

void loop() {
  // Unique ID doesn't change, so just poll once more after a delay
  delay(10000);

  UBX_SEC_UNIQID_t uniqid;
  if (ubx.pollSecUniqid(&uniqid)) {
    Serial.print(F("SEC-UNIQID: "));
    for (uint8_t i = 0; i < 5; i++) {
      printHex(uniqid.uniqueId[i]);
      if (i < 4) Serial.print(F(":"));
    }
    Serial.println();
  } else {
    Serial.println(F("SEC-UNIQID poll timeout"));
  }
}
