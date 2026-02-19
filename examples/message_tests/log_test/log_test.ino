/*!
 * @file log_test.ino
 *
 * Message test: Create, info, and erase log using LOG messages.
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
  Serial.print(pass ? F("PASS") : F("FAIL"));
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

void printLogInfo(const UBX_LOG_INFO_t& info) {
  Serial.println(F("--- LOG-INFO ---"));

  Serial.print(F("filestoreCapacity: "));
  Serial.print(info.filestoreCapacity);
  Serial.println(F(" bytes"));

  Serial.print(F("currentMaxLogSize: "));
  Serial.print(info.currentMaxLogSize);
  Serial.println(F(" bytes"));

  Serial.print(F("currentLogSize: "));
  Serial.print(info.currentLogSize);
  Serial.println(F(" bytes"));

  Serial.print(F("entryCount: "));
  Serial.println(info.entryCount);

  if (info.oldestYear > 0) {
    Serial.print(F("oldest: "));
    Serial.print(info.oldestYear);
    Serial.print(F("-"));
    if (info.oldestMonth < 10)
      Serial.print(F("0"));
    Serial.print(info.oldestMonth);
    Serial.print(F("-"));
    if (info.oldestDay < 10)
      Serial.print(F("0"));
    Serial.print(info.oldestDay);
    Serial.print(F(" "));
    if (info.oldestHour < 10)
      Serial.print(F("0"));
    Serial.print(info.oldestHour);
    Serial.print(F(":"));
    if (info.oldestMinute < 10)
      Serial.print(F("0"));
    Serial.print(info.oldestMinute);
    Serial.print(F(":"));
    if (info.oldestSecond < 10)
      Serial.print(F("0"));
    Serial.println(info.oldestSecond);
  }

  if (info.newestYear > 0) {
    Serial.print(F("newest: "));
    Serial.print(info.newestYear);
    Serial.print(F("-"));
    if (info.newestMonth < 10)
      Serial.print(F("0"));
    Serial.print(info.newestMonth);
    Serial.print(F("-"));
    if (info.newestDay < 10)
      Serial.print(F("0"));
    Serial.print(info.newestDay);
    Serial.print(F(" "));
    if (info.newestHour < 10)
      Serial.print(F("0"));
    Serial.print(info.newestHour);
    Serial.print(F(":"));
    if (info.newestMinute < 10)
      Serial.print(F("0"));
    Serial.print(info.newestMinute);
    Serial.print(F(":"));
    if (info.newestSecond < 10)
      Serial.print(F("0"));
    Serial.println(info.newestSecond);
  }

  Serial.print(F("status: 0x"));
  Serial.print(info.status, HEX);
  Serial.print(F(" (recording="));
  Serial.print((info.status & UBX_LOG_INFO_STATUS_RECORDING) ? F("Y") : F("N"));
  Serial.print(F(", inactive="));
  Serial.print((info.status & UBX_LOG_INFO_STATUS_INACTIVE) ? F("Y") : F("N"));
  Serial.print(F(", circular="));
  Serial.print((info.status & UBX_LOG_INFO_STATUS_CIRCULAR) ? F("Y") : F("N"));
  Serial.println(F(")"));

  Serial.println();
}

uint8_t runTests() {
  uint8_t passed = 0;
  const uint8_t total = 6;
  UBX_LOG_INFO_t info;

  Serial.println();
  Serial.println(F("Running LOG tests..."));

  // Test 1: First erase any existing log
  Serial.println(F("Erasing any existing log..."));
  ubx.eraseLog();
  delay(500);

  // Test 2: Create a new log (minimum size, circular)
  bool create_ok = ubx.createLog(UBX_LOG_SIZE_MINIMUM, true, 0);
  printTestResult(F("create_log"), create_ok);
  Serial.println(create_ok ? F("OK") : F("FAIL"));
  if (create_ok)
    passed++;

  delay(500);

  // Test 3: Poll log info
  bool poll_ok = ubx.pollLogInfo(&info);
  printTestResult(F("poll_info"), poll_ok);
  Serial.println(poll_ok ? F("OK") : F("FAIL"));
  if (poll_ok)
    passed++;

  if (poll_ok) {
    printLogInfo(info);
  }

  // Test 4: Log should not be inactive (log exists)
  bool active_ok = poll_ok && !(info.status & UBX_LOG_INFO_STATUS_INACTIVE);
  printTestResult(F("log_active"), active_ok);
  Serial.println(active_ok ? F("OK") : F("FAIL"));
  if (active_ok)
    passed++;

  // Test 5: Log should be circular
  bool circular_ok = poll_ok && (info.status & UBX_LOG_INFO_STATUS_CIRCULAR);
  printTestResult(F("log_circular"), circular_ok);
  Serial.println(circular_ok ? F("OK") : F("FAIL"));
  if (circular_ok)
    passed++;

  // Test 6: Capacity should be > 0
  bool capacity_ok = poll_ok && info.filestoreCapacity > 0;
  printTestResult(F("capacity_nonzero"), capacity_ok);
  Serial.print(info.filestoreCapacity);
  Serial.println(F(" bytes"));
  if (capacity_ok)
    passed++;

  // Test 7: Erase the log
  bool erase_ok = ubx.eraseLog();
  printTestResult(F("erase_log"), erase_ok);
  Serial.println(erase_ok ? F("OK") : F("FAIL"));
  if (erase_ok)
    passed++;

  delay(500);

  // Verify log is erased
  if (ubx.pollLogInfo(&info)) {
    Serial.println(F("After erase:"));
    printLogInfo(info);
  }

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-LOG Message Test ==="));

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
  Serial.println(F("Waiting a few seconds for module to stabilize..."));
  delay(3000);

  runTests();
  tests_run = true;
}

void loop() {
  // Nothing to do - tests are complete
  delay(1000);
}
