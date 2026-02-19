/*!
 * @file upd_sos_test.ino
 *
 * Message test: Test UPD-SOS backup/restore functionality.
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

const char* getResponseStr(uint8_t resp) {
  switch (resp) {
    case UBX_UPD_SOS_RESP_UNKNOWN:
      return "Unknown";
    case UBX_UPD_SOS_RESP_FAILED:
      return "Failed";
    case UBX_UPD_SOS_RESP_RESTORED:
      return "Restored";
    case UBX_UPD_SOS_RESP_NONE:
      return "No backup";
    case 0xFF:
      return "Poll error";
    default:
      return "Invalid";
  }
}

uint8_t runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;
  UBX_UPD_SOS_response_t response;

  Serial.println();
  Serial.println(F("Running UPD-SOS tests..."));

  // Test 1: Poll current backup status
  uint8_t status = ubx.pollUpdSos(&response);
  bool poll_ok = (status != 0xFF);
  printTestResult(F("poll_status"), poll_ok);
  Serial.print(status);
  Serial.print(F(" ("));
  Serial.print(getResponseStr(status));
  Serial.println(F(")"));
  if (poll_ok)
    passed++;

  // Test 2: Clear any existing backup
  bool clear_ok = ubx.clearBackup();
  printTestResult(F("clear_backup"), clear_ok);
  Serial.println(clear_ok ? F("OK") : F("FAIL"));
  if (clear_ok)
    passed++;

  delay(500);

  // Test 3: Create a backup
  bool backup_ok = ubx.backupToFlash();
  printTestResult(F("create_backup"), backup_ok);
  Serial.println(backup_ok ? F("OK") : F("FAIL"));
  if (backup_ok)
    passed++;

  delay(500);

  // Test 4: Poll status again - should show backup exists
  status = ubx.pollUpdSos(&response);
  // After creating backup, we expect either "restored" (if just powered on)
  // or "none" (if no power cycle). The key is the poll works.
  bool status_ok = (status != 0xFF);
  printTestResult(F("status_after_backup"), status_ok);
  Serial.print(status);
  Serial.print(F(" ("));
  Serial.print(getResponseStr(status));
  Serial.println(F(")"));
  if (status_ok)
    passed++;

  // Clean up: clear backup
  Serial.println();
  Serial.println(F("Cleaning up: clearing backup..."));
  ubx.clearBackup();

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  Serial.println();
  Serial.println(F("Note: To fully test backup/restore, you need to:"));
  Serial.println(F("  1. Create backup (backupToFlash)"));
  Serial.println(F("  2. Power cycle the module"));
  Serial.println(F("  3. Poll status - should show 'Restored'"));
  Serial.println(F("  4. Clear backup to prevent stale data on next boot"));

  return passed;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-UPD-SOS Message Test ==="));

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
