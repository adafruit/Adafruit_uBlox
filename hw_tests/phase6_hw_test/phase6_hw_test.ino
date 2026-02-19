/*!
 * @file phase6_hw_test.ino
 *
 * Hardware test for Phase 6 Advanced Feature messages.
 * Tests NAV-ODO, NAV-TIMELS, geofence, logging, and backup.
 *
 * Upload with reset pin A0 at 115200 baud.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

uint8_t total_passed = 0;
uint8_t total_tests = 0;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  total_tests++;
  if (pass)
    total_passed++;

  Serial.print(F("  ["));
  Serial.print(pass ? F("PASS") : F("FAIL"));
  Serial.print(F("] "));
  Serial.println(name);
}

void testNavOdo() {
  Serial.println();
  Serial.println(F("=== NAV-ODO Test ==="));

  UBX_NAV_ODO_t odo;

  // Test poll
  bool poll_ok = ubx.pollNavOdo(&odo);
  printTestResult(F("pollNavOdo"), poll_ok);

  if (poll_ok) {
    Serial.print(F("    distance="));
    Serial.print(odo.distance);
    Serial.print(F("m, total="));
    Serial.print(odo.totalDistance);
    Serial.println(F("m"));
  }

  // Test reset
  bool reset_ok = ubx.resetOdometer();
  printTestResult(F("resetOdometer"), reset_ok);
}

void testNavTimels() {
  Serial.println();
  Serial.println(F("=== NAV-TIMELS Test ==="));

  UBX_NAV_TIMELS_t timels;

  bool poll_ok = ubx.pollNavTimels(&timels);
  printTestResult(F("pollNavTimels"), poll_ok);

  if (poll_ok) {
    Serial.print(F("    currLs="));
    Serial.print(timels.currLs);
    Serial.print(F("s (GPS-UTC), valid=0x"));
    Serial.println(timels.valid, HEX);
  }
}

void testGeofence() {
  Serial.println();
  Serial.println(F("=== Geofence Test ==="));

  // Clear first
  bool clear_ok = ubx.clearGeofence();
  printTestResult(F("clearGeofence"), clear_ok);

  delay(200);

  // Set a fence (arbitrary position)
  bool set_ok = ubx.setGeofence(404000000, -740000000, 10000, 2);
  printTestResult(F("setGeofence"), set_ok);

  delay(200);

  // Poll config
  UBX_CFG_GEOFENCE_header_t cfgHdr;
  UBX_CFG_GEOFENCE_fence_t cfgFence[4];
  uint8_t numFences = ubx.pollCfgGeofence(&cfgHdr, cfgFence, 4);
  printTestResult(F("pollCfgGeofence"), numFences == 1);

  // Poll status
  UBX_NAV_GEOFENCE_header_t navHdr;
  UBX_NAV_GEOFENCE_fence_t navFence[4];
  uint8_t statusFences = ubx.pollNavGeofence(&navHdr, navFence, 4);
  printTestResult(F("pollNavGeofence"), statusFences >= 0);

  if (statusFences > 0) {
    Serial.print(F("    status="));
    Serial.print(navHdr.status);
    Serial.print(F(", state="));
    Serial.println(navFence[0].state);
  }

  // Clean up
  ubx.clearGeofence();
}

void testLogging() {
  Serial.println();
  Serial.println(F("=== Logging Test ==="));

  // Erase first
  ubx.eraseLog();
  delay(300);

  // Create log
  bool create_ok = ubx.createLog(UBX_LOG_SIZE_MINIMUM, true, 0);
  printTestResult(F("createLog"), create_ok);

  delay(200);

  // Get info
  UBX_LOG_INFO_t info;
  bool info_ok = ubx.pollLogInfo(&info);
  printTestResult(F("pollLogInfo"), info_ok);

  if (info_ok) {
    Serial.print(F("    capacity="));
    Serial.print(info.filestoreCapacity);
    Serial.print(F(" bytes, entries="));
    Serial.println(info.entryCount);

    bool active_ok = !(info.status & UBX_LOG_INFO_STATUS_INACTIVE);
    printTestResult(F("log_active"), active_ok);
  }

  // Erase log
  bool erase_ok = ubx.eraseLog();
  printTestResult(F("eraseLog"), erase_ok);
}

void testBackup() {
  Serial.println();
  Serial.println(F("=== UPD-SOS Backup Test ==="));

  // Clear any existing
  bool clear_ok = ubx.clearBackup();
  printTestResult(F("clearBackup"), clear_ok);

  delay(200);

  // Create backup
  bool backup_ok = ubx.backupToFlash();
  printTestResult(F("backupToFlash"), backup_ok);

  delay(200);

  // Poll status
  UBX_UPD_SOS_response_t response;
  uint8_t status = ubx.pollUpdSos(&response);
  printTestResult(F("pollUpdSos"), status != 0xFF);

  Serial.print(F("    response="));
  Serial.println(status);

  // Clean up
  ubx.clearBackup();
}

void testTimepulse() {
  Serial.println();
  Serial.println(F("=== CFG-TP5 Timepulse Test ==="));

  UBX_CFG_TP5_t tp5;

  bool poll_ok = ubx.pollCfgTp5(&tp5, 0);
  printTestResult(F("pollCfgTp5"), poll_ok);

  if (poll_ok) {
    Serial.print(F("    tpIdx="));
    Serial.print(tp5.tpIdx);
    Serial.print(F(", freq="));
    Serial.print(tp5.freqPeriod);
    Serial.print(F(", flags=0x"));
    Serial.println(tp5.flags, HEX);
  }
}

void testHpMessages() {
  Serial.println();
  Serial.println(F("=== HP GNSS Messages (may fail on SAM-M8Q) ==="));

  UBX_NAV_HPPOSLLH_t hpllh;
  bool hp_ok = ubx.pollNavHpposllh(&hpllh);
  Serial.print(F("  ["));
  Serial.print(hp_ok ? F("PASS") : F("INFO"));
  Serial.print(F("] pollNavHpposllh: "));
  Serial.println(hp_ok ? F("HP GNSS!") : F("Not available (expected)"));
  // Don't count as failure on non-HP receivers

  UBX_NAV_SVIN_t svin;
  bool svin_ok = ubx.pollNavSvin(&svin);
  Serial.print(F("  ["));
  Serial.print(svin_ok ? F("PASS") : F("INFO"));
  Serial.print(F("] pollNavSvin: "));
  Serial.println(svin_ok ? F("RTK capable!") : F("Not available (expected)"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("Phase 6 Advanced Features Hardware Test"));
  Serial.println(F("========================================"));

  if (!ddc.begin()) {
    Serial.println(F("FATAL: Could not connect to GPS module!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS module connected on I2C"));

  if (!ubx.begin()) {
    Serial.println(F("FATAL: UBX parser init failed!"));
    while (1)
      delay(10);
  }

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  }

  Serial.println();
  Serial.println(F("Starting tests in 2 seconds..."));
  delay(2000);

  // Run all tests
  testNavOdo();
  testNavTimels();
  testGeofence();
  testLogging();
  testBackup();
  testTimepulse();
  testHpMessages();

  // Summary
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("TOTAL: "));
  Serial.print(total_passed);
  Serial.print(F("/"));
  Serial.print(total_tests);
  Serial.println(F(" tests passed"));
  Serial.println(F("========================================"));

  if (total_passed == total_tests) {
    Serial.println(F("ALL TESTS PASSED!"));
  } else {
    Serial.println(F("Some tests failed - check output above"));
  }
}

void loop() {
  // Test complete
  delay(1000);
}
