/*!
 * @file cfg_nav5_test.ino
 *
 * Message test: Poll UBX-CFG-NAV5, validate fields,
 * set dynamic model to Pedestrian, verify, restore.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

bool tests_run = false;
uint8_t original_model = 0;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

const char* getDynModelName(uint8_t model) {
  switch (model) {
    case 0: return "Portable";
    case 2: return "Stationary";
    case 3: return "Pedestrian";
    case 4: return "Automotive";
    case 5: return "Sea";
    case 6: return "Airborne1g";
    case 7: return "Airborne2g";
    case 8: return "Airborne4g";
    default: return "Unknown";
  }
}

void printNav5(UBX_CFG_NAV5_t* nav5) {
  Serial.print(F("  dynModel: "));
  Serial.print(nav5->dynModel);
  Serial.print(F(" ("));
  Serial.print(getDynModelName(nav5->dynModel));
  Serial.println(F(")"));
  Serial.print(F("  fixMode: "));
  Serial.println(nav5->fixMode);
  Serial.print(F("  minElev: "));
  Serial.print(nav5->minElev);
  Serial.println(F(" deg"));
  Serial.print(F("  pDop: "));
  Serial.println(nav5->pDop * 0.1);
  Serial.print(F("  tDop: "));
  Serial.println(nav5->tDop * 0.1);
  Serial.print(F("  staticHoldThresh: "));
  Serial.print(nav5->staticHoldThresh);
  Serial.println(F(" cm/s"));
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 5;

  Serial.println();
  Serial.println(F("Running tests..."));

  // Test 1: Poll CFG-NAV5
  UBX_CFG_NAV5_t nav5;
  bool poll_ok = ubx.pollCfgNav5(&nav5);
  printTestResult(F("poll_cfg_nav5"), poll_ok);
  Serial.println(poll_ok ? F("OK") : F("FAILED"));
  if (poll_ok) {
    passed++;
    original_model = nav5.dynModel;
    printNav5(&nav5);
  }

  // Test 2: Get dynamic model via convenience method
  uint8_t model = ubx.getDynamicModel();
  bool get_model_ok = (model != 0xFF) && (model == original_model);
  printTestResult(F("get_dynamic_model"), get_model_ok);
  Serial.print(model);
  Serial.print(F(" ("));
  Serial.print(getDynModelName(model));
  Serial.println(F(")"));
  if (get_model_ok) passed++;

  // Test 3: Set dynamic model to Pedestrian
  bool set_ped_ok = ubx.setDynamicModel(UBX_DYNMODEL_PEDESTRIAN);
  printTestResult(F("set_pedestrian"), set_ped_ok);
  Serial.println(set_ped_ok ? F("OK") : F("FAILED"));
  if (set_ped_ok) passed++;

  delay(100);

  // Test 4: Verify change took effect
  uint8_t new_model = ubx.getDynamicModel();
  bool verify_ok = (new_model == UBX_DYNMODEL_PEDESTRIAN);
  printTestResult(F("verify_pedestrian"), verify_ok);
  Serial.print(new_model);
  Serial.print(F(" (expected 3)"));
  Serial.println();
  if (verify_ok) passed++;

  // Test 5: Restore original model
  bool restore_ok = ubx.setDynamicModel(original_model);
  printTestResult(F("restore_original"), restore_ok);
  Serial.println(restore_ok ? F("OK") : F("FAILED"));
  if (restore_ok) passed++;

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

  Serial.println(F("UBX-CFG-NAV5 Message Test"));
  Serial.println(F("=========================="));

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

  ubx.verbose_debug = 1; // Enable debug output

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  runTests();
  tests_run = true;
}

void loop() {
  // Nothing to do in loop
  delay(1000);
}
