/*!
 * @file cfg_cfg_test.ino
 *
 * Message test: Use UBX-CFG-CFG to save/load configuration changes.
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
  Serial.print(name);
  Serial.print(F(": "));
}

bool initModule() {
  if (!ddc.begin()) {
    return false;
  }
  if (!ubx.begin()) {
    return false;
  }
  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  }
  return true;
}

bool reconnectModule(uint32_t timeout_ms) {
  uint32_t start_ms = millis();
  while (millis() - start_ms < timeout_ms) {
    if (initModule()) {
      return true;
    }
    delay(200);
  }
  return false;
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Setting rate to 5 Hz and saving config..."));
  bool set_rate = ubx.setRate(200);
  bool save_ok = set_rate && ubx.saveConfig();
  printTestResult(F("save_ok"), save_ok);
  Serial.println(save_ok ? F("OK") : F("BAD"));
  if (save_ok)
    passed++;

  Serial.println(F("Issuing hot start..."));
  ubx.hotStart();
  delay(3000);

  bool module_reconnected = reconnectModule(10000);
  if (!module_reconnected) {
    Serial.println(F("FAIL: Could not reconnect after reset"));
  }

  UBX_CFG_RATE_t rate;
  bool rate_persisted = false;
  if (module_reconnected && ubx.getRate(&rate)) {
    rate_persisted = rate.measRate == 200;
  }
  printTestResult(F("rate_persisted"), rate_persisted);
  Serial.print(rate.measRate);
  Serial.println(F(" ms"));
  if (rate_persisted)
    passed++;

  bool load_defaults_ok = ubx.loadDefaults();
  printTestResult(F("load_defaults_ok"), load_defaults_ok);
  Serial.println(load_defaults_ok ? F("OK") : F("BAD"));
  if (load_defaults_ok)
    passed++;

  bool rate_restored = false;
  if (load_defaults_ok && ubx.getRate(&rate)) {
    rate_restored = rate.measRate == 1000;
  }
  printTestResult(F("rate_restored"), rate_restored);
  Serial.print(rate.measRate);
  Serial.println(F(" ms"));
  if (rate_restored)
    passed++;

  Serial.println(F("Saving defaults back to NVM..."));
  ubx.saveConfig();

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

  Serial.println(F("UBX-CFG-CFG Message Test"));
  Serial.println(F("========================"));

  if (!initModule()) {
    Serial.println(F("FAIL: Could not connect to GPS module!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS module connected on I2C"));

  runTests();
}

void loop() {
  delay(1000);
}
