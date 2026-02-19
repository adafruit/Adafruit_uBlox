/*!
 * @file cfg_rxm_test.ino (hw_test)
 *
 * Hardware test for UBX-CFG-RXM message.
 *
 * Target: QT Py RP2040 + SAM-M8Q (I2C 0x42)
 * Reset pin: A0, Baud: 115200
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

#define RESET_PIN A0

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void resetGPS() {
  Serial.println(F("Resetting GPS module..."));
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(RESET_PIN, INPUT);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== HW TEST: CFG-RXM ==="));

  resetGPS();

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS connected"));

  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  uint8_t passed = 0;
  const uint8_t total = 3;

  // Test 1: Poll RXM config
  UBX_CFG_RXM_t rxm;
  bool test1 = ubx.pollCfgRxm(&rxm);
  Serial.print(F("["));
  Serial.print(test1 ? F("PASS") : F("FAIL"));
  Serial.print(F("] pollCfgRxm: lpMode="));
  Serial.println(rxm.lpMode);
  if (test1)
    passed++;

  uint8_t originalMode = rxm.lpMode;

  // Test 2: Set power save mode
  bool test2 = ubx.setPowerSave(true);
  delay(100);
  UBX_CFG_RXM_t verify;
  ubx.pollCfgRxm(&verify);
  test2 = test2 && (verify.lpMode == UBX_RXM_LPMODE_POWERSAVE);
  Serial.print(F("["));
  Serial.print(test2 ? F("PASS") : F("FAIL"));
  Serial.print(F("] setPowerSave(true): lpMode="));
  Serial.println(verify.lpMode);
  if (test2)
    passed++;

  // Test 3: Restore continuous mode
  bool test3 = ubx.setPowerSave(originalMode == UBX_RXM_LPMODE_POWERSAVE);
  delay(100);
  ubx.pollCfgRxm(&verify);
  test3 = test3 && (verify.lpMode == originalMode);
  Serial.print(F("["));
  Serial.print(test3 ? F("PASS") : F("FAIL"));
  Serial.print(F("] restore: lpMode="));
  Serial.println(verify.lpMode);
  if (test3)
    passed++;

  Serial.println();
  Serial.print(F("RESULT: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(passed == total ? F(" PASS") : F(" FAIL"));
}

void loop() {
  delay(1000);
}
