/*!
 * @file cfg_sbas_test.ino (hw_test)
 *
 * Hardware test for UBX-CFG-SBAS message.
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

  Serial.println(F("=== HW TEST: CFG-SBAS ==="));

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

  // Test 1: Poll SBAS config
  UBX_CFG_SBAS_t sbas;
  bool test1 = ubx.pollCfgSbas(&sbas);
  Serial.print(F("["));
  Serial.print(test1 ? F("PASS") : F("FAIL"));
  Serial.print(F("] pollCfgSbas: mode=0x"));
  Serial.println(sbas.mode, HEX);
  if (test1)
    passed++;

  bool originalEnabled = (sbas.mode & UBX_SBAS_MODE_ENABLED) != 0;

  // Test 2: Toggle SBAS
  bool test2 = ubx.enableSBAS(!originalEnabled);
  delay(100);
  UBX_CFG_SBAS_t verify;
  ubx.pollCfgSbas(&verify);
  bool toggled = ((verify.mode & UBX_SBAS_MODE_ENABLED) != 0) != originalEnabled;
  test2 = test2 && toggled;
  Serial.print(F("["));
  Serial.print(test2 ? F("PASS") : F("FAIL"));
  Serial.println(F("] toggle SBAS"));
  if (test2)
    passed++;

  // Test 3: Restore
  bool test3 = ubx.enableSBAS(originalEnabled);
  delay(100);
  ubx.pollCfgSbas(&verify);
  bool restored = ((verify.mode & UBX_SBAS_MODE_ENABLED) != 0) == originalEnabled;
  test3 = test3 && restored;
  Serial.print(F("["));
  Serial.print(test3 ? F("PASS") : F("FAIL"));
  Serial.println(F("] restore SBAS"));
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
