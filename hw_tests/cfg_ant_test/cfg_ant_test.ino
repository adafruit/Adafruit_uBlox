/*!
 * @file cfg_ant_test.ino (hw_test)
 *
 * Hardware test for UBX-CFG-ANT message.
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

  Serial.println(F("=== HW TEST: CFG-ANT ==="));

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
  const uint8_t total = 2;

  // Test 1: Poll ANT config
  UBX_CFG_ANT_t ant;
  bool test1 = ubx.pollCfgAnt(&ant);
  Serial.print(F("["));
  Serial.print(test1 ? F("PASS") : F("FAIL"));
  Serial.print(F("] pollCfgAnt: flags=0x"));
  Serial.print(ant.flags, HEX);
  Serial.print(F(", pins=0x"));
  Serial.println(ant.pins, HEX);
  if (test1)
    passed++;

  UBX_CFG_ANT_t original;
  memcpy(&original, &ant, sizeof(ant));

  // Test 2: Toggle SCD flag and restore
  ant.flags ^= UBX_ANT_FLAG_SCD;
  bool set_ok = ubx.setCfgAnt(&ant);
  delay(100);
  UBX_CFG_ANT_t verify;
  ubx.pollCfgAnt(&verify);
  bool changed = (verify.flags != original.flags);
  // Restore
  ubx.setCfgAnt(&original);
  bool test2 = set_ok && changed;
  Serial.print(F("["));
  Serial.print(test2 ? F("PASS") : F("FAIL"));
  Serial.println(F("] toggle SCD flag"));
  if (test2)
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
