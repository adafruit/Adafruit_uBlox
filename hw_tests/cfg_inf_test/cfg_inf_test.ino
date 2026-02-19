/*!
 * @file cfg_inf_test.ino (hw_test)
 *
 * Hardware test for UBX-CFG-INF message.
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

  Serial.println(F("=== HW TEST: CFG-INF ==="));

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

  // Test 1: Poll UBX INF config
  UBX_CFG_INF_block_t blocks[2];
  uint8_t numBlocks = ubx.pollCfgInf(UBX_INF_PROTOCOL_UBX, blocks, 2);
  bool test1 = (numBlocks > 0);
  Serial.print(F("["));
  Serial.print(test1 ? F("PASS") : F("FAIL"));
  Serial.print(F("] pollCfgInf(UBX): "));
  Serial.print(numBlocks);
  Serial.println(F(" block(s)"));
  if (test1) {
    passed++;
    Serial.print(F("  DDC mask: 0x"));
    Serial.println(blocks[0].infMsgMask[0], HEX);
  }

  // Test 2: Poll NMEA INF config
  numBlocks = ubx.pollCfgInf(UBX_INF_PROTOCOL_NMEA, blocks, 2);
  bool test2 = (numBlocks > 0);
  Serial.print(F("["));
  Serial.print(test2 ? F("PASS") : F("FAIL"));
  Serial.print(F("] pollCfgInf(NMEA): "));
  Serial.print(numBlocks);
  Serial.println(F(" block(s)"));
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
