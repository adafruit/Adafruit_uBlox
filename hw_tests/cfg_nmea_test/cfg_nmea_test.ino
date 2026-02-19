/*!
 * @file cfg_nmea_test.ino (hw_test)
 *
 * Hardware test for UBX-CFG-NMEA message.
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

  Serial.println(F("=== HW TEST: CFG-NMEA ==="));

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

  // Test 1: Poll NMEA config
  UBX_CFG_NMEA_t nmea;
  bool test1 = ubx.pollCfgNmea(&nmea);
  Serial.print(F("["));
  Serial.print(test1 ? F("PASS") : F("FAIL"));
  Serial.print(F("] pollCfgNmea: ver=0x"));
  Serial.println(nmea.nmeaVersion, HEX);
  if (test1)
    passed++;

  UBX_CFG_NMEA_t original;
  memcpy(&original, &nmea, sizeof(nmea));

  // Test 2: Set numSV and restore
  nmea.numSV = 8;
  bool set_ok = ubx.setCfgNmea(&nmea);
  delay(100);
  UBX_CFG_NMEA_t verify;
  ubx.pollCfgNmea(&verify);
  bool test2 = set_ok && (verify.numSV == 8);
  // Restore
  ubx.setCfgNmea(&original);
  Serial.print(F("["));
  Serial.print(test2 ? F("PASS") : F("FAIL"));
  Serial.print(F("] set numSV=8: "));
  Serial.println(verify.numSV);
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
