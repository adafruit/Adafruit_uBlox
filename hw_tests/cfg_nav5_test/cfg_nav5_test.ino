/*!
 * @file cfg_nav5_test.ino (hw_test)
 *
 * Hardware test for UBX-CFG-NAV5 message.
 * Tests dynamic model get/set on real hardware.
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

const char* getDynModelName(uint8_t model) {
  switch (model) {
    case UBX_DYNMODEL_PORTABLE:
      return "Portable";
    case UBX_DYNMODEL_STATIONARY:
      return "Stationary";
    case UBX_DYNMODEL_PEDESTRIAN:
      return "Pedestrian";
    case UBX_DYNMODEL_AUTOMOTIVE:
      return "Automotive";
    case UBX_DYNMODEL_SEA:
      return "Sea";
    case UBX_DYNMODEL_AIRBORNE1G:
      return "Airborne<1g";
    case UBX_DYNMODEL_AIRBORNE2G:
      return "Airborne<2g";
    case UBX_DYNMODEL_AIRBORNE4G:
      return "Airborne<4g";
    default:
      return "Unknown";
  }
}

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

  Serial.println(F("=== HW TEST: CFG-NAV5 ==="));

  resetGPS();

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS connected"));

  ubx.begin();
  ubx.verbose_debug = 1; // Enable debug
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  // Run tests
  uint8_t passed = 0;
  const uint8_t total = 3;

  // Test 1: Get current model
  uint8_t model = ubx.getDynamicModel();
  bool test1 = (model != 0xFF);
  Serial.print(F("["));
  Serial.print(test1 ? F("PASS") : F("FAIL"));
  Serial.print(F("] getDynamicModel: "));
  Serial.print(model);
  Serial.print(F(" ("));
  Serial.print(getDynModelName(model));
  Serial.println(F(")"));
  if (test1)
    passed++;

  uint8_t original = model;

  // Test 2: Set to Pedestrian
  bool test2 = ubx.setDynamicModel(UBX_DYNMODEL_PEDESTRIAN);
  delay(100);
  model = ubx.getDynamicModel();
  test2 = test2 && (model == UBX_DYNMODEL_PEDESTRIAN);
  Serial.print(F("["));
  Serial.print(test2 ? F("PASS") : F("FAIL"));
  Serial.print(F("] setDynamicModel(Pedestrian): "));
  Serial.println(getDynModelName(model));
  if (test2)
    passed++;

  // Test 3: Restore
  bool test3 = ubx.setDynamicModel(original);
  delay(100);
  model = ubx.getDynamicModel();
  test3 = test3 && (model == original);
  Serial.print(F("["));
  Serial.print(test3 ? F("PASS") : F("FAIL"));
  Serial.print(F("] restore: "));
  Serial.println(getDynModelName(model));
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
