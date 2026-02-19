/*!
 * @file sec_uniqid_hw.ino
 * @brief Hardware test for SEC-UNIQID (Unique Chip ID)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void printHex(uint8_t v) {
  if (v < 0x10) Serial.print(F("0"));
  Serial.print(v, HEX);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("SEC-UNIQID Hardware Test"));
  Serial.println(F("========================"));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_SEC_UNIQID_t uniqid;
  if (ubx.pollSecUniqid(&uniqid)) {
    Serial.println(F("PASS: SEC-UNIQID poll OK"));
    Serial.print(F("  Version: "));
    Serial.println(uniqid.version);
    Serial.print(F("  Unique ID: "));
    for (uint8_t i = 0; i < 5; i++) {
      printHex(uniqid.uniqueId[i]);
      if (i < 4) Serial.print(F(":"));
    }
    Serial.println();
  } else {
    Serial.println(F("FAIL: SEC-UNIQID poll failed"));
  }
}

void loop() {
  delay(10000);
  UBX_SEC_UNIQID_t uniqid;
  if (ubx.pollSecUniqid(&uniqid)) {
    Serial.print(F("ID: "));
    for (uint8_t i = 0; i < 5; i++) {
      printHex(uniqid.uniqueId[i]);
      if (i < 4) Serial.print(F(":"));
    }
    Serial.println();
  }
}
