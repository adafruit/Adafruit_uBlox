/*!
 * @file mon_hw2_hw.ino
 * @brief Hardware test for MON-HW2 (Extended Hardware Status)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* cfgSrc(uint8_t s) {
  switch (s) {
    case 114: return "ROM";
    case 111: return "OTP";
    case 112: return "Pins";
    case 102: return "Flash";
    default: return "???";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("MON-HW2 Hardware Test"));
  Serial.println(F("====================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_MON_HW2_t hw2;
  if (ubx.pollMonHw2(&hw2)) {
    Serial.println(F("PASS: MON-HW2 poll OK"));
    Serial.print(F("  ofsI="));
    Serial.print(hw2.ofsI);
    Serial.print(F(" magI="));
    Serial.println(hw2.magI);
    Serial.print(F("  ofsQ="));
    Serial.print(hw2.ofsQ);
    Serial.print(F(" magQ="));
    Serial.println(hw2.magQ);
    Serial.print(F("  Config: "));
    Serial.println(cfgSrc(hw2.cfgSource));
    Serial.print(F("  POST: 0x"));
    Serial.println(hw2.postStatus, HEX);
  } else {
    Serial.println(F("FAIL: MON-HW2 poll failed"));
  }
}

void loop() {
  delay(2000);
  UBX_MON_HW2_t hw2;
  if (ubx.pollMonHw2(&hw2)) {
    Serial.print(F("I("));
    Serial.print(hw2.ofsI);
    Serial.print(F(","));
    Serial.print(hw2.magI);
    Serial.print(F(") Q("));
    Serial.print(hw2.ofsQ);
    Serial.print(F(","));
    Serial.print(hw2.magQ);
    Serial.print(F(") cfg="));
    Serial.println(cfgSrc(hw2.cfgSource));
  }
}
