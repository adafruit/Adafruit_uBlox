/*!
 * @file mon_msgpp_hw.ino
 * @brief Hardware test for MON-MSGPP (Message Parse/Process Status)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("MON-MSGPP Hardware Test"));
  Serial.println(F("======================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_MON_MSGPP_t msgpp;
  if (ubx.pollMonMsgpp(&msgpp)) {
    Serial.println(F("PASS: MON-MSGPP poll OK"));
    Serial.println(F("  DDC counts: UBX NMEA RTCM2 -- -- RTCM3 -- --"));
    Serial.print(F("              "));
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(msgpp.msg[0][i]);
      Serial.print(F("    "));
    }
    Serial.println();
    Serial.print(F("  DDC skipped: "));
    Serial.println(msgpp.skipped[0]);
  } else {
    Serial.println(F("FAIL: MON-MSGPP poll failed"));
  }
}

void loop() {
  delay(5000);
  UBX_MON_MSGPP_t msgpp;
  if (ubx.pollMonMsgpp(&msgpp)) {
    Serial.print(F("DDC: UBX="));
    Serial.print(msgpp.msg[0][0]);
    Serial.print(F(" NMEA="));
    Serial.print(msgpp.msg[0][1]);
    Serial.print(F(" skip="));
    Serial.println(msgpp.skipped[0]);
  }
}
