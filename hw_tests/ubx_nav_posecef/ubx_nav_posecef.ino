/*!
 * @file ubx_nav_posecef.ino
 *
 * Hardware test: Poll UBX-NAV-POSECEF and display all fields
 * Tests the pollNAVPOSECEF() method and NAV-POSECEF struct parsing.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-NAV-POSECEF Poll Test"));
  Serial.println(F("========================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: Could not connect to GPS module!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS module connected on I2C"));

  if (!ubx.begin()) {
    Serial.println(F("FAIL: UBX parser init failed!"));
    while (1)
      delay(10);
  }

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  Serial.println();
}

void loop() {
  UBX_NAV_POSECEF_t pos;

  if (ubx.pollNAVPOSECEF(&pos)) {
    Serial.println(F("--- NAV-POSECEF (ECEF Position) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(pos.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("ECEF X: "));
    Serial.print(pos.ecefX / 100.0, 2);
    Serial.println(F(" m"));

    Serial.print(F("ECEF Y: "));
    Serial.print(pos.ecefY / 100.0, 2);
    Serial.println(F(" m"));

    Serial.print(F("ECEF Z: "));
    Serial.print(pos.ecefZ / 100.0, 2);
    Serial.println(F(" m"));

    Serial.print(F("Position accuracy: "));
    Serial.print(pos.pAcc / 100.0, 2);
    Serial.println(F(" m"));

    // Calculate and display radius from Earth center
    double x = pos.ecefX / 100.0;
    double y = pos.ecefY / 100.0;
    double z = pos.ecefZ / 100.0;
    double radius = sqrt(x * x + y * y + z * z);
    Serial.print(F("Radius from center: "));
    Serial.print(radius / 1000.0, 3);
    Serial.println(F(" km"));

    Serial.println();
  } else {
    Serial.println(F("NAV-POSECEF poll failed (timeout)"));
  }

  delay(1000);
}
