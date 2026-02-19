/*!
 * @file ubx_nav_posllh.ino
 *
 * Hardware test: Poll UBX-NAV-POSLLH and display all fields
 * Tests the pollNAVPOSLLH() method and NAV-POSLLH struct parsing.
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

  Serial.println(F("UBX-NAV-POSLLH Poll Test"));
  Serial.println(F("========================"));

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
  UBX_NAV_POSLLH_t pos;

  if (ubx.pollNAVPOSLLH(&pos)) {
    Serial.println(F("--- NAV-POSLLH (Geodetic Position) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(pos.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("Longitude: "));
    Serial.print(pos.lon * 1e-7, 7);
    Serial.println(F(" deg"));

    Serial.print(F("Latitude: "));
    Serial.print(pos.lat * 1e-7, 7);
    Serial.println(F(" deg"));

    Serial.print(F("Height (ellipsoid): "));
    Serial.print(pos.height / 1000.0, 3);
    Serial.println(F(" m"));

    Serial.print(F("Height (MSL): "));
    Serial.print(pos.hMSL / 1000.0, 3);
    Serial.println(F(" m"));

    Serial.print(F("Horizontal accuracy: "));
    Serial.print(pos.hAcc / 1000.0, 3);
    Serial.println(F(" m"));

    Serial.print(F("Vertical accuracy: "));
    Serial.print(pos.vAcc / 1000.0, 3);
    Serial.println(F(" m"));

    Serial.println();
  } else {
    Serial.println(F("NAV-POSLLH poll failed (timeout)"));
  }

  delay(1000);
}
