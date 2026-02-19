/*!
 * @file ubx_nav_velned.ino
 *
 * Hardware test: Poll UBX-NAV-VELNED and display all fields
 * Tests the pollNAVVELNED() method and NAV-VELNED struct parsing.
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

  Serial.println(F("UBX-NAV-VELNED Poll Test"));
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
  UBX_NAV_VELNED_t vel;

  if (ubx.pollNAVVELNED(&vel)) {
    Serial.println(F("--- NAV-VELNED (NED Velocity) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(vel.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("North velocity: "));
    Serial.print(vel.velN / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("East velocity: "));
    Serial.print(vel.velE / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("Down velocity: "));
    Serial.print(vel.velD / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("3D Speed: "));
    Serial.print(vel.speed / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("Ground Speed: "));
    Serial.print(vel.gSpeed / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("Heading: "));
    Serial.print(vel.heading * 1e-5, 2);
    Serial.println(F(" deg"));

    Serial.print(F("Speed accuracy: "));
    Serial.print(vel.sAcc / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("Course accuracy: "));
    Serial.print(vel.cAcc * 1e-5, 2);
    Serial.println(F(" deg"));

    Serial.println();
  } else {
    Serial.println(F("NAV-VELNED poll failed (timeout)"));
  }

  delay(1000);
}
