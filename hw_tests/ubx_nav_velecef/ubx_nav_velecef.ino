/*!
 * @file ubx_nav_velecef.ino
 *
 * Hardware test: Poll UBX-NAV-VELECEF and display all fields
 * Tests the pollNAVVELECEF() method and NAV-VELECEF struct parsing.
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

  Serial.println(F("UBX-NAV-VELECEF Poll Test"));
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
  UBX_NAV_VELECEF_t vel;

  if (ubx.pollNAVVELECEF(&vel)) {
    Serial.println(F("--- NAV-VELECEF (ECEF Velocity) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(vel.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("ECEF VX: "));
    Serial.print(vel.ecefVX / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("ECEF VY: "));
    Serial.print(vel.ecefVY / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("ECEF VZ: "));
    Serial.print(vel.ecefVZ / 100.0, 2);
    Serial.println(F(" m/s"));

    Serial.print(F("Speed accuracy: "));
    Serial.print(vel.sAcc / 100.0, 2);
    Serial.println(F(" m/s"));

    // Calculate and display 3D speed
    double vx = vel.ecefVX / 100.0;
    double vy = vel.ecefVY / 100.0;
    double vz = vel.ecefVZ / 100.0;
    double speed3d = sqrt(vx * vx + vy * vy + vz * vz);
    Serial.print(F("3D Speed: "));
    Serial.print(speed3d, 2);
    Serial.println(F(" m/s"));

    Serial.println();
  } else {
    Serial.println(F("NAV-VELECEF poll failed (timeout)"));
  }

  delay(1000);
}
