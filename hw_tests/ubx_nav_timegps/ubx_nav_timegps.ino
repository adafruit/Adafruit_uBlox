/*!
 * @file ubx_nav_timegps.ino
 *
 * Hardware test: Poll UBX-NAV-TIMEGPS and display all fields
 * Tests the pollNAVTIMEGPS() method and NAV-TIMEGPS struct parsing.
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

  Serial.println(F("UBX-NAV-TIMEGPS Poll Test"));
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
  UBX_NAV_TIMEGPS_t gps;

  if (ubx.pollNAVTIMEGPS(&gps)) {
    Serial.println(F("--- NAV-TIMEGPS (GPS Time) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(gps.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("fTOW: "));
    Serial.print(gps.fTOW);
    Serial.println(F(" ns"));

    // Calculate precise GPS time
    double precise_tow = (gps.iTOW * 1e-3) + (gps.fTOW * 1e-9);
    Serial.print(F("Precise TOW: "));
    Serial.print(precise_tow, 9);
    Serial.println(F(" s"));

    Serial.print(F("GPS week: "));
    Serial.println(gps.week);

    Serial.print(F("Leap seconds: "));
    Serial.print(gps.leapS);
    Serial.println(F(" s"));

    Serial.print(F("Time accuracy: "));
    Serial.print(gps.tAcc);
    Serial.println(F(" ns"));

    Serial.print(F("Valid flags: TOW="));
    Serial.print(gps.valid & 0x01);
    Serial.print(F(" Week="));
    Serial.print((gps.valid >> 1) & 0x01);
    Serial.print(F(" LeapS="));
    Serial.println((gps.valid >> 2) & 0x01);

    Serial.println();
  } else {
    Serial.println(F("NAV-TIMEGPS poll failed (timeout)"));
  }

  delay(1000);
}
