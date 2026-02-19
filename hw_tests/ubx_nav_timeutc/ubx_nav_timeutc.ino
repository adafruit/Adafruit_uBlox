/*!
 * @file ubx_nav_timeutc.ino
 *
 * Hardware test: Poll UBX-NAV-TIMEUTC and display all fields
 * Tests the pollNAVTIMEUTC() method and NAV-TIMEUTC struct parsing.
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

  Serial.println(F("UBX-NAV-TIMEUTC Poll Test"));
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
  UBX_NAV_TIMEUTC_t utc;

  if (ubx.pollNAVTIMEUTC(&utc)) {
    Serial.println(F("--- NAV-TIMEUTC (UTC Time) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(utc.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("Date: "));
    char datebuf[16];
    snprintf(datebuf, sizeof(datebuf), "%04u-%02u-%02u", utc.year, utc.month,
             utc.day);
    Serial.println(datebuf);

    Serial.print(F("Time: "));
    char timebuf[16];
    snprintf(timebuf, sizeof(timebuf), "%02u:%02u:%02u", utc.hour, utc.min,
             utc.sec);
    Serial.println(timebuf);

    Serial.print(F("Nanoseconds: "));
    Serial.println(utc.nano);

    Serial.print(F("Time accuracy: "));
    Serial.print(utc.tAcc);
    Serial.println(F(" ns"));

    Serial.print(F("Valid flags: TOW="));
    Serial.print(utc.valid & 0x01);
    Serial.print(F(" WKN="));
    Serial.print((utc.valid >> 1) & 0x01);
    Serial.print(F(" UTC="));
    Serial.println((utc.valid >> 2) & 0x01);

    Serial.println();
  } else {
    Serial.println(F("NAV-TIMEUTC poll failed (timeout)"));
  }

  delay(1000);
}
