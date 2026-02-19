/*!
 * @file ubx_nav_status.ino
 *
 * Hardware test: Poll UBX-NAV-STATUS and display all fields
 * Tests the poll() method and NAV-STATUS struct parsing.
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

  Serial.println(F("UBX-NAV-STATUS Poll Test"));
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
  UBX_NAV_STATUS_t status;

  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_STATUS, &status, sizeof(status))) {
    Serial.println(F("--- NAV-STATUS ---"));

    Serial.print(F("iTOW: "));
    Serial.print(status.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("gpsFix: "));
    Serial.println(status.gpsFix);

    Serial.print(F("flags: 0x"));
    Serial.println(status.flags, HEX);

    Serial.print(F("fixStat: 0x"));
    Serial.println(status.fixStat, HEX);

    Serial.print(F("flags2: 0x"));
    Serial.println(status.flags2, HEX);

    Serial.print(F("ttff: "));
    Serial.print(status.ttff);
    Serial.println(F(" ms"));

    Serial.print(F("msss: "));
    Serial.print(status.msss);
    Serial.println(F(" ms"));

    Serial.println();
  } else {
    Serial.println(F("NAV-STATUS poll failed (timeout)"));
  }

  delay(1000);
}
