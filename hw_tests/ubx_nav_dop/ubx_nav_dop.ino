/*!
 * @file ubx_nav_dop.ino
 *
 * Hardware test: Poll UBX-NAV-DOP and display all fields
 * Tests the poll() method and NAV-DOP struct parsing.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

double dopScaled(uint16_t value) {
  return value / 100.0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-NAV-DOP Poll Test"));
  Serial.println(F("===================="));

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
  UBX_NAV_DOP_t dop;

  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_DOP, &dop, sizeof(dop))) {
    Serial.println(F("--- NAV-DOP (Dilution of Precision) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(dop.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("Geometric DOP: "));
    Serial.println(dopScaled(dop.gDOP), 2);

    Serial.print(F("Position DOP: "));
    Serial.println(dopScaled(dop.pDOP), 2);

    Serial.print(F("Time DOP: "));
    Serial.println(dopScaled(dop.tDOP), 2);

    Serial.print(F("Vertical DOP: "));
    Serial.println(dopScaled(dop.vDOP), 2);

    Serial.print(F("Horizontal DOP: "));
    Serial.println(dopScaled(dop.hDOP), 2);

    Serial.print(F("Northing DOP: "));
    Serial.println(dopScaled(dop.nDOP), 2);

    Serial.print(F("Easting DOP: "));
    Serial.println(dopScaled(dop.eDOP), 2);

    Serial.println();
  } else {
    Serial.println(F("NAV-DOP poll failed (timeout)"));
  }

  delay(1000);
}
