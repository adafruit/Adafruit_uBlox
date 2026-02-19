/*!
 * @file ubx_nav_pvt.ino
 *
 * Hardware test: Poll UBX-NAV-PVT and display all fields
 * Tests the poll() method and NAV-PVT struct parsing.
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

  Serial.println(F("UBX-NAV-PVT Poll Test"));
  Serial.println(F("====================="));

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

  // Switch to UBX-only mode
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
  UBX_NAV_PVT_t pvt;

  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt))) {
    Serial.println(F("--- NAV-PVT ---"));

    // Time
    Serial.print(F("Time: "));
    Serial.print(pvt.year);
    Serial.print(F("-"));
    if (pvt.month < 10)
      Serial.print(F("0"));
    Serial.print(pvt.month);
    Serial.print(F("-"));
    if (pvt.day < 10)
      Serial.print(F("0"));
    Serial.print(pvt.day);
    Serial.print(F(" "));
    if (pvt.hour < 10)
      Serial.print(F("0"));
    Serial.print(pvt.hour);
    Serial.print(F(":"));
    if (pvt.min < 10)
      Serial.print(F("0"));
    Serial.print(pvt.min);
    Serial.print(F(":"));
    if (pvt.sec < 10)
      Serial.print(F("0"));
    Serial.println(pvt.sec);

    // Fix info
    Serial.print(F("Fix type: "));
    Serial.print(pvt.fixType);
    Serial.print(F("  Sats: "));
    Serial.print(pvt.numSV);
    Serial.print(F("  Fix OK: "));
    Serial.println(pvt.flags & 0x01);

    // Valid flags
    Serial.print(F("Valid: date="));
    Serial.print(pvt.valid & 0x01);
    Serial.print(F(" time="));
    Serial.print((pvt.valid >> 1) & 0x01);
    Serial.print(F(" resolved="));
    Serial.println((pvt.valid >> 2) & 0x01);

    // Position
    Serial.print(F("Lat: "));
    Serial.print(pvt.lat * 1e-7, 7);
    Serial.println(F(" deg"));
    Serial.print(F("Lon: "));
    Serial.print(pvt.lon * 1e-7, 7);
    Serial.println(F(" deg"));
    Serial.print(F("Height: "));
    Serial.print(pvt.hMSL / 1000.0, 1);
    Serial.println(F(" m (MSL)"));
    Serial.print(F("hAcc: "));
    Serial.print(pvt.hAcc / 1000.0, 1);
    Serial.print(F(" m  vAcc: "));
    Serial.print(pvt.vAcc / 1000.0, 1);
    Serial.println(F(" m"));

    // Velocity
    Serial.print(F("Ground speed: "));
    Serial.print(pvt.gSpeed / 1000.0, 3);
    Serial.println(F(" m/s"));
    Serial.print(F("Heading: "));
    Serial.print(pvt.headMot * 1e-5, 1);
    Serial.println(F(" deg"));

    // DOP
    Serial.print(F("pDOP: "));
    Serial.println(pvt.pDOP * 0.01, 2);

    Serial.println();
  } else {
    Serial.println(F("NAV-PVT poll failed (timeout)"));
  }

  delay(1000);
}
