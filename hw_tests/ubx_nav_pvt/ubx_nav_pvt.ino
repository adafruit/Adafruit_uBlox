/*!
 * @file ubx_nav_pvt.ino
 *
 * Hardware test: Poll UBX-NAV-PVT and display all fields
 * Tests the poll() method and NAV-PVT struct parsing.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBloxDDC.h>
#include <Adafruit_UBX.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("UBX-NAV-PVT Poll Test");
  Serial.println("=====================");

  if (!ddc.begin()) {
    Serial.println("FAIL: Could not connect to GPS module!");
    while (1)
      delay(10);
  }
  Serial.println("GPS module connected on I2C");

  if (!ubx.begin()) {
    Serial.println("FAIL: UBX parser init failed!");
    while (1)
      delay(10);
  }

  // Switch to UBX-only mode
  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print("WARNING: setUBXOnly status: ");
    Serial.println(status);
  } else {
    Serial.println("UBX-only mode set on DDC port");
  }

  Serial.println();
}

void loop() {
  UBX_NAV_PVT_t pvt;

  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt))) {
    Serial.println("--- NAV-PVT ---");

    // Time
    Serial.print("Time: ");
    Serial.print(pvt.year);
    Serial.print("-");
    if (pvt.month < 10)
      Serial.print("0");
    Serial.print(pvt.month);
    Serial.print("-");
    if (pvt.day < 10)
      Serial.print("0");
    Serial.print(pvt.day);
    Serial.print(" ");
    if (pvt.hour < 10)
      Serial.print("0");
    Serial.print(pvt.hour);
    Serial.print(":");
    if (pvt.min < 10)
      Serial.print("0");
    Serial.print(pvt.min);
    Serial.print(":");
    if (pvt.sec < 10)
      Serial.print("0");
    Serial.println(pvt.sec);

    // Fix info
    Serial.print("Fix type: ");
    Serial.print(pvt.fixType);
    Serial.print("  Sats: ");
    Serial.print(pvt.numSV);
    Serial.print("  Fix OK: ");
    Serial.println(pvt.flags & 0x01);

    // Valid flags
    Serial.print("Valid: date=");
    Serial.print(pvt.valid & 0x01);
    Serial.print(" time=");
    Serial.print((pvt.valid >> 1) & 0x01);
    Serial.print(" resolved=");
    Serial.println((pvt.valid >> 2) & 0x01);

    // Position
    Serial.print("Lat: ");
    Serial.print(pvt.lat * 1e-7, 7);
    Serial.println(" deg");
    Serial.print("Lon: ");
    Serial.print(pvt.lon * 1e-7, 7);
    Serial.println(" deg");
    Serial.print("Height: ");
    Serial.print(pvt.hMSL / 1000.0, 1);
    Serial.println(" m (MSL)");
    Serial.print("hAcc: ");
    Serial.print(pvt.hAcc / 1000.0, 1);
    Serial.print(" m  vAcc: ");
    Serial.print(pvt.vAcc / 1000.0, 1);
    Serial.println(" m");

    // Velocity
    Serial.print("Ground speed: ");
    Serial.print(pvt.gSpeed / 1000.0, 3);
    Serial.println(" m/s");
    Serial.print("Heading: ");
    Serial.print(pvt.headMot * 1e-5, 1);
    Serial.println(" deg");

    // DOP
    Serial.print("pDOP: ");
    Serial.println(pvt.pDOP * 0.01, 2);

    Serial.println();
  } else {
    Serial.println("NAV-PVT poll failed (timeout)");
  }

  delay(1000);
}
