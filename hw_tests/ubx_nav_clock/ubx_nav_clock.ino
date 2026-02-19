/*!
 * @file ubx_nav_clock.ino
 *
 * Hardware test: Poll UBX-NAV-CLOCK and display all fields
 * Tests the pollNAVCLOCK() method and NAV-CLOCK struct parsing.
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

  Serial.println(F("UBX-NAV-CLOCK Poll Test"));
  Serial.println(F("======================="));

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
  UBX_NAV_CLOCK_t clk;

  if (ubx.pollNAVCLOCK(&clk)) {
    Serial.println(F("--- NAV-CLOCK (Clock Solution) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(clk.iTOW);
    Serial.println(F(" ms"));

    Serial.print(F("Clock bias: "));
    Serial.print(clk.clkB);
    Serial.print(F(" ns ("));
    Serial.print(clk.clkB / 1000000.0, 6);
    Serial.println(F(" ms)"));

    Serial.print(F("Clock drift: "));
    Serial.print(clk.clkD);
    Serial.println(F(" ns/s"));

    Serial.print(F("Time accuracy: "));
    Serial.print(clk.tAcc);
    Serial.println(F(" ns"));

    Serial.print(F("Freq accuracy: "));
    Serial.print(clk.fAcc);
    Serial.println(F(" ps/s"));

    Serial.println();
  } else {
    Serial.println(F("NAV-CLOCK poll failed (timeout)"));
  }

  delay(1000);
}
