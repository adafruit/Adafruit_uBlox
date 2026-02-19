/*!
 * @file ubx_nav_eoe.ino
 *
 * Hardware test: Poll UBX-NAV-EOE and display all fields
 * Tests the pollNAVEOE() method and NAV-EOE struct parsing.
 *
 * Note: NAV-EOE is normally sent at end of each nav epoch,
 * but can also be polled.
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

  Serial.println(F("UBX-NAV-EOE Poll Test"));
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
  UBX_NAV_EOE_t eoe;

  if (ubx.pollNAVEOE(&eoe)) {
    Serial.println(F("--- NAV-EOE (End of Epoch) ---"));

    Serial.print(F("iTOW: "));
    Serial.print(eoe.iTOW);
    Serial.println(F(" ms"));

    // Convert iTOW to time of week
    uint32_t seconds = eoe.iTOW / 1000;
    uint32_t days = seconds / 86400;
    uint32_t hours = (seconds % 86400) / 3600;
    uint32_t mins = (seconds % 3600) / 60;
    uint32_t secs = seconds % 60;

    Serial.print(F("GPS week time: day "));
    Serial.print(days);
    Serial.print(F(" "));
    char buf[16];
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)hours,
             (unsigned long)mins, (unsigned long)secs);
    Serial.println(buf);

    Serial.println();
  } else {
    Serial.println(F("NAV-EOE poll failed (timeout)"));
  }

  delay(1000);
}
