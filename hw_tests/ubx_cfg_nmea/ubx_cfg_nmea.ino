/*!
 * @file ubx_cfg_nmea.ino
 *
 * Hardware test: Get and set UBX-CFG-NMEA protocol settings.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* getNmeaVersionName(uint8_t ver) {
  switch (ver) {
    case 0x21: return "2.1";
    case 0x23: return "2.3";
    case 0x40: return "4.0";
    case 0x41: return "4.10";
    case 0x4B: return "4.11";
    default: return "Unknown";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-CFG-NMEA Hardware Test"));
  Serial.println(F("==========================="));

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
  Serial.println(F("--- NMEA Protocol Settings ---"));

  UBX_CFG_NMEA_t nmea;
  if (ubx.pollCfgNmea(&nmea)) {
    Serial.print(F("NMEA Version: "));
    Serial.print(getNmeaVersionName(nmea.nmeaVersion));
    Serial.print(F(" (0x"));
    Serial.print(nmea.nmeaVersion, HEX);
    Serial.println(F(")"));

    Serial.print(F("Max SVs: "));
    Serial.println(nmea.numSV == 0 ? F("unlimited") : String(nmea.numSV));

    Serial.print(F("Flags: 0x"));
    Serial.println(nmea.flags, HEX);
    Serial.print(F("  compat: "));
    Serial.println((nmea.flags & UBX_NMEA_FLAGS_COMPAT) ? F("yes") : F("no"));
    Serial.print(F("  limit82: "));
    Serial.println((nmea.flags & UBX_NMEA_FLAGS_LIMIT82) ? F("yes") : F("no"));
    Serial.print(F("  highPrec: "));
    Serial.println((nmea.flags & UBX_NMEA_FLAGS_HIGHPREC) ? F("yes") : F("no"));

    Serial.print(F("mainTalkerId: "));
    Serial.println(nmea.mainTalkerId);
  } else {
    Serial.println(F("pollCfgNmea failed"));
  }

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
