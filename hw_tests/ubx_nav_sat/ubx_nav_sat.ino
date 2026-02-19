/*!
 * @file ubx_nav_sat.ino
 *
 * Hardware test: Poll UBX-NAV-SAT and display all satellite fields.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

#define MAX_SVS 40

UBX_NAV_SAT_sv_t svs[MAX_SVS];
UBX_NAV_SAT_header_t satHeader;

const __FlashStringHelper* qualityName(uint32_t flags) {
  uint8_t quality = flags & 0x07;
  switch (quality) {
    case 0:
      return F("no signal");
    case 1:
      return F("searching");
    case 2:
      return F("acquired");
    case 3:
      return F("unusable");
    case 4:
      return F("code locked");
    default:
      return F("code+carrier locked");
  }
}

void printSatTable(uint8_t svsRead) {
  Serial.println();
  Serial.print(F("--- NAV-SAT ("));
  Serial.print(svsRead);
  Serial.print(F(" of "));
  Serial.print(satHeader.numSvs);
  Serial.println(F(" satellites read) ---"));
  Serial.println(F(" # | GNSS     | SV | CNO | Elev | Azim | Used | Quality"));
  Serial.println(F("---+----------+----+-----+------+------+------+--------"));

  for (uint8_t i = 0; i < svsRead; i++) {
    UBX_NAV_SAT_sv_t& sv = svs[i];
    bool used = (sv.flags & (1 << 3)) != 0;

    char line[80];
    const char* gnss;
    switch (sv.gnssId) {
      case 0:
        gnss = "GPS";
        break;
      case 1:
        gnss = "SBAS";
        break;
      case 2:
        gnss = "Galileo";
        break;
      case 3:
        gnss = "BeiDou";
        break;
      case 5:
        gnss = "IMES";
        break;
      case 6:
        gnss = "GLONASS";
        break;
      default:
        gnss = "???";
        break;
    }
    snprintf(line, sizeof(line), "%2d | %-8s | %3d | %3d | %4d | %4d | %-4s| ",
             i + 1, gnss, sv.svId, sv.cno, sv.elev, sv.azim,
             used ? "yes " : "no  ");
    Serial.print(line);
    Serial.println(qualityName(sv.flags));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-NAV-SAT Poll Test"));
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
  uint8_t svsRead = ubx.pollNAVSAT(&satHeader, svs, MAX_SVS, 1000);

  if (svsRead > 0) {
    printSatTable(svsRead);
  } else {
    Serial.println(F("NAV-SAT poll failed (timeout)"));
  }

  delay(2000);
}
