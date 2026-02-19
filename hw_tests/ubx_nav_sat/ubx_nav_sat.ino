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

uint8_t printGnssName(uint8_t gnssId) {
  switch (gnssId) {
    case 0:
      Serial.print(F("GPS"));
      return 3;
    case 1:
      Serial.print(F("SBAS"));
      return 4;
    case 2:
      Serial.print(F("Galileo"));
      return 7;
    case 3:
      Serial.print(F("BeiDou"));
      return 6;
    case 5:
      Serial.print(F("IMES"));
      return 4;
    case 6:
      Serial.print(F("GLONASS"));
      return 7;
    default:
      Serial.print(F("Unknown"));
      return 7;
  }
}

const __FlashStringHelper* qualityName(uint8_t flags) {
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

    if (i + 1 < 10) {
      Serial.print(F(" "));
    }
    Serial.print(i + 1);
    Serial.print(F(" | "));

    uint8_t gnssLen = printGnssName(sv.gnssId);
    for (uint8_t pad = gnssLen; pad < 8; pad++) {
      Serial.print(F(" "));
    }

    Serial.print(F(" | "));
    if (sv.svId < 10) {
      Serial.print(F("0"));
    }
    Serial.print(sv.svId);
    Serial.print(F(" | "));
    if (sv.cno < 100) {
      Serial.print(F(" "));
    }
    if (sv.cno < 10) {
      Serial.print(F(" "));
    }
    Serial.print(sv.cno);
    Serial.print(F(" | "));
    if (sv.elev >= 0 && sv.elev < 10) {
      Serial.print(F(" "));
    }
    Serial.print(sv.elev);
    Serial.print(F(" | "));
    if (sv.azim < 100) {
      Serial.print(F(" "));
    }
    if (sv.azim < 10) {
      Serial.print(F(" "));
    }
    Serial.print(sv.azim);
    Serial.print(F(" | "));
    if (used) {
      Serial.print(F("yes"));
    } else {
      Serial.print(F("no"));
    }
    Serial.print(F(" | "));
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
