/*!
 * @file nav_sat_test.ino
 *
 * Message test: Poll UBX-NAV-SAT, wait for fix, validate fields,
 * then stream parsed data for inspection.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

#define MAX_SVS 40

static const uint32_t FIX_TIMEOUT_MS = 120000;

UBX_NAV_SAT_sv_t svs[MAX_SVS];
UBX_NAV_SAT_header_t satHeader;

bool tests_run = false;
bool printed_continuous_header = false;
uint32_t attempt = 0;
unsigned long fix_start_ms = 0;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
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
    snprintf(line, sizeof(line), "%2d | %-8s | %3d | %3d | %4d | %4d | %-4s | ",
             i + 1, gnss, sv.svId, sv.cno, sv.elev, sv.azim,
             used ? "yes " : "no  ");
    Serial.print(line);
    Serial.println(qualityName(sv.flags));
  }
}

uint8_t runTests(uint8_t svsRead) {
  uint8_t passed = 0;
  const uint8_t total = 9;

  Serial.println();
  Serial.println(F("Running tests..."));

  bool got_satellites = svsRead > 0;
  printTestResult(F("got_satellites"), got_satellites);
  Serial.println(svsRead);
  if (got_satellites)
    passed++;

  bool numsvs_positive = satHeader.numSvs > 0;
  printTestResult(F("numsvs_positive"), numsvs_positive);
  Serial.println(satHeader.numSvs);
  if (numsvs_positive)
    passed++;

  bool version_valid = satHeader.version == 1;
  printTestResult(F("version_valid"), version_valid);
  Serial.println(satHeader.version);
  if (version_valid)
    passed++;

  bool gnssid_valid = true;
  bool cno_range = true;
  bool elevation_range = true;
  bool azimuth_range = true;
  bool has_gps = false;
  bool some_used = false;

  for (uint8_t i = 0; i < svsRead; i++) {
    UBX_NAV_SAT_sv_t& sv = svs[i];
    if (sv.gnssId > 6) {
      gnssid_valid = false;
    }
    if (sv.cno > 0 && sv.cno >= 60) {
      cno_range = false;
    }
    if (sv.elev < -90 || sv.elev > 90) {
      elevation_range = false;
    }
    if (sv.azim < 0 || sv.azim > 360) {
      azimuth_range = false;
    }
    if (sv.gnssId == 0) {
      has_gps = true;
    }
    if (sv.flags & (1 << 3)) {
      some_used = true;
    }
  }

  printTestResult(F("gnssid_valid"), gnssid_valid);
  Serial.println(gnssid_valid ? F("OK") : F("BAD"));
  if (gnssid_valid)
    passed++;

  printTestResult(F("cno_range"), cno_range);
  Serial.println(cno_range ? F("OK") : F("BAD"));
  if (cno_range)
    passed++;

  printTestResult(F("elevation_range"), elevation_range);
  Serial.println(elevation_range ? F("OK") : F("BAD"));
  if (elevation_range)
    passed++;

  printTestResult(F("azimuth_range"), azimuth_range);
  Serial.println(azimuth_range ? F("OK") : F("BAD"));
  if (azimuth_range)
    passed++;

  printTestResult(F("has_gps"), has_gps);
  Serial.println(has_gps ? F("OK") : F("BAD"));
  if (has_gps)
    passed++;

  printTestResult(F("some_used"), some_used);
  Serial.println(some_used ? F("OK") : F("BAD"));
  if (some_used)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-SAT Message Test ==="));

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
  fix_start_ms = millis();
}

void loop() {
  UBX_NAV_PVT_t pvt;

  if (!tests_run) {
    attempt++;
    bool got = ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt));

    if (attempt == 1) {
      Serial.println(F("Waiting for fix..."));
    } else {
      Serial.print(F("Waiting for fix... (attempt "));
      Serial.print(attempt);
      if (got) {
        Serial.print(F(", fixType="));
        Serial.print(pvt.fixType);
        Serial.print(F(", sats="));
        Serial.print(pvt.numSV);
        Serial.println(F(")"));
      } else {
        Serial.println(F(", poll failed)"));
      }
    }

    bool fix_ok = got && pvt.fixType >= 2 && (pvt.flags & 0x01);
    if (fix_ok) {
      Serial.print(F("Fix acquired! fixType="));
      Serial.print(pvt.fixType);
      Serial.print(F(", sats="));
      Serial.println(pvt.numSV);

      uint8_t svsRead = ubx.pollNAVSAT(&satHeader, svs, MAX_SVS, 1000);
      if (svsRead > 0) {
        runTests(svsRead);
        tests_run = true;
        printed_continuous_header = false;
        delay(2000);
        return;
      } else {
        Serial.println(F("NAV-SAT poll failed; retrying..."));
      }
    }

    if (millis() - fix_start_ms > FIX_TIMEOUT_MS) {
      Serial.println(F("Timeout waiting for fix after 120 seconds."));
      tests_run = true;
    }

    delay(1000);
    return;
  }

  if (!printed_continuous_header) {
    Serial.println();
    Serial.println(F("Continuous output:"));
    printed_continuous_header = true;
  }

  uint8_t svsRead = ubx.pollNAVSAT(&satHeader, svs, MAX_SVS, 1000);
  if (svsRead > 0) {
    printSatTable(svsRead);
  } else {
    Serial.println(F("NAV-SAT poll failed (timeout)"));
  }

  delay(2000);
}
