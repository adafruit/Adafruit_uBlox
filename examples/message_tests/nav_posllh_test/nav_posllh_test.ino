/*!
 * @file nav_posllh_test.ino
 *
 * Message test: Poll UBX-NAV-POSLLH, wait for fix, validate fields,
 * then stream parsed data for inspection.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

static const uint32_t FIX_TIMEOUT_MS = 120000;

bool tests_run = false;
bool fix_acquired = false;
bool printed_continuous_header = false;
uint32_t attempt = 0;
unsigned long fix_start_ms = 0;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  Serial.print(pass ? F("PASS") : F("FAIL"));
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

uint8_t runTests(const UBX_NAV_POSLLH_t& posllh) {
  uint8_t passed = 0;
  const uint8_t total = 6;

  Serial.println();
  Serial.println(F("Running NAV-POSLLH tests..."));

  // iTOW should be non-zero when we have a fix
  bool itow_ok = posllh.iTOW > 0;
  printTestResult(F("iTOW_nonzero"), itow_ok);
  Serial.println(posllh.iTOW);
  if (itow_ok)
    passed++;

  // Latitude range check: continental US 24-50°N
  double lat = posllh.lat * 1e-7;
  bool lat_ok = lat >= 24.0 && lat <= 50.0;
  printTestResult(F("latitude_range"), lat_ok);
  Serial.println(lat, 7);
  if (lat_ok)
    passed++;

  // Longitude range check: continental US 60-130°W
  double lon = posllh.lon * 1e-7;
  bool lon_ok = lon >= -130.0 && lon <= -60.0;
  printTestResult(F("longitude_range"), lon_ok);
  Serial.println(lon, 7);
  if (lon_ok)
    passed++;

  // Height MSL should be reasonable (-500m to 10000m)
  double hMSL_m = posllh.hMSL / 1000.0;
  bool height_ok = hMSL_m >= -500.0 && hMSL_m <= 10000.0;
  printTestResult(F("height_range"), height_ok);
  Serial.print(hMSL_m, 1);
  Serial.println(F(" m"));
  if (height_ok)
    passed++;

  // Horizontal accuracy should be < 100m
  double hAcc_m = posllh.hAcc / 1000.0;
  bool hAcc_ok = posllh.hAcc < 100000;
  printTestResult(F("hAcc_reasonable"), hAcc_ok);
  Serial.print(hAcc_m, 1);
  Serial.println(F(" m"));
  if (hAcc_ok)
    passed++;

  // Vertical accuracy should be < 200m
  double vAcc_m = posllh.vAcc / 1000.0;
  bool vAcc_ok = posllh.vAcc < 200000;
  printTestResult(F("vAcc_reasonable"), vAcc_ok);
  Serial.print(vAcc_m, 1);
  Serial.println(F(" m"));
  if (vAcc_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printPOSLLH(const UBX_NAV_POSLLH_t& posllh) {
  Serial.println(F("--- NAV-POSLLH ---"));

  Serial.print(F("iTOW: "));
  Serial.print(posllh.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("Lat: "));
  Serial.print(posllh.lat * 1e-7, 7);
  Serial.println(F(" deg"));

  Serial.print(F("Lon: "));
  Serial.print(posllh.lon * 1e-7, 7);
  Serial.println(F(" deg"));

  Serial.print(F("Height (ellipsoid): "));
  Serial.print(posllh.height / 1000.0, 3);
  Serial.println(F(" m"));

  Serial.print(F("Height (MSL): "));
  Serial.print(posllh.hMSL / 1000.0, 3);
  Serial.println(F(" m"));

  Serial.print(F("hAcc: "));
  Serial.print(posllh.hAcc / 1000.0, 3);
  Serial.println(F(" m"));

  Serial.print(F("vAcc: "));
  Serial.print(posllh.vAcc / 1000.0, 3);
  Serial.println(F(" m"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-POSLLH Message Test ==="));

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
  UBX_NAV_POSLLH_t posllh;

  if (!tests_run) {
    attempt++;
    bool got = ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt));

    if (attempt == 1) {
      Serial.println(F("Waiting for 3D fix..."));
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

    if (got && pvt.fixType == 3 && (pvt.flags & 0x01)) {
      fix_acquired = true;
      Serial.print(F("Fix acquired! fixType="));
      Serial.print(pvt.fixType);
      Serial.print(F(", sats="));
      Serial.println(pvt.numSV);

      // Now poll NAV-POSLLH and run tests
      if (ubx.pollNAVPOSLLH(&posllh)) {
        runTests(posllh);
      } else {
        Serial.println(F("FAIL: Could not poll NAV-POSLLH after fix!"));
      }
      tests_run = true;
      printed_continuous_header = false;
      delay(2000);
      return;
    }

    if (millis() - fix_start_ms > FIX_TIMEOUT_MS) {
      Serial.println(F("Timeout waiting for 3D fix after 120 seconds."));
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

  if (ubx.pollNAVPOSLLH(&posllh)) {
    printPOSLLH(posllh);
  } else {
    Serial.println(F("NAV-POSLLH poll failed (timeout)"));
  }

  delay(2000);
}
