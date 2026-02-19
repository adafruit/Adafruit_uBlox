/*!
 * @file nav_pvt_test.ino
 *
 * Message test: Poll UBX-NAV-PVT, wait for fix, validate fields,
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
  Serial.print("  [");
  Serial.print(pass ? "PASS" : "FAIL");
  Serial.print("] ");
  Serial.print(name);
  Serial.print(": ");
}

void printDate(uint16_t year, uint8_t month, uint8_t day) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u", year, month, day);
  Serial.print(buf);
}

void printTime(uint8_t hour, uint8_t min, uint8_t sec) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u", hour, min, sec);
  Serial.print(buf);
}

uint8_t runTests(const UBX_NAV_PVT_t& pvt) {
  uint8_t passed = 0;
  const uint8_t total = 12;

  Serial.println();
  Serial.println("Running tests...");

  bool fix_type = (pvt.fixType == 2) || (pvt.fixType == 3);
  printTestResult(F("fix_type"), fix_type);
  Serial.println(pvt.fixType);
  if (fix_type)
    passed++;

  bool fix_ok_flag = (pvt.flags & 0x01) != 0;
  printTestResult(F("fix_ok_flag"), fix_ok_flag);
  Serial.println(fix_ok_flag ? 1 : 0);
  if (fix_ok_flag)
    passed++;

  bool valid_date = (pvt.valid & 0x01) != 0 && pvt.year >= 2024 &&
                    pvt.year <= 2035 && pvt.month >= 1 && pvt.month <= 12 &&
                    pvt.day >= 1 && pvt.day <= 31;
  printTestResult(F("valid_date"), valid_date);
  printDate(pvt.year, pvt.month, pvt.day);
  Serial.println();
  if (valid_date)
    passed++;

  bool valid_time = (pvt.valid & 0x02) != 0 && pvt.hour <= 23 &&
                    pvt.min <= 59 && pvt.sec <= 60;
  printTestResult(F("valid_time"), valid_time);
  printTime(pvt.hour, pvt.min, pvt.sec);
  Serial.println();
  if (valid_time)
    passed++;

  bool satellites = pvt.numSV >= 3;
  printTestResult(F("satellites"), satellites);
  Serial.println(pvt.numSV);
  if (satellites)
    passed++;

  double lat = pvt.lat * 1e-7;
  bool latitude_range = lat >= 20.0 && lat <= 55.0;
  printTestResult(F("latitude_range"), latitude_range);
  Serial.println(lat, 7);
  if (latitude_range)
    passed++;

  double lon = pvt.lon * 1e-7;
  bool longitude_range = lon >= -130.0 && lon <= -60.0;
  printTestResult(F("longitude_range"), longitude_range);
  Serial.println(lon, 7);
  if (longitude_range)
    passed++;

  double altitude_m = pvt.hMSL / 1000.0;
  bool altitude_range = altitude_m >= -500.0 && altitude_m <= 10000.0;
  printTestResult(F("altitude_range"), altitude_range);
  Serial.print(altitude_m, 1);
  Serial.println(" m");
  if (altitude_range)
    passed++;

  double hacc_m = pvt.hAcc / 1000.0;
  bool accuracy_h = pvt.hAcc < 100000;
  printTestResult(F("accuracy_h"), accuracy_h);
  Serial.print(hacc_m, 1);
  Serial.println(" m");
  if (accuracy_h)
    passed++;

  double vacc_m = pvt.vAcc / 1000.0;
  bool accuracy_v = pvt.vAcc < 200000;
  printTestResult(F("accuracy_v"), accuracy_v);
  Serial.print(vacc_m, 1);
  Serial.println(" m");
  if (accuracy_v)
    passed++;

  double pdop = pvt.pDOP * 0.01;
  bool pdop_ok = pvt.pDOP > 0 && pvt.pDOP < 2000;
  printTestResult(F("pdop"), pdop_ok);
  Serial.println(pdop, 2);
  if (pdop_ok)
    passed++;

  double gspeed_ms = pvt.gSpeed / 1000.0;
  bool ground_speed = pvt.gSpeed >= 0 && pvt.gSpeed < 1000000;
  printTestResult(F("ground_speed"), ground_speed);
  Serial.print(gspeed_ms, 3);
  Serial.println(" m/s");
  if (ground_speed)
    passed++;

  Serial.println();
  Serial.print("Results: ");
  Serial.print(passed);
  Serial.print("/");
  Serial.print(total);
  Serial.println(" tests passed");

  return passed;
}

void printPVT(const UBX_NAV_PVT_t& pvt) {
  Serial.println("--- NAV-PVT ---");

  Serial.print("Time: ");
  printDate(pvt.year, pvt.month, pvt.day);
  Serial.print(" ");
  printTime(pvt.hour, pvt.min, pvt.sec);
  Serial.println();

  Serial.print("Fix type: ");
  Serial.print(pvt.fixType);
  Serial.print("  Sats: ");
  Serial.print(pvt.numSV);
  Serial.print("  Fix OK: ");
  Serial.println(pvt.flags & 0x01);

  Serial.print("Valid: date=");
  Serial.print(pvt.valid & 0x01);
  Serial.print(" time=");
  Serial.print((pvt.valid >> 1) & 0x01);
  Serial.print(" resolved=");
  Serial.println((pvt.valid >> 2) & 0x01);

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

  Serial.print("Ground speed: ");
  Serial.print(pvt.gSpeed / 1000.0, 3);
  Serial.println(" m/s");
  Serial.print("Heading: ");
  Serial.print(pvt.headMot * 1e-5, 1);
  Serial.println(" deg");

  Serial.print("pDOP: ");
  Serial.println(pvt.pDOP * 0.01, 2);

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("=== UBX-NAV-PVT Message Test ===");

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

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print("WARNING: setUBXOnly status: ");
    Serial.println(status);
  } else {
    Serial.println("UBX-only mode set on DDC port");
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
      Serial.println("Waiting for fix...");
    } else {
      Serial.print("Waiting for fix... (attempt ");
      Serial.print(attempt);
      if (got) {
        Serial.print(", fixType=");
        Serial.print(pvt.fixType);
        Serial.print(", sats=");
        Serial.print(pvt.numSV);
        Serial.println(")");
      } else {
        Serial.println(", poll failed)");
      }
    }

    if (got && pvt.fixType == 3 && (pvt.flags & 0x01)) {
      fix_acquired = true;
      Serial.print("Fix acquired! fixType=");
      Serial.print(pvt.fixType);
      Serial.print(", sats=");
      Serial.println(pvt.numSV);
      runTests(pvt);
      tests_run = true;
      printed_continuous_header = false;
      delay(2000);
      return;
    }

    if (millis() - fix_start_ms > FIX_TIMEOUT_MS) {
      Serial.println("Timeout waiting for 3D fix after 120 seconds.");
      tests_run = true;
    }

    delay(1000);
    return;
  }

  if (!printed_continuous_header) {
    Serial.println();
    Serial.println("Continuous output:");
    printed_continuous_header = true;
  }

  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt))) {
    printPVT(pvt);
  } else {
    Serial.println("NAV-PVT poll failed (timeout)");
  }

  delay(2000);
}
