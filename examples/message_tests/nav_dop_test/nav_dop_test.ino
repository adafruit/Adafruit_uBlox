/*!
 * @file nav_dop_test.ino
 *
 * Message test: Poll UBX-NAV-DOP, validate fields,
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
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

double dopScaled(uint16_t value) {
  return value / 100.0;
}

uint8_t runTests(const UBX_NAV_DOP_t& dop) {
  uint8_t passed = 0;
  const uint8_t total = 9;

  Serial.println();
  Serial.println(F("Running tests..."));

  double g_dop = dopScaled(dop.gDOP);
  bool gdop_range = (dop.gDOP > 0) && (dop.gDOP < 5000);
  printTestResult(F("gdop_range"), gdop_range);
  Serial.println(g_dop, 2);
  if (gdop_range)
    passed++;

  double p_dop = dopScaled(dop.pDOP);
  bool pdop_range = (dop.pDOP > 0) && (dop.pDOP < 5000);
  printTestResult(F("pdop_range"), pdop_range);
  Serial.println(p_dop, 2);
  if (pdop_range)
    passed++;

  double t_dop = dopScaled(dop.tDOP);
  bool tdop_range = (dop.tDOP > 0) && (dop.tDOP < 5000);
  printTestResult(F("tdop_range"), tdop_range);
  Serial.println(t_dop, 2);
  if (tdop_range)
    passed++;

  double v_dop = dopScaled(dop.vDOP);
  bool vdop_range = (dop.vDOP > 0) && (dop.vDOP < 5000);
  printTestResult(F("vdop_range"), vdop_range);
  Serial.println(v_dop, 2);
  if (vdop_range)
    passed++;

  double h_dop = dopScaled(dop.hDOP);
  bool hdop_range = (dop.hDOP > 0) && (dop.hDOP < 5000);
  printTestResult(F("hdop_range"), hdop_range);
  Serial.println(h_dop, 2);
  if (hdop_range)
    passed++;

  double n_dop = dopScaled(dop.nDOP);
  bool ndop_range = (dop.nDOP > 0) && (dop.nDOP < 5000);
  printTestResult(F("ndop_range"), ndop_range);
  Serial.println(n_dop, 2);
  if (ndop_range)
    passed++;

  double e_dop = dopScaled(dop.eDOP);
  bool edop_range = (dop.eDOP > 0) && (dop.eDOP < 5000);
  printTestResult(F("edop_range"), edop_range);
  Serial.println(e_dop, 2);
  if (edop_range)
    passed++;

  bool hdop_le_pdop = dop.hDOP <= dop.pDOP;
  printTestResult(F("hdop_le_pdop"), hdop_le_pdop);
  Serial.print(h_dop, 2);
  Serial.print(F(" <= "));
  Serial.println(p_dop, 2);
  if (hdop_le_pdop)
    passed++;

  bool pdop_le_gdop = dop.pDOP <= dop.gDOP;
  printTestResult(F("pdop_le_gdop"), pdop_le_gdop);
  Serial.print(p_dop, 2);
  Serial.print(F(" <= "));
  Serial.println(g_dop, 2);
  if (pdop_le_gdop)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printDOP(const UBX_NAV_DOP_t& dop) {
  Serial.println(F("--- NAV-DOP (Dilution of Precision) ---"));

  Serial.print(F("Geometric DOP: "));
  Serial.println(dopScaled(dop.gDOP), 2);

  Serial.print(F("Position DOP: "));
  Serial.println(dopScaled(dop.pDOP), 2);

  Serial.print(F("Time DOP: "));
  Serial.println(dopScaled(dop.tDOP), 2);

  Serial.print(F("Vertical DOP: "));
  Serial.println(dopScaled(dop.vDOP), 2);

  Serial.print(F("Horizontal DOP: "));
  Serial.println(dopScaled(dop.hDOP), 2);

  Serial.print(F("Northing DOP: "));
  Serial.println(dopScaled(dop.nDOP), 2);

  Serial.print(F("Easting DOP: "));
  Serial.println(dopScaled(dop.eDOP), 2);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-DOP Message Test ==="));

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
  UBX_NAV_DOP_t dop;

  if (!tests_run) {
    if (!fix_acquired) {
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

      if (got && pvt.fixType >= 2 && (pvt.flags & 0x01)) {
        fix_acquired = true;
        Serial.print(F("Fix acquired! fixType="));
        Serial.print(pvt.fixType);
        Serial.print(F(", sats="));
        Serial.println(pvt.numSV);
      }

      if (millis() - fix_start_ms > FIX_TIMEOUT_MS) {
        Serial.println(F("Timeout waiting for fix after 120 seconds."));
        tests_run = true;
      }

      delay(1000);
      return;
    }

    if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_DOP, &dop, sizeof(dop))) {
      runTests(dop);
      tests_run = true;
      printed_continuous_header = false;
      delay(2000);
      return;
    }

    Serial.println(F("NAV-DOP poll failed (timeout)"));
    delay(1000);
    return;
  }

  if (!printed_continuous_header) {
    Serial.println();
    Serial.println(F("Continuous output:"));
    printed_continuous_header = true;
  }

  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_DOP, &dop, sizeof(dop))) {
    printDOP(dop);
  } else {
    Serial.println(F("NAV-DOP poll failed (timeout)"));
  }

  delay(2000);
}
