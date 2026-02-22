/*!
 * @file nav_status_test.ino
 *
 * Message test: Poll UBX-NAV-STATUS, validate fields,
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

const __FlashStringHelper* psmLabel(uint8_t psm_state) {
  switch (psm_state) {
    case 0:
      return F("acquisition");
    case 1:
      return F("tracking");
    case 2:
      return F("power optimized");
    case 3:
      return F("inactive");
    default:
      return F("unknown");
  }
}

const __FlashStringHelper* spoofLabel(uint8_t spoof_state) {
  switch (spoof_state) {
    case 0:
      return F("unknown/deactivated");
    case 1:
      return F("no spoofing");
    case 2:
      return F("spoofing indicated");
    case 3:
      return F("multiple spoofing indicated");
    default:
      return F("unknown");
  }
}

uint8_t runTests(const UBX_NAV_STATUS_t& status) {
  uint8_t passed = 0;
  const uint8_t total = 8;

  Serial.println();
  Serial.println(F("Running tests..."));

  bool fix_type = (status.gpsFix == 2) || (status.gpsFix == 3);
  printTestResult(F("fix_type"), fix_type);
  Serial.println(status.gpsFix);
  if (fix_type)
    passed++;

  bool fix_ok_flag = (status.flags & 0x01) != 0;
  printTestResult(F("fix_ok_flag"), fix_ok_flag);
  Serial.println(fix_ok_flag ? 1 : 0);
  if (fix_ok_flag)
    passed++;

  bool wkn_set = (status.flags & 0x04) != 0;
  printTestResult(F("wkn_set"), wkn_set);
  Serial.println(wkn_set ? 1 : 0);
  if (wkn_set)
    passed++;

  bool tow_set = (status.flags & 0x08) != 0;
  printTestResult(F("tow_set"), tow_set);
  Serial.println(tow_set ? 1 : 0);
  if (tow_set)
    passed++;

  bool ttff_valid = (status.ttff > 0) && (status.ttff < 600000);
  printTestResult(F("ttff_valid"), ttff_valid);
  Serial.print(status.ttff);
  Serial.println(F(" ms"));
  if (ttff_valid)
    passed++;

  bool msss_valid = status.msss > 0;
  printTestResult(F("msss_valid"), msss_valid);
  Serial.print(status.msss);
  Serial.println(F(" ms"));
  if (msss_valid)
    passed++;

  uint8_t spoof_state = (status.flags2 >> 3) & 0x03;
  bool spoof_ok = (spoof_state == 0) || (spoof_state == 1);
  printTestResult(F("spoof_state"), spoof_ok);
  Serial.print(spoof_state);
  Serial.print(F(" ("));
  Serial.print(spoofLabel(spoof_state));
  Serial.println(F(")"));
  if (spoof_ok)
    passed++;

  uint8_t psm_state = status.flags2 & 0x07;
  bool psm_ok = psm_state <= 3;
  printTestResult(F("psm_state"), psm_ok);
  Serial.print(psm_state);
  Serial.print(F(" ("));
  Serial.print(psmLabel(psm_state));
  Serial.println(F(")"));
  if (psm_ok)
    passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printStatus(const UBX_NAV_STATUS_t& status) {
  Serial.println(F("--- NAV-STATUS ---"));

  Serial.print(F("Fix type: "));
  Serial.print(status.gpsFix);
  switch (status.gpsFix) {
    case 0:
      Serial.println(F(" (no fix)"));
      break;
    case 1:
      Serial.println(F(" (dead reckoning)"));
      break;
    case 2:
      Serial.println(F(" (2D fix)"));
      break;
    case 3:
      Serial.println(F(" (3D fix)"));
      break;
    case 4:
      Serial.println(F(" (GNSS + dead reckoning)"));
      break;
    case 5:
      Serial.println(F(" (time only)"));
      break;
    default:
      Serial.println(F(" (unknown)"));
      break;
  }

  Serial.print(F("Fix valid: "));
  Serial.println((status.flags & 0x01) ? F("yes") : F("no"));
  Serial.print(F("Differential corrections: "));
  Serial.println(((status.flags >> 1) & 0x01) ? F("applied") : F("none"));
  Serial.print(F("Week number valid: "));
  Serial.println(((status.flags >> 2) & 0x01) ? F("yes") : F("no"));
  Serial.print(F("Time of week valid: "));
  Serial.println(((status.flags >> 3) & 0x01) ? F("yes") : F("no"));

  Serial.print(F("Time to first fix: "));
  Serial.print(status.ttff / 1000.0, 1);
  Serial.println(F(" sec"));
  Serial.print(F("Uptime since reset: "));
  Serial.print(status.msss / 1000.0, 1);
  Serial.println(F(" sec"));

  uint8_t psm_state = status.flags2 & 0x07;
  uint8_t spoof_state = (status.flags2 >> 3) & 0x03;
  uint8_t carr_soln = (status.flags2 >> 6) & 0x03;
  Serial.print(F("Power save mode: "));
  Serial.println(psmLabel(psm_state));
  Serial.print(F("Spoofing detection: "));
  Serial.println(spoofLabel(spoof_state));
  Serial.print(F("Carrier solution: "));
  switch (carr_soln) {
    case 0:
      Serial.println(F("none"));
      break;
    case 1:
      Serial.println(F("floating"));
      break;
    case 2:
      Serial.println(F("fixed"));
      break;
    default:
      Serial.println(F("unknown"));
      break;
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-NAV-STATUS Message Test ==="));

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
  UBX_NAV_STATUS_t status;

  if (!tests_run) {
    attempt++;
    bool got = ubx.poll(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt));

    if (attempt == 1) {
      Serial.println(F("Waiting for fix..."));
    }

    if (got && pvt.fixType >= 2) {
      fix_acquired = true;
      Serial.println(F("Fix acquired!"));
      if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_STATUS, &status, sizeof(status))) {
        runTests(status);
      } else {
        Serial.println(F("NAV-STATUS poll failed (timeout)"));
      }
      tests_run = true;
      printed_continuous_header = false;
      delay(2000);
      return;
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

  if (ubx.poll(UBX_CLASS_NAV, UBX_NAV_STATUS, &status, sizeof(status))) {
    printStatus(status);
  } else {
    Serial.println(F("NAV-STATUS poll failed (timeout)"));
  }

  delay(2000);
}
