/*!
 * @file cfg_geofence_test.ino
 *
 * Message test: Configure geofence around current position and verify status.
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

// Store position for geofence
int32_t fence_lat = 0;
int32_t fence_lon = 0;

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  Serial.print(pass ? F("PASS") : F("FAIL"));
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

const char* getStateStr(uint8_t state) {
  switch (state) {
    case 0:
      return "Unknown";
    case 1:
      return "Inside";
    case 2:
      return "Outside";
    default:
      return "Invalid";
  }
}

uint8_t runTests(int32_t lat, int32_t lon) {
  uint8_t passed = 0;
  const uint8_t total = 6;

  Serial.println();
  Serial.println(F("Running CFG-GEOFENCE tests..."));

  // Test 1: Clear any existing geofences
  bool clear_ok = ubx.clearGeofence();
  printTestResult(F("clear_geofence"), clear_ok);
  Serial.println(clear_ok ? F("OK") : F("FAIL"));
  if (clear_ok)
    passed++;

  delay(500);

  // Test 2: Set a geofence around current position (100m radius)
  // Radius is in cm, so 100m = 10000cm
  bool set_ok = ubx.setGeofence(lat, lon, 10000, 2); // 95% confidence
  printTestResult(F("set_geofence"), set_ok);
  Serial.println(set_ok ? F("OK") : F("FAIL"));
  if (set_ok)
    passed++;

  delay(500);

  // Test 3: Poll CFG-GEOFENCE to verify configuration
  UBX_CFG_GEOFENCE_header_t cfgHeader;
  UBX_CFG_GEOFENCE_fence_t cfgFences[4];
  uint8_t numFences = ubx.pollCfgGeofence(&cfgHeader, cfgFences, 4);
  bool cfg_ok = (numFences == 1);
  printTestResult(F("config_poll"), cfg_ok);
  Serial.print(numFences);
  Serial.println(F(" fence(s)"));
  if (cfg_ok)
    passed++;

  // Test 4: Verify fence parameters
  bool params_ok = false;
  if (numFences > 0) {
    params_ok = (cfgFences[0].lat == lat) && (cfgFences[0].lon == lon) &&
                (cfgFences[0].radius == 10000);
    printTestResult(F("fence_params"), params_ok);
    Serial.print(F("lat="));
    Serial.print(cfgFences[0].lat);
    Serial.print(F(", lon="));
    Serial.print(cfgFences[0].lon);
    Serial.print(F(", radius="));
    Serial.print(cfgFences[0].radius);
    Serial.println(F(" cm"));
  } else {
    printTestResult(F("fence_params"), false);
    Serial.println(F("no fences"));
  }
  if (params_ok)
    passed++;

  delay(1000); // Wait for status update

  // Test 5: Poll NAV-GEOFENCE to check status
  UBX_NAV_GEOFENCE_header_t navHeader;
  UBX_NAV_GEOFENCE_fence_t navFences[4];
  uint8_t statusFences = ubx.pollNavGeofence(&navHeader, navFences, 4);
  bool status_ok = (statusFences >= 1) && (navHeader.status == 1);
  printTestResult(F("status_poll"), status_ok);
  Serial.print(F("status="));
  Serial.print(navHeader.status);
  Serial.print(F(", fences="));
  Serial.println(statusFences);
  if (status_ok)
    passed++;

  // Test 6: We should be inside the fence (if we set it around our position)
  bool inside_ok = false;
  if (statusFences > 0) {
    inside_ok = (navFences[0].state == 1); // 1 = Inside
    printTestResult(F("inside_fence"), inside_ok);
    Serial.print(F("state="));
    Serial.print(navFences[0].state);
    Serial.print(F(" ("));
    Serial.print(getStateStr(navFences[0].state));
    Serial.println(F(")"));
  } else {
    printTestResult(F("inside_fence"), false);
    Serial.println(F("no status"));
  }
  if (inside_ok)
    passed++;

  // Clean up: clear geofence
  ubx.clearGeofence();

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));

  return passed;
}

void printGeofenceStatus() {
  UBX_NAV_GEOFENCE_header_t header;
  UBX_NAV_GEOFENCE_fence_t fences[4];

  uint8_t numFences = ubx.pollNavGeofence(&header, fences, 4);

  Serial.println(F("--- NAV-GEOFENCE ---"));
  Serial.print(F("iTOW: "));
  Serial.print(header.iTOW);
  Serial.println(F(" ms"));

  Serial.print(F("status: "));
  Serial.println(header.status == 1 ? F("active") : F("not available"));

  Serial.print(F("numFences: "));
  Serial.println(header.numFences);

  Serial.print(F("combState: "));
  Serial.print(header.combState);
  Serial.print(F(" ("));
  Serial.print(getStateStr(header.combState));
  Serial.println(F(")"));

  for (uint8_t i = 0; i < numFences; i++) {
    Serial.print(F("  Fence "));
    Serial.print(i);
    Serial.print(F(": state="));
    Serial.print(fences[i].state);
    Serial.print(F(" ("));
    Serial.print(getStateStr(fences[i].state));
    Serial.print(F("), id="));
    Serial.println(fences[i].id);
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("=== UBX-CFG-GEOFENCE Message Test ==="));

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
      Serial.println(F("Waiting for 3D fix (need position for geofence)..."));
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
      fence_lat = pvt.lat;
      fence_lon = pvt.lon;

      Serial.print(F("Fix acquired at lat="));
      Serial.print(fence_lat / 10000000.0, 7);
      Serial.print(F(", lon="));
      Serial.println(fence_lon / 10000000.0, 7);

      runTests(fence_lat, fence_lon);
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
    Serial.println(F("Continuous output (geofence cleared):"));
    printed_continuous_header = true;
  }

  printGeofenceStatus();
  delay(2000);
}
