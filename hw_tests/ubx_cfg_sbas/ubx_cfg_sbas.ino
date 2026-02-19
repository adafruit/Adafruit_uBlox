/*!
 * @file ubx_cfg_sbas.ino
 *
 * Hardware test: Get and set UBX-CFG-SBAS settings.
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

  Serial.println(F("UBX-CFG-SBAS Hardware Test"));
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
  Serial.println(F("--- SBAS Settings ---"));

  UBX_CFG_SBAS_t sbas;
  if (ubx.pollCfgSbas(&sbas)) {
    Serial.print(F("mode: 0x"));
    Serial.println(sbas.mode, HEX);
    Serial.print(F("  enabled: "));
    Serial.println((sbas.mode & UBX_SBAS_MODE_ENABLED) ? F("yes") : F("no"));
    Serial.print(F("  test: "));
    Serial.println((sbas.mode & UBX_SBAS_MODE_TEST) ? F("yes") : F("no"));

    Serial.print(F("usage: 0x"));
    Serial.println(sbas.usage, HEX);
    Serial.print(F("  range: "));
    Serial.println((sbas.usage & UBX_SBAS_USAGE_RANGE) ? F("yes") : F("no"));
    Serial.print(F("  diffCorr: "));
    Serial.println((sbas.usage & UBX_SBAS_USAGE_DIFFCORR) ? F("yes") : F("no"));
    Serial.print(F("  integrity: "));
    Serial.println((sbas.usage & UBX_SBAS_USAGE_INTEGRITY) ? F("yes") : F("no"));

    Serial.print(F("maxSBAS: "));
    Serial.println(sbas.maxSBAS);
    Serial.print(F("scanmode1: 0x"));
    Serial.println(sbas.scanmode1, HEX);
  } else {
    Serial.println(F("pollCfgSbas failed"));
  }

  Serial.println(F("\nToggling SBAS enable..."));
  bool current = (sbas.mode & UBX_SBAS_MODE_ENABLED) != 0;
  if (ubx.enableSBAS(!current)) {
    Serial.print(F("SBAS now: "));
    Serial.println(!current ? F("enabled") : F("disabled"));
  } else {
    Serial.println(F("enableSBAS failed"));
  }

  delay(100);

  // Restore
  ubx.enableSBAS(current);

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
