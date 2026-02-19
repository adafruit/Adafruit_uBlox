/*!
 * @file ubx_cfg_cfg.ino
 *
 * Hardware test: Demo save/load configuration via UBX-CFG-CFG.
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

  Serial.println(F("UBX-CFG-CFG Hardware Test"));
  Serial.println(F("========================="));

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

void printRate() {
  UBX_CFG_RATE_t rate;
  if (ubx.getRate(&rate)) {
    Serial.print(F("Current measRate: "));
    Serial.print(rate.measRate);
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("getRate failed"));
  }
}

void loop() {
  Serial.println(F("--- Save Config Demo ---"));
  printRate();

  Serial.println(F("\nSetting rate to 5 Hz..."));
  ubx.setRate(200);
  delay(100);
  printRate();

  Serial.println(F("\nCalling saveConfig()..."));
  if (ubx.saveConfig()) {
    Serial.println(F("saveConfig() OK - rate saved to NVM"));
  } else {
    Serial.println(F("saveConfig() FAILED"));
  }

  Serial.println(F("\n--- Load Defaults Demo ---"));
  Serial.println(F("Calling loadDefaults()..."));
  if (ubx.loadDefaults()) {
    Serial.println(F("loadDefaults() OK"));
  } else {
    Serial.println(F("loadDefaults() FAILED"));
  }
  delay(100);
  printRate();

  Serial.println(F("\nSaving defaults back to NVM..."));
  if (ubx.saveConfig()) {
    Serial.println(F("saveConfig() OK"));
  } else {
    Serial.println(F("saveConfig() FAILED"));
  }

  Serial.println(F("\n--- Done ---\n"));
  delay(10000);
}
