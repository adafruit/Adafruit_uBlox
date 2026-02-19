/*!
 * @file ubx_cfg_nav5.ino
 *
 * Hardware test: Get and set UBX-CFG-NAV5 navigation engine settings.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* getDynModelName(uint8_t model) {
  switch (model) {
    case 0: return "Portable";
    case 2: return "Stationary";
    case 3: return "Pedestrian";
    case 4: return "Automotive";
    case 5: return "Sea";
    case 6: return "Airborne1g";
    case 7: return "Airborne2g";
    case 8: return "Airborne4g";
    default: return "Unknown";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-CFG-NAV5 Hardware Test"));
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
  
  // ubx.verbose_debug = 2;

  // Try setUBXOnly but don't fail if it doesn't work
  // (some modules may have I2C buffer limitations)
  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("Note: setUBXOnly status: "));
    Serial.println(status);
    Serial.println(F("Continuing without UBX-only mode..."));
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }
  
  delay(500); // Wait for any NMEA to clear

  Serial.println();
}

void loop() {
  Serial.println(F("--- Current NAV5 Settings ---"));
  UBX_CFG_NAV5_t nav5;
  if (ubx.pollCfgNav5(&nav5)) {
    Serial.print(F("dynModel: "));
    Serial.print(nav5.dynModel);
    Serial.print(F(" ("));
    Serial.print(getDynModelName(nav5.dynModel));
    Serial.println(F(")"));
    Serial.print(F("fixMode: "));
    Serial.println(nav5.fixMode);
    Serial.print(F("minElev: "));
    Serial.print(nav5.minElev);
    Serial.println(F(" deg"));
    Serial.print(F("pDop: "));
    Serial.println(nav5.pDop * 0.1);
  } else {
    Serial.println(F("pollCfgNav5 failed"));
  }

  Serial.println(F("\nSetting dynamic model to Pedestrian (3)..."));
  if (ubx.setDynamicModel(UBX_DYNMODEL_PEDESTRIAN)) {
    Serial.println(F("setDynamicModel(3) OK"));
  } else {
    Serial.println(F("setDynamicModel(3) FAILED"));
  }
  delay(100);

  uint8_t model = ubx.getDynamicModel();
  Serial.print(F("Current model: "));
  Serial.print(model);
  Serial.print(F(" ("));
  Serial.print(getDynModelName(model));
  Serial.println(F(")"));

  Serial.println(F("\nRestoring to Portable (0)..."));
  if (ubx.setDynamicModel(UBX_DYNMODEL_PORTABLE)) {
    Serial.println(F("setDynamicModel(0) OK"));
  } else {
    Serial.println(F("setDynamicModel(0) FAILED"));
  }

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
