/*!
 * @file cfg_pms_hw.ino
 * @brief Hardware test for CFG-PMS (Power Mode Setup)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* modeName(uint8_t m) {
  switch (m) {
    case 0: return "Full";
    case 1: return "Balanced";
    case 2: return "Interval";
    case 3: return "Aggr1Hz";
    case 4: return "Aggr2Hz";
    case 5: return "Aggr4Hz";
    default: return "???";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("CFG-PMS Hardware Test"));
  Serial.println(F("====================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_CFG_PMS_t pms;
  if (ubx.pollCfgPms(&pms)) {
    Serial.println(F("PASS: CFG-PMS poll OK"));
    Serial.print(F("  Mode: "));
    Serial.println(modeName(pms.powerSetupValue));
  } else {
    Serial.println(F("FAIL: CFG-PMS poll failed"));
  }

  // Test setPowerMode
  if (ubx.setPowerMode(UBX_PMS_FULLPOWER)) {
    Serial.println(F("PASS: setPowerMode(FULL) OK"));
  } else {
    Serial.println(F("FAIL: setPowerMode(FULL) failed"));
  }
}

void loop() {
  delay(1000);
}
