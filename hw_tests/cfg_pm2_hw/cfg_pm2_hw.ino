/*!
 * @file cfg_pm2_hw.ino
 * @brief Hardware test for CFG-PM2 (Extended Power Management)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("CFG-PM2 Hardware Test"));
  Serial.println(F("====================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_CFG_PM2_t pm2;
  if (ubx.pollCfgPm2(&pm2)) {
    Serial.println(F("PASS: CFG-PM2 poll OK"));
    Serial.print(F("  Version: "));
    Serial.println(pm2.version);
    Serial.print(F("  Update period: "));
    Serial.print(pm2.updatePeriod);
    Serial.println(F(" ms"));
    Serial.print(F("  On time: "));
    Serial.print(pm2.onTime);
    Serial.println(F(" s"));
    Serial.print(F("  Flags: 0x"));
    Serial.println(pm2.flags, HEX);
  } else {
    Serial.println(F("FAIL: CFG-PM2 poll failed"));
  }
}

void loop() {
  delay(1000);
}
