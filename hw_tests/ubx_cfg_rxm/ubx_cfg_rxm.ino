/*!
 * @file ubx_cfg_rxm.ino
 *
 * Hardware test: Get and set UBX-CFG-RXM receiver manager settings.
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

  Serial.println(F("UBX-CFG-RXM Hardware Test"));
  Serial.println(F("=========================="));

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
  Serial.println(F("--- Receiver Manager Settings ---"));

  UBX_CFG_RXM_t rxm;
  if (ubx.pollCfgRxm(&rxm)) {
    Serial.print(F("reserved1: "));
    Serial.println(rxm.reserved1);
    Serial.print(F("lpMode: "));
    Serial.print(rxm.lpMode);
    Serial.print(F(" ("));
    Serial.print(rxm.lpMode == UBX_RXM_LPMODE_CONTINUOUS ? F("Continuous") : F("Power Save"));
    Serial.println(F(")"));
  } else {
    Serial.println(F("pollCfgRxm failed"));
  }

  Serial.println(F("\nToggling power mode..."));
  bool current_ps = (rxm.lpMode == UBX_RXM_LPMODE_POWERSAVE);
  if (ubx.setPowerSave(!current_ps)) {
    Serial.print(F("Mode now: "));
    Serial.println(!current_ps ? F("Power Save") : F("Continuous"));
  } else {
    Serial.println(F("setPowerSave failed"));
  }

  delay(100);

  // Verify
  if (ubx.pollCfgRxm(&rxm)) {
    Serial.print(F("Verified lpMode: "));
    Serial.println(rxm.lpMode);
  }

  // Restore
  ubx.setPowerSave(current_ps);
  Serial.println(F("Restored original mode"));

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
