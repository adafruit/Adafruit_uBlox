/*!
 * @file ubx_cfg_ant.ino
 *
 * Hardware test: Get UBX-CFG-ANT antenna control settings.
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

  Serial.println(F("UBX-CFG-ANT Hardware Test"));
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
  Serial.println(F("--- Antenna Settings ---"));

  UBX_CFG_ANT_t ant;
  if (ubx.pollCfgAnt(&ant)) {
    Serial.print(F("flags: 0x"));
    Serial.println(ant.flags, HEX);
    Serial.print(F("  svcs (supply voltage control): "));
    Serial.println((ant.flags & UBX_ANT_FLAG_SVCS) ? F("enabled") : F("disabled"));
    Serial.print(F("  scd (short circuit detect): "));
    Serial.println((ant.flags & UBX_ANT_FLAG_SCD) ? F("enabled") : F("disabled"));
    Serial.print(F("  ocd (open circuit detect): "));
    Serial.println((ant.flags & UBX_ANT_FLAG_OCD) ? F("enabled") : F("disabled"));
    Serial.print(F("  pdwnOnSCD: "));
    Serial.println((ant.flags & UBX_ANT_FLAG_PDWNONSCD) ? F("enabled") : F("disabled"));
    Serial.print(F("  recovery: "));
    Serial.println((ant.flags & UBX_ANT_FLAG_RECOVERY) ? F("enabled") : F("disabled"));

    Serial.print(F("pins: 0x"));
    Serial.println(ant.pins, HEX);
    Serial.print(F("  pinSwitch: "));
    Serial.println(ant.pins & 0x1F);
    Serial.print(F("  pinSCD: "));
    Serial.println((ant.pins >> 5) & 0x1F);
    Serial.print(F("  pinOCD: "));
    Serial.println((ant.pins >> 10) & 0x1F);
  } else {
    Serial.println(F("pollCfgAnt failed"));
  }

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
