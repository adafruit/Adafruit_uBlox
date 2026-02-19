/*!
 * @file ubx_cfg_rst.ino
 *
 * Hardware test: Demo hot/warm/cold start via UBX-CFG-RST.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

bool initModule() {
  if (!ddc.begin()) {
    return false;
  }
  if (!ubx.begin()) {
    return false;
  }
  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  }
  return true;
}

bool reconnectModule(uint32_t timeout_ms) {
  uint32_t start_ms = millis();
  while (millis() - start_ms < timeout_ms) {
    if (initModule()) {
      return true;
    }
    delay(200);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-CFG-RST Hardware Test"));
  Serial.println(F("========================="));

  if (!initModule()) {
    Serial.println(F("FAIL: Could not connect to GPS module!"));
    while (1)
      delay(10);
  }
  Serial.println(F("GPS module connected on I2C"));
  Serial.println();
}

void loop() {
  Serial.println(F("--- Performing Hot Start ---"));
  Serial.println(F("(Preserves ephemeris, fast re-acquisition)"));
  if (ubx.hotStart()) {
    Serial.println(F("hotStart() sent OK"));
  } else {
    Serial.println(F("hotStart() FAILED"));
  }

  Serial.println(F("Waiting 3 seconds for reset..."));
  delay(3000);

  Serial.println(F("Reconnecting..."));
  if (reconnectModule(10000)) {
    Serial.println(F("Reconnected OK"));
  } else {
    Serial.println(F("Reconnect FAILED"));
  }

  Serial.println();
  delay(5000);

  Serial.println(F("--- Performing Warm Start ---"));
  Serial.println(F("(Clears ephemeris bit 0, keeps most data)"));
  if (ubx.warmStart()) {
    Serial.println(F("warmStart() sent OK"));
  } else {
    Serial.println(F("warmStart() FAILED"));
  }

  Serial.println(F("Waiting 3 seconds for reset..."));
  delay(3000);

  Serial.println(F("Reconnecting..."));
  if (reconnectModule(10000)) {
    Serial.println(F("Reconnected OK"));
  } else {
    Serial.println(F("Reconnect FAILED"));
  }

  Serial.println();
  Serial.println(F("--- Done (waiting 30 sec before next cycle) ---\n"));
  delay(30000);
}
