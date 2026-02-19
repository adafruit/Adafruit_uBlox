/*!
 * @file ubx_cfg_rate.ino
 *
 * Hardware test: Get and set UBX-CFG-RATE measurement rate.
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

  Serial.println(F("UBX-CFG-RATE Hardware Test"));
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

void printRate() {
  UBX_CFG_RATE_t rate;
  if (ubx.getRate(&rate)) {
    Serial.print(F("measRate: "));
    Serial.print(rate.measRate);
    Serial.println(F(" ms"));
    Serial.print(F("navRate: "));
    Serial.println(rate.navRate);
    Serial.print(F("timeRef: "));
    Serial.println(rate.timeRef);
  } else {
    Serial.println(F("getRate failed"));
  }
}

void loop() {
  Serial.println(F("--- Current Rate ---"));
  printRate();

  Serial.println(F("\nSetting rate to 5 Hz (200 ms)..."));
  if (ubx.setRate(200)) {
    Serial.println(F("setRate(200) OK"));
  } else {
    Serial.println(F("setRate(200) FAILED"));
  }
  delay(100);
  printRate();

  Serial.println(F("\nSetting rate to 10 Hz (100 ms)..."));
  if (ubx.setRate(100)) {
    Serial.println(F("setRate(100) OK"));
  } else {
    Serial.println(F("setRate(100) FAILED"));
  }
  delay(100);
  printRate();

  Serial.println(F("\nRestoring rate to 1 Hz (1000 ms)..."));
  if (ubx.setRate(1000)) {
    Serial.println(F("setRate(1000) OK"));
  } else {
    Serial.println(F("setRate(1000) FAILED"));
  }
  delay(100);
  printRate();

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
