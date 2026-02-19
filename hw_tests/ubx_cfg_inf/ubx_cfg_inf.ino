/*!
 * @file ubx_cfg_inf.ino
 *
 * Hardware test: Get UBX-CFG-INF info message configuration.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* getPortName(uint8_t port) {
  switch (port) {
    case 0: return "DDC";
    case 1: return "UART1";
    case 2: return "UART2";
    case 3: return "USB";
    case 4: return "SPI";
    default: return "?";
  }
}

void printInfMask(uint8_t mask) {
  if (mask & UBX_INF_MSG_ERROR) Serial.print(F("ERR "));
  if (mask & UBX_INF_MSG_WARNING) Serial.print(F("WARN "));
  if (mask & UBX_INF_MSG_NOTICE) Serial.print(F("NOTICE "));
  if (mask & UBX_INF_MSG_TEST) Serial.print(F("TEST "));
  if (mask & UBX_INF_MSG_DEBUG) Serial.print(F("DEBUG "));
  if (mask == 0) Serial.print(F("(none)"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-CFG-INF Hardware Test"));
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
  Serial.println(F("--- UBX Protocol Info Messages ---"));
  UBX_CFG_INF_block_t ubx_inf[2];
  uint8_t num = ubx.pollCfgInf(UBX_INF_PROTOCOL_UBX, ubx_inf, 2);
  if (num > 0) {
    for (uint8_t port = 0; port < 6; port++) {
      Serial.print(F("  "));
      Serial.print(getPortName(port));
      Serial.print(F(": "));
      printInfMask(ubx_inf[0].infMsgMask[port]);
      Serial.println();
    }
  } else {
    Serial.println(F("  poll failed"));
  }

  Serial.println(F("\n--- NMEA Protocol Info Messages ---"));
  UBX_CFG_INF_block_t nmea_inf[2];
  num = ubx.pollCfgInf(UBX_INF_PROTOCOL_NMEA, nmea_inf, 2);
  if (num > 0) {
    for (uint8_t port = 0; port < 6; port++) {
      Serial.print(F("  "));
      Serial.print(getPortName(port));
      Serial.print(F(": "));
      printInfMask(nmea_inf[0].infMsgMask[port]);
      Serial.println();
    }
  } else {
    Serial.println(F("  poll failed"));
  }

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
