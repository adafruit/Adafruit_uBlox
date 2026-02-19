/*!
 * @file ubx_cfg_gnss.ino
 *
 * Hardware test: Get UBX-CFG-GNSS GNSS system configuration.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* getGnssName(uint8_t gnssId) {
  switch (gnssId) {
    case UBX_GNSS_ID_GPS: return "GPS";
    case UBX_GNSS_ID_SBAS: return "SBAS";
    case UBX_GNSS_ID_GALILEO: return "Galileo";
    case UBX_GNSS_ID_BEIDOU: return "BeiDou";
    case UBX_GNSS_ID_IMES: return "IMES";
    case UBX_GNSS_ID_QZSS: return "QZSS";
    case UBX_GNSS_ID_GLONASS: return "GLONASS";
    default: return "Unknown";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-CFG-GNSS Hardware Test"));
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
  Serial.println(F("--- GNSS Configuration ---"));

  UBX_CFG_GNSS_header_t header;
  UBX_CFG_GNSS_block_t blocks[8];
  uint8_t numBlocks = ubx.pollCfgGnss(&header, blocks, 8);

  if (numBlocks > 0) {
    Serial.print(F("Hardware tracking channels: "));
    Serial.println(header.numTrkChHw);
    Serial.print(F("Tracking channels in use: "));
    Serial.println(header.numTrkChUse);
    Serial.print(F("Config blocks: "));
    Serial.println(numBlocks);
    Serial.println();

    for (uint8_t i = 0; i < numBlocks; i++) {
      Serial.print(getGnssName(blocks[i].gnssId));
      Serial.print(F(": "));
      Serial.print((blocks[i].flags & UBX_GNSS_FLAG_ENABLE) ? F("ENABLED") : F("disabled"));
      Serial.print(F(" (res="));
      Serial.print(blocks[i].resTrkCh);
      Serial.print(F(", max="));
      Serial.print(blocks[i].maxTrkCh);
      Serial.print(F(", flags=0x"));
      Serial.print(blocks[i].flags, HEX);
      Serial.println(F(")"));
    }
  } else {
    Serial.println(F("pollCfgGnss failed"));
  }

  Serial.println(F("\n--- Done ---\n"));
  delay(5000);
}
