/*!
 * @file ubx_mon_ver.ino
 *
 * Hardware test: Poll UBX-MON-VER and print version info.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

#define MAX_EXT 6

UBX_MON_VER_header_t header;
UBX_MON_VER_ext_t extensions[MAX_EXT];

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-MON-VER Hardware Test"));
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

void loop() {
  Serial.println(F("Polling MON-VER..."));

  uint8_t ext_count = ubx.pollMONVER(&header, extensions, MAX_EXT, 2000);

  if (ext_count == 0) {
    Serial.println(F("MON-VER poll failed (timeout or no response)"));
  } else {
    Serial.println(F("--- MON-VER ---"));

    Serial.print(F("SW Version: "));
    char sw_buf[32];
    memcpy(sw_buf, header.swVersion, sizeof(header.swVersion));
    sw_buf[sizeof(header.swVersion)] = '\0';
    Serial.println(sw_buf);

    Serial.print(F("HW Version: "));
    char hw_buf[12];
    memcpy(hw_buf, header.hwVersion, sizeof(header.hwVersion));
    hw_buf[sizeof(header.hwVersion)] = '\0';
    Serial.println(hw_buf);

    Serial.print(F("Extensions: "));
    Serial.println(ext_count);

    for (uint8_t i = 0; i < ext_count; i++) {
      Serial.print(F("  Ext "));
      Serial.print(i + 1);
      Serial.print(F(": "));
      char ext_buf[32];
      memcpy(ext_buf, extensions[i].extension, sizeof(extensions[i].extension));
      ext_buf[sizeof(extensions[i].extension)] = '\0';
      Serial.println(ext_buf);
    }
  }

  Serial.println();
  delay(5000);
}
