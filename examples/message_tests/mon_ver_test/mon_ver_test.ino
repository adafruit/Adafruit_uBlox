/*!
 * @file mon_ver_test.ino
 *
 * Message test: Poll UBX-MON-VER and validate version strings.
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

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.print(name);
  Serial.print(F(": "));
}

bool hasContent(const char* text, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (text[i] != '\0' && text[i] != ' ') {
      return true;
    }
  }
  return false;
}

void printFixedString(const char* label, const char* text, size_t len) {
  char buf[64];
  size_t copy_len = len;
  if (copy_len >= sizeof(buf)) {
    copy_len = sizeof(buf) - 1;
  }
  memcpy(buf, text, copy_len);
  buf[copy_len] = '\0';
  Serial.print(label);
  Serial.println(buf);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-MON-VER Message Test"));
  Serial.println(F("========================"));

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

  delay(500); // Give GPS module time to stabilize after boot

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  Serial.println();
  Serial.println(F("Polling MON-VER..."));

  uint8_t ext_count = ubx.pollMONVER(&header, extensions, MAX_EXT, 2000);
  bool got_version = ext_count > 0;
  printTestResult(F("got_version"), got_version);
  Serial.println(ext_count);

  bool sw_version_valid =
      hasContent(header.swVersion, sizeof(header.swVersion));
  printTestResult(F("sw_version_valid"), sw_version_valid);
  Serial.println(sw_version_valid ? F("OK") : F("BAD"));

  bool hw_version_valid =
      hasContent(header.hwVersion, sizeof(header.hwVersion));
  printTestResult(F("hw_version_valid"), hw_version_valid);
  Serial.println(hw_version_valid ? F("OK") : F("BAD"));

  bool has_extensions = ext_count > 0;
  printTestResult(F("has_extensions"), has_extensions);
  Serial.println(has_extensions ? F("OK") : F("BAD"));

  Serial.println();
  Serial.println(F("--- MON-VER ---"));
  printFixedString("SW Version: ", header.swVersion, sizeof(header.swVersion));
  printFixedString("HW Version: ", header.hwVersion, sizeof(header.hwVersion));

  for (uint8_t i = 0; i < ext_count; i++) {
    Serial.print(F("Ext "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    char ext_buf[32];
    memcpy(ext_buf, extensions[i].extension, sizeof(extensions[i].extension));
    ext_buf[sizeof(extensions[i].extension)] = '\0';
    Serial.println(ext_buf);
  }
}

void loop() {
  delay(1000);
}
