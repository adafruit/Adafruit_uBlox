/*!
 * @file mon_gnss_hw.ino
 * @brief Hardware test for MON-GNSS (GNSS System Information)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void printGnss(uint8_t bits) {
  if (bits & UBX_MON_GNSS_GPS) Serial.print(F("GPS "));
  if (bits & UBX_MON_GNSS_GLONASS) Serial.print(F("GLO "));
  if (bits & UBX_MON_GNSS_BEIDOU) Serial.print(F("BDS "));
  if (bits & UBX_MON_GNSS_GALILEO) Serial.print(F("GAL "));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("MON-GNSS Hardware Test"));
  Serial.println(F("======================"));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_MON_GNSS_t gnss;
  if (ubx.pollMonGnss(&gnss)) {
    Serial.println(F("PASS: MON-GNSS poll OK"));
    Serial.print(F("  Supported: "));
    printGnss(gnss.supported);
    Serial.println();
    Serial.print(F("  Enabled: "));
    printGnss(gnss.enabled);
    Serial.println();
    Serial.print(F("  Max simultaneous: "));
    Serial.println(gnss.simultaneous);
  } else {
    Serial.println(F("FAIL: MON-GNSS poll failed"));
  }
}

void loop() {
  delay(5000);
  UBX_MON_GNSS_t gnss;
  if (ubx.pollMonGnss(&gnss)) {
    Serial.print(F("enabled="));
    printGnss(gnss.enabled);
    Serial.print(F("max="));
    Serial.println(gnss.simultaneous);
  }
}
