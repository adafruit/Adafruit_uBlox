/*!
 * @file mon_hw_hw.ino
 * @brief Hardware test for MON-HW (Hardware Status)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* antStatus(uint8_t s) {
  switch (s) {
    case 0: return "INIT";
    case 1: return "UNKNOWN";
    case 2: return "OK";
    case 3: return "SHORT";
    case 4: return "OPEN";
    default: return "???";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("MON-HW Hardware Test"));
  Serial.println(F("===================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_MON_HW_t hw;
  if (ubx.pollMonHw(&hw)) {
    Serial.println(F("PASS: MON-HW poll OK"));
    Serial.print(F("  Antenna: "));
    Serial.println(antStatus(hw.aStatus));
    Serial.print(F("  Noise: "));
    Serial.println(hw.noisePerMS);
    Serial.print(F("  AGC: "));
    Serial.print((hw.agcCnt * 100) / 8191);
    Serial.println(F("%"));
    Serial.print(F("  Jamming: "));
    Serial.println(hw.jamInd);
  } else {
    Serial.println(F("FAIL: MON-HW poll failed"));
  }
}

void loop() {
  delay(2000);
  UBX_MON_HW_t hw;
  if (ubx.pollMonHw(&hw)) {
    Serial.print(F("ant="));
    Serial.print(antStatus(hw.aStatus));
    Serial.print(F(" noise="));
    Serial.print(hw.noisePerMS);
    Serial.print(F(" AGC="));
    Serial.print((hw.agcCnt * 100) / 8191);
    Serial.print(F("% jam="));
    Serial.println(hw.jamInd);
  }
}
