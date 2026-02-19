/*!
 * @file mon_rxbuf_hw.ino
 * @brief Hardware test for MON-RXBUF (Receiver Buffer Status)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("MON-RXBUF Hardware Test"));
  Serial.println(F("======================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_MON_RXBUF_t rxbuf;
  if (ubx.pollMonRxbuf(&rxbuf)) {
    Serial.println(F("PASS: MON-RXBUF poll OK"));
    Serial.println(F("  Port   Pending  Use%  Peak%"));
    const char* names[] = {"DDC", "UART1", "UART2", "USB", "SPI", "P5"};
    for (uint8_t i = 0; i < 6; i++) {
      Serial.print(F("  "));
      Serial.print(names[i]);
      Serial.print(F("\t"));
      Serial.print(rxbuf.pending[i]);
      Serial.print(F("\t"));
      Serial.print(rxbuf.usage[i]);
      Serial.print(F("\t"));
      Serial.println(rxbuf.peakUsage[i]);
    }
  } else {
    Serial.println(F("FAIL: MON-RXBUF poll failed"));
  }
}

void loop() {
  delay(2000);
  UBX_MON_RXBUF_t rxbuf;
  if (ubx.pollMonRxbuf(&rxbuf)) {
    Serial.print(F("DDC pend="));
    Serial.print(rxbuf.pending[0]);
    Serial.print(F(" use="));
    Serial.print(rxbuf.usage[0]);
    Serial.print(F("% peak="));
    Serial.print(rxbuf.peakUsage[0]);
    Serial.println(F("%"));
  }
}
