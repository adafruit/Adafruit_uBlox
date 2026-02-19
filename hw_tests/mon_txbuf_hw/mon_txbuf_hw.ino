/*!
 * @file mon_txbuf_hw.ino
 * @brief Hardware test for MON-TXBUF (Transmitter Buffer Status)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("MON-TXBUF Hardware Test"));
  Serial.println(F("======================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_MON_TXBUF_t txbuf;
  if (ubx.pollMonTxbuf(&txbuf)) {
    Serial.println(F("PASS: MON-TXBUF poll OK"));
    Serial.println(F("  Port   Pending  Use%  Peak%"));
    const char* names[] = {"DDC", "UART1", "UART2", "USB", "SPI", "P5"};
    for (uint8_t i = 0; i < 6; i++) {
      Serial.print(F("  "));
      Serial.print(names[i]);
      Serial.print(F("\t"));
      Serial.print(txbuf.pending[i]);
      Serial.print(F("\t"));
      Serial.print(txbuf.usage[i]);
      Serial.print(F("\t"));
      Serial.println(txbuf.peakUsage[i]);
    }
    Serial.print(F("  Total: use="));
    Serial.print(txbuf.tUsage);
    Serial.print(F("% peak="));
    Serial.print(txbuf.tPeakUsage);
    Serial.print(F("% err=0x"));
    Serial.println(txbuf.errors, HEX);
  } else {
    Serial.println(F("FAIL: MON-TXBUF poll failed"));
  }
}

void loop() {
  delay(2000);
  UBX_MON_TXBUF_t txbuf;
  if (ubx.pollMonTxbuf(&txbuf)) {
    Serial.print(F("DDC pend="));
    Serial.print(txbuf.pending[0]);
    Serial.print(F(" total="));
    Serial.print(txbuf.tUsage);
    Serial.print(F("% err=0x"));
    Serial.println(txbuf.errors, HEX);
  }
}
