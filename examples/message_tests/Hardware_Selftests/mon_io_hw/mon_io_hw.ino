/*!
 * @file mon_io_hw.ino
 * @brief Hardware test for MON-IO (I/O System Status)
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

const char* portName(uint8_t p) {
  switch (p) {
    case 0: return "DDC";
    case 1: return "UART1";
    case 2: return "UART2";
    case 3: return "USB";
    case 4: return "SPI";
    default: return "???";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("MON-IO Hardware Test"));
  Serial.println(F("===================="));

  if (!ddc.begin()) {
    Serial.println(F("FAIL: GPS not found"));
    while (1) delay(10);
  }
  ubx.begin();
  delay(500);
  ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);

  UBX_MON_IO_port_t ports[6];
  uint8_t n = ubx.pollMonIo(ports, 6);
  if (n > 0) {
    Serial.print(F("PASS: MON-IO poll OK, "));
    Serial.print(n);
    Serial.println(F(" ports"));
    for (uint8_t i = 0; i < n; i++) {
      Serial.print(F("  "));
      Serial.print(portName(i));
      Serial.print(F(": rx="));
      Serial.print(ports[i].rxBytes);
      Serial.print(F(" tx="));
      Serial.println(ports[i].txBytes);
    }
  } else {
    Serial.println(F("FAIL: MON-IO poll failed"));
  }
}

void loop() {
  delay(3000);
  UBX_MON_IO_port_t ports[6];
  uint8_t n = ubx.pollMonIo(ports, 6);
  if (n > 0) {
    Serial.print(F("DDC rx="));
    Serial.print(ports[0].rxBytes);
    Serial.print(F(" tx="));
    Serial.println(ports[0].txBytes);
  }
}
