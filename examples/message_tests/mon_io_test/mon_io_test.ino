/*!
 * @file mon_io_test.ino
 *
 * Message test: Poll UBX-MON-IO (I/O System Status)
 * Displays RX/TX bytes and error counts per port.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

void printTestResult(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("  ["));
  if (pass) {
    Serial.print(F("PASS"));
  } else {
    Serial.print(F("FAIL"));
  }
  Serial.print(F("] "));
  Serial.println(name);
}

const char* getPortName(uint8_t port) {
  switch (port) {
    case 0: return "DDC/I2C";
    case 1: return "UART1";
    case 2: return "UART2";
    case 3: return "USB";
    case 4: return "SPI";
    default: return "???";
  }
}

void printIoDetails(UBX_MON_IO_port_t* ports, uint8_t numPorts) {
  Serial.println(F("\nMON-IO Details:"));
  Serial.print(F("  Number of ports: "));
  Serial.println(numPorts);

  for (uint8_t i = 0; i < numPorts; i++) {
    Serial.print(F("\n  Port "));
    Serial.print(i);
    Serial.print(F(" ("));
    Serial.print(getPortName(i));
    Serial.println(F("):"));

    Serial.print(F("    RX bytes: "));
    Serial.println(ports[i].rxBytes);
    Serial.print(F("    TX bytes: "));
    Serial.println(ports[i].txBytes);
    Serial.print(F("    Parity errors: "));
    Serial.println(ports[i].parityErrs);
    Serial.print(F("    Framing errors: "));
    Serial.println(ports[i].framingErrs);
    Serial.print(F("    Overrun errors: "));
    Serial.println(ports[i].overrunErrs);
    Serial.print(F("    Break conditions: "));
    Serial.println(ports[i].breakCond);
  }
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running MON-IO tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_MON_IO_port_t) == 20);
  printTestResult(F("struct_size_20"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll MON-IO
  UBX_MON_IO_port_t ports[6];
  uint8_t numPorts = ubx.pollMonIo(ports, 6);
  bool poll_ok = (numPorts > 0);
  printTestResult(F("poll_mon_io"), poll_ok);
  if (poll_ok) {
    passed++;
    printIoDetails(ports, numPorts);
  }

  // Test 3: Verify we got at least DDC port (port 0)
  bool ddc_present = (numPorts >= 1);
  printTestResult(F("ddc_port_present"), ddc_present);
  if (ddc_present) passed++;

  // Test 4: Verify DDC has received some bytes (we've been talking to it)
  bool ddc_active = ddc_present && (ports[0].rxBytes > 0);
  printTestResult(F("ddc_rx_active"), ddc_active);
  if (ddc_active) passed++;

  Serial.println();
  Serial.print(F("Results: "));
  Serial.print(passed);
  Serial.print(F("/"));
  Serial.print(total);
  Serial.println(F(" tests passed"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("UBX-MON-IO Message Test"));
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

  delay(500);

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  runTests();
  Serial.println();
  Serial.println(F("Streaming I/O stats..."));
}

void loop() {
  delay(3000);

  UBX_MON_IO_port_t ports[6];
  uint8_t numPorts = ubx.pollMonIo(ports, 6);
  if (numPorts > 0) {
    Serial.print(F("MON-IO: "));
    for (uint8_t i = 0; i < numPorts; i++) {
      if (ports[i].rxBytes > 0 || ports[i].txBytes > 0) {
        Serial.print(getPortName(i));
        Serial.print(F("(rx="));
        Serial.print(ports[i].rxBytes);
        Serial.print(F(",tx="));
        Serial.print(ports[i].txBytes);
        Serial.print(F(") "));
      }
    }
    Serial.println();
  } else {
    Serial.println(F("MON-IO poll timeout"));
  }
}
