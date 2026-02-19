/*!
 * @file mon_msgpp_test.ino
 *
 * Message test: Poll UBX-MON-MSGPP (Message Parse/Process Status)
 * Displays message counts per protocol per port.
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
    case 0: return "DDC";
    case 1: return "UART1";
    case 2: return "UART2";
    case 3: return "USB";
    case 4: return "SPI";
    case 5: return "Port5";
    default: return "???";
  }
}

void printMsgppDetails(UBX_MON_MSGPP_t* msgpp) {
  Serial.println(F("\nMON-MSGPP Details:"));

  Serial.println(F("  Message counts per port/protocol:"));
  Serial.println(F("  Port      UBX   NMEA RTCM2  --    --  RTCM3   --    --"));

  for (uint8_t p = 0; p < 6; p++) {
    Serial.print(F("  "));
    Serial.print(getPortName(p));
    if (strlen(getPortName(p)) < 5) Serial.print(F("\t"));

    bool hasData = false;
    for (uint8_t pr = 0; pr < 8; pr++) {
      Serial.print(F(" "));
      if (msgpp->msg[p][pr] < 10000) Serial.print(F(" "));
      if (msgpp->msg[p][pr] < 1000) Serial.print(F(" "));
      if (msgpp->msg[p][pr] < 100) Serial.print(F(" "));
      if (msgpp->msg[p][pr] < 10) Serial.print(F(" "));
      Serial.print(msgpp->msg[p][pr]);
      if (msgpp->msg[p][pr] > 0) hasData = true;
    }
    if (hasData) Serial.print(F(" *"));
    Serial.println();
  }

  Serial.println(F("\n  Skipped bytes per port:"));
  for (uint8_t p = 0; p < 6; p++) {
    if (msgpp->skipped[p] > 0) {
      Serial.print(F("    "));
      Serial.print(getPortName(p));
      Serial.print(F(": "));
      Serial.println(msgpp->skipped[p]);
    }
  }
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running MON-MSGPP tests..."));

  // Test 1: Check struct size
  bool size_ok = (sizeof(UBX_MON_MSGPP_t) == 120);
  printTestResult(F("struct_size_120"), size_ok);
  if (size_ok) passed++;

  // Test 2: Poll MON-MSGPP
  UBX_MON_MSGPP_t msgpp;
  bool poll_ok = ubx.pollMonMsgpp(&msgpp);
  printTestResult(F("poll_mon_msgpp"), poll_ok);
  if (poll_ok) {
    passed++;
    printMsgppDetails(&msgpp);
  }

  // Test 3: Verify UBX messages received on DDC (port 0, protocol 0)
  bool ubx_on_ddc = poll_ok && (msgpp.msg[0][0] > 0);
  printTestResult(F("ubx_on_ddc"), ubx_on_ddc);
  if (ubx_on_ddc) passed++;

  // Test 4: Verify at least some messages were processed
  uint32_t totalMsgs = 0;
  if (poll_ok) {
    for (uint8_t p = 0; p < 6; p++) {
      for (uint8_t pr = 0; pr < 8; pr++) {
        totalMsgs += msgpp.msg[p][pr];
      }
    }
  }
  bool msgs_processed = (totalMsgs > 0);
  printTestResult(F("msgs_processed"), msgs_processed);
  if (msgs_processed) passed++;

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

  Serial.println(F("UBX-MON-MSGPP Message Test"));
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
  Serial.println(F("Streaming message stats..."));
}

void loop() {
  delay(5000);

  UBX_MON_MSGPP_t msgpp;
  if (ubx.pollMonMsgpp(&msgpp)) {
    Serial.print(F("MON-MSGPP: DDC UBX="));
    Serial.print(msgpp.msg[0][0]);
    Serial.print(F(", NMEA="));
    Serial.print(msgpp.msg[0][1]);
    Serial.print(F(", skipped="));
    Serial.println(msgpp.skipped[0]);
  } else {
    Serial.println(F("MON-MSGPP poll timeout"));
  }
}
