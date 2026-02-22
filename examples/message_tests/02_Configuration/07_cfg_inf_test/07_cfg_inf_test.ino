/*!
 * @file cfg_inf_test.ino
 *
 * Message test: Poll UBX-CFG-INF, display info message configuration.
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

bool tests_run = false;

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

void printInfMask(uint8_t mask) {
  if (mask & UBX_INF_MSG_ERROR)
    Serial.print(F("ERR "));
  if (mask & UBX_INF_MSG_WARNING)
    Serial.print(F("WARN "));
  if (mask & UBX_INF_MSG_NOTICE)
    Serial.print(F("NOTE "));
  if (mask & UBX_INF_MSG_TEST)
    Serial.print(F("TEST "));
  if (mask & UBX_INF_MSG_DEBUG)
    Serial.print(F("DBG "));
  if (mask == 0)
    Serial.print(F("(none)"));
}

void printInfBlock(UBX_CFG_INF_block_t* block) {
  Serial.print(F("  Protocol: "));
  Serial.print(block->protocolID == UBX_INF_PROTOCOL_UBX ? F("UBX") : F("NMEA"));
  Serial.println();

  const char* portNames[] = {"DDC", "UART1", "UART2", "USB", "SPI", "Rsvd"};
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(F("    "));
    Serial.print(portNames[i]);
    Serial.print(F(": "));
    printInfMask(block->infMsgMask[i]);
    Serial.println();
  }
}

void runTests() {
  uint8_t passed = 0;
  const uint8_t total = 4;

  Serial.println();
  Serial.println(F("Running CFG-INF tests..."));

  // Test 1: Poll UBX protocol INF config
  UBX_CFG_INF_block_t blocks[2];
  uint8_t numBlocks = ubx.pollCfgInf(UBX_INF_PROTOCOL_UBX, blocks, 2);
  bool poll_ubx_ok = (numBlocks > 0);
  printTestResult(F("poll_ubx_inf"), poll_ubx_ok);
  Serial.print(numBlocks);
  Serial.println(F(" block(s)"));
  if (poll_ubx_ok) {
    passed++;
    for (uint8_t i = 0; i < numBlocks; i++) {
      printInfBlock(&blocks[i]);
    }
  }

  // Save original UBX settings
  UBX_CFG_INF_block_t original;
  if (numBlocks > 0) {
    memcpy(&original, &blocks[0], sizeof(original));
  }

  // Test 2: Poll NMEA protocol INF config
  numBlocks = ubx.pollCfgInf(UBX_INF_PROTOCOL_NMEA, blocks, 2);
  bool poll_nmea_ok = (numBlocks > 0);
  printTestResult(F("poll_nmea_inf"), poll_nmea_ok);
  Serial.print(numBlocks);
  Serial.println(F(" block(s)"));
  if (poll_nmea_ok) {
    passed++;
    for (uint8_t i = 0; i < numBlocks; i++) {
      printInfBlock(&blocks[i]);
    }
  }

  // Test 3: Enable ERROR messages on DDC for UBX
  UBX_CFG_INF_block_t newBlock;
  memcpy(&newBlock, &original, sizeof(newBlock));
  newBlock.protocolID = UBX_INF_PROTOCOL_UBX;
  newBlock.infMsgMask[0] |= UBX_INF_MSG_ERROR; // DDC port
  bool set_ok = ubx.setCfgInf(&newBlock, 1);
  delay(100);

  // Verify
  numBlocks = ubx.pollCfgInf(UBX_INF_PROTOCOL_UBX, blocks, 2);
  bool verify_error = (numBlocks > 0) &&
                      (blocks[0].infMsgMask[0] & UBX_INF_MSG_ERROR);
  printTestResult(F("enable_error"), set_ok && verify_error);
  Serial.println(verify_error ? F("enabled") : F("not enabled"));
  if (set_ok && verify_error)
    passed++;

  // Test 4: Restore original settings
  bool restore = ubx.setCfgInf(&original, 1);
  delay(100);
  numBlocks = ubx.pollCfgInf(UBX_INF_PROTOCOL_UBX, blocks, 2);
  bool verify_restore = (numBlocks > 0) &&
                        (blocks[0].infMsgMask[0] == original.infMsgMask[0]);
  printTestResult(F("restore"), restore && verify_restore);
  Serial.println(verify_restore ? F("OK") : F("mismatch"));
  if (restore && verify_restore)
    passed++;

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

  Serial.println(F("UBX-CFG-INF Message Test"));
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

  delay(500);

  UBXSendStatus status = ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
  if (status != UBX_SEND_SUCCESS) {
    Serial.print(F("WARNING: setUBXOnly status: "));
    Serial.println(status);
  } else {
    Serial.println(F("UBX-only mode set on DDC port"));
  }

  runTests();
  tests_run = true;
}

void loop() {
  delay(1000);
}
