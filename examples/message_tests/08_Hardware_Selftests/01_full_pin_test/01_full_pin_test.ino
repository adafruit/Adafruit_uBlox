/*!
 * @file full_pin_test.ino
 *
 * Comprehensive hardware pin test for SAM-M8Q GPS module.
 * Tests I2C, UART, Reset, PPS, and EXTINT pins.
 *
 * Pin connections:
 *   A0 -> RESET (active low)
 *   A1 -> PPS   (1Hz pulse with fix)
 *   A2 -> EXTINT (interrupt/wakeup)
 *   Serial1 TX/RX -> SAM-M8Q UART RX/TX
 *   Wire (I2C) -> DDC (0x42)
 *
 * Written by Limor 'ladyada' Fried with assistance from Claude Code
 */

#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

#define PIN_RESET A0
#define PIN_PPS A1
#define PIN_EXTINT A2

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);

int passCount = 0;
int failCount = 0;
int skipCount = 0;

void printResult(const __FlashStringHelper* name, const char* result) {
  Serial.print(F("["));
  Serial.print(result);
  Serial.print(F("] "));
  Serial.println(name);
  if (strcmp(result, "PASS") == 0)
    passCount++;
  else if (strcmp(result, "FAIL") == 0)
    failCount++;
  else
    skipCount++;
}

// Send raw UBX poll and check for UBX response header on Serial1
bool uartProbe(uint32_t baud) {
  Serial1.end();
  Serial1.begin(baud);
  delay(100);

  // Flush any pending data
  while (Serial1.available())
    Serial1.read();

  // Send UBX NAV-PVT poll: B5 62 01 07 00 00 08 19
  uint8_t poll[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
  Serial1.write(poll, sizeof(poll));
  Serial1.flush();

  // Wait up to 2 seconds for response
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (Serial1.available() >= 2) {
      uint8_t b1 = Serial1.read();
      if (b1 == 0xB5) {
        if (Serial1.available()) {
          uint8_t b2 = Serial1.read();
          if (b2 == 0x62) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

// Read a full UBX message from Serial1, return payload length or -1
int uartReadUBX(uint8_t* buf, int maxLen, uint32_t timeoutMs) {
  unsigned long start = millis();
  int state = 0; // 0=sync1, 1=sync2, 2=class, 3=id, 4=lenLo, 5=lenHi,
                 // 6=payload, 7=ckA, 8=ckB
  uint8_t cls = 0, id = 0;
  uint16_t payLen = 0;
  int payIdx = 0;

  while (millis() - start < timeoutMs) {
    if (!Serial1.available())
      continue;
    uint8_t b = Serial1.read();
    switch (state) {
      case 0:
        if (b == 0xB5)
          state = 1;
        break;
      case 1:
        state = (b == 0x62) ? 2 : 0;
        break;
      case 2:
        cls = b;
        state = 3;
        break;
      case 3:
        id = b;
        state = 4;
        break;
      case 4:
        payLen = b;
        state = 5;
        break;
      case 5:
        payLen |= (uint16_t)b << 8;
        payIdx = 0;
        state = (payLen > 0) ? 6 : 7;
        break;
      case 6:
        if (payIdx < maxLen)
          buf[payIdx] = b;
        payIdx++;
        if (payIdx >= payLen)
          state = 7;
        break;
      case 7:
        state = 8;
        break; // ckA
      case 8:
        // Done - check if this is NAV-PVT (0x01 0x07)
        if (cls == 0x01 && id == 0x07 && payLen >= 84) {
          return payLen;
        }
        // Not what we wanted, keep looking
        state = 0;
        break;
    }
  }
  return -1;
}

void setup() {
  // Drive RESET high IMMEDIATELY — before Serial, before anything else.
  // RP2040 pins float at boot, which can hold the module in reset.
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, HIGH);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("  SAM-M8Q Full Pin Test"));
  Serial.println(F("========================================"));
  Serial.println(F(""));

  // Configure other pins
  pinMode(PIN_PPS, INPUT);
  pinMode(PIN_EXTINT, INPUT);

  // Give module time to boot after reset release
  Serial.println(F("Waiting 2s for module startup..."));
  delay(2000);

  // ==========================================
  // TEST 1: I2C Communication
  // ==========================================
  Serial.println(F("--- Test 1: I2C Communication ---"));

  bool i2cOk = false;
  if (!ddc.begin()) {
    Serial.println(F("  Could not connect to GPS on I2C"));
    printResult(F("I2C connection"), "FAIL");
  } else {
    Serial.println(F("  GPS module found on I2C 0x42"));
    if (!ubx.begin()) {
      Serial.println(F("  UBX parser init failed"));
      printResult(F("I2C communication"), "FAIL");
    } else {
      // Switch to UBX-only on DDC
      ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
      delay(100);

      UBX_NAV_PVT_t pvt;
      if (ubx.poll(UBX_CLASS_NAV, (uint8_t)UBXNavMessageId::UBX_NAV_PVT,
                   (uint8_t*)&pvt, sizeof(pvt), 3000)) {
        Serial.print(F("  iTOW: "));
        Serial.print(pvt.iTOW);
        Serial.print(F(" ms, fixType: "));
        Serial.println(pvt.fixType);
        i2cOk = true;
        printResult(F("I2C communication"), "PASS");
      } else {
        Serial.println(F("  NAV-PVT poll failed"));
        printResult(F("I2C communication"), "FAIL");
      }
    }
  }
  Serial.println(F(""));

  // ==========================================
  // TEST 2: Hardware Reset
  // ==========================================
  Serial.println(F("--- Test 2: Hardware Reset ---"));

  if (!i2cOk) {
    Serial.println(F("  Skipping (I2C not working)"));
    printResult(F("hardware reset"), "SKIP");
  } else {
    Serial.println(F("  Pulling RESET low for 100ms..."));
    digitalWrite(PIN_RESET, LOW);
    delay(100);
    digitalWrite(PIN_RESET, HIGH);
    Serial.println(F("  Released RESET, waiting 1.5s for reboot..."));
    delay(1500);

    // Re-initialize
    ddc.begin();
    ubx.begin();
    ubx.setUBXOnly(UBX_PORT_DDC, true, 1000);
    delay(100);

    UBX_NAV_PVT_t pvt;
    if (ubx.poll(UBX_CLASS_NAV, (uint8_t)UBXNavMessageId::UBX_NAV_PVT,
                 (uint8_t*)&pvt, sizeof(pvt), 3000)) {
      Serial.print(F("  Module responded after reset, iTOW: "));
      Serial.println(pvt.iTOW);
      printResult(F("hardware reset"), "PASS");
    } else {
      Serial.println(F("  Module did not respond after reset"));
      printResult(F("hardware reset"), "FAIL");
    }
  }
  Serial.println(F(""));

  // ==========================================
  // TEST 3: UART Auto-Baud Detection
  // ==========================================
  Serial.println(F("--- Test 3: UART Auto-Baud Detection ---"));

  uint32_t bauds[] = {9600, 57600, 115200};
  uint32_t detectedBaud = 0;

  for (int i = 0; i < 3; i++) {
    Serial.print(F("  Trying "));
    Serial.print(bauds[i]);
    Serial.print(F(" baud... "));
    if (uartProbe(bauds[i])) {
      Serial.println(F("RESPONSE"));
      detectedBaud = bauds[i];
      break;
    } else {
      Serial.println(F("no response"));
    }
  }

  if (detectedBaud > 0) {
    Serial.print(F("  Detected baud rate: "));
    Serial.println(detectedBaud);
    printResult(F("UART auto-baud"), "PASS");
  } else {
    Serial.println(F("  No response at any baud rate"));
    printResult(F("UART auto-baud"), "FAIL");
  }
  Serial.println(F(""));

  // ==========================================
  // TEST 4: UART Communication
  // ==========================================
  Serial.println(F("--- Test 4: UART Communication ---"));

  if (detectedBaud == 0) {
    Serial.println(F("  Skipping (no UART baud detected)"));
    printResult(F("UART communication"), "SKIP");
  } else {
    Serial1.begin(detectedBaud);
    delay(100);
    while (Serial1.available())
      Serial1.read();

    // Send NAV-PVT poll
    uint8_t poll[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
    Serial1.write(poll, sizeof(poll));
    Serial1.flush();

    uint8_t buf[92];
    int len = uartReadUBX(buf, sizeof(buf), 3000);
    if (len >= 84) {
      // Parse iTOW and fixType from raw payload
      uint32_t iTOW = buf[0] | ((uint32_t)buf[1] << 8) |
                      ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
      uint8_t fixType = buf[20];
      Serial.print(F("  UART NAV-PVT: iTOW="));
      Serial.print(iTOW);
      Serial.print(F(" ms, fixType="));
      Serial.println(fixType);
      printResult(F("UART communication"), "PASS");
    } else {
      Serial.println(F("  Failed to read NAV-PVT over UART"));
      printResult(F("UART communication"), "FAIL");
    }
  }
  Serial.println(F(""));

  // ==========================================
  // TEST 5: PPS Signal Detection
  // ==========================================
  Serial.println(F("--- Test 5: PPS Signal ---"));

  // Check for fix first
  UBX_NAV_PVT_t pvt2;
  bool hasFix = false;
  if (i2cOk) {
    // Re-init I2C after UART tests may have changed module state
    if (ubx.poll(UBX_CLASS_NAV, (uint8_t)UBXNavMessageId::UBX_NAV_PVT,
                 (uint8_t*)&pvt2, sizeof(pvt2), 3000)) {
      hasFix = (pvt2.fixType >= 2);
      Serial.print(F("  Fix type: "));
      Serial.print(pvt2.fixType);
      Serial.print(F(", Satellites: "));
      Serial.println(pvt2.numSV);
    }
  }

  if (!hasFix) {
    Serial.println(F("  No fix — PPS requires fix to pulse"));
    printResult(F("PPS signal"), "SKIP");
  } else {
    Serial.println(F("  Counting PPS edges over 3 seconds..."));
    int edgeCount = 0;
    bool lastState = digitalRead(PIN_PPS);
    unsigned long firstEdge = 0;
    unsigned long lastEdge = 0;
    unsigned long start = millis();

    while (millis() - start < 3500) {
      bool state = digitalRead(PIN_PPS);
      if (state && !lastState) { // rising edge
        edgeCount++;
        if (edgeCount == 1)
          firstEdge = millis();
        lastEdge = millis();
      }
      lastState = state;
    }

    Serial.print(F("  Rising edges detected: "));
    Serial.println(edgeCount);

    if (edgeCount >= 2) {
      unsigned long period = (lastEdge - firstEdge) / (edgeCount - 1);
      Serial.print(F("  Average period: "));
      Serial.print(period);
      Serial.println(F(" ms (expect ~1000 ms)"));
    }

    if (edgeCount >= 2 && edgeCount <= 5) {
      printResult(F("PPS signal"), "PASS");
    } else if (edgeCount == 1) {
      Serial.println(F("  Only 1 edge — may need longer measurement"));
      printResult(F("PPS signal"), "FAIL");
    } else if (edgeCount == 0) {
      Serial.println(F("  No edges detected — check PPS wiring"));
      printResult(F("PPS signal"), "FAIL");
    } else {
      Serial.print(F("  Unexpected edge count: "));
      Serial.println(edgeCount);
      printResult(F("PPS signal"), "FAIL");
    }
  }
  Serial.println(F(""));

  // ==========================================
  // TEST 6: EXTINT Pin
  // ==========================================
  Serial.println(F("--- Test 6: EXTINT Pin ---"));
  bool extState = digitalRead(PIN_EXTINT);
  Serial.print(F("  EXTINT idle state: "));
  Serial.println(extState ? F("HIGH") : F("LOW"));
  Serial.println(F("  (INFO only — EXTINT used for wakeup from power save)"));
  printResult(F("EXTINT readable"), "PASS");
  Serial.println(F(""));

  // ==========================================
  // SUMMARY
  // ==========================================
  Serial.println(F("========================================"));
  Serial.println(F("  SUMMARY"));
  Serial.println(F("========================================"));
  Serial.print(F("  PASS: "));
  Serial.println(passCount);
  Serial.print(F("  FAIL: "));
  Serial.println(failCount);
  Serial.print(F("  SKIP: "));
  Serial.println(skipCount);
  Serial.print(F("  TOTAL: "));
  Serial.println(passCount + failCount + skipCount);
  Serial.println(F("========================================"));
}

void loop() {
  // Nothing — tests run once in setup
  delay(10000);
}
