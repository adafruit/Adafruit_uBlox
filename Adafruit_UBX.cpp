/*!
 * @file Adafruit_UBX.cpp
 *
 * @mainpage Arduino library for UBX protocol from u-blox GPS/RTK modules
 *
 * @section intro_sec Introduction
 *
 * This is a library for parsing UBX protocol messages from u-blox GPS/RTK
 * modules. It works with any Stream-based interface including UART and DDC
 * (I2C).
 *
 * Designed specifically to work with u-blox GPS/RTK modules
 * like NEO-M8P, ZED-F9P, etc.
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_UBX.h"

/*!
 *  @brief  Constructor
 *  @param  stream Reference to a Stream object (Serial, Adafruit_UBloxDDC,
 * etc.)
 */
Adafruit_UBX::Adafruit_UBX(Stream& stream) {
  _stream = &stream;
  onUBXMessage = NULL;
}

/*!
 *  @brief  Destructor
 */
Adafruit_UBX::~Adafruit_UBX() {
  if (onUBXMessage)
    onUBXMessage = NULL;
}

/*!
 *  @brief  Initializes the UBX parser
 *  @return Always returns true (initialization is trivial)
 */
bool Adafruit_UBX::begin() {
  resetParser();
  return true;
}

void Adafruit_UBX::updateChecksum(uint8_t incomingByte) {
  _checksumA += incomingByte;
  _checksumB += _checksumA;
}

/*!\
 *  @brief  Poll a UBX message and wait for response
 *  @param  msgClass Message class to poll
 *  @param  msgId Message ID to poll
 *  @param  response Pointer to response payload buffer
 *  @param  responseSize Size of response buffer in bytes
 *  @param  timeout_ms Maximum time to wait for response in milliseconds
 *  @return True if response received, false on timeout or send failure
 */
bool Adafruit_UBX::poll(uint8_t msgClass, uint8_t msgId, void* response,
                        uint16_t responseSize, uint16_t timeout_ms) {
  if (!sendMessage(msgClass, msgId, NULL, 0)) {
    return false;
  }

  uint32_t startTime = millis();
  while ((millis() - startTime) < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == msgClass && _lastMsgId == msgId) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        uint16_t copyLen = min(responseSize, usablePayload);
        memcpy(response, _buffer + 6, copyLen);
        return true;
      }
    }
    delay(1);
  }

  return false;
}

/*! * @brief Poll NAV-SAT and fill as many satellites as will fit
 * @param header Pointer to header struct to fill
 * @param svArray Array of sv structs to fill
 * @param maxSvs Maximum number of satellites the array can hold
 * @param timeout_ms Timeout in milliseconds
 * @return Number of satellites actually read (may be less than numSvs in header
 * if buffer too small), or 0 on failure
 */
uint8_t Adafruit_UBX::pollNAVSAT(UBX_NAV_SAT_header_t* header,
                                 UBX_NAV_SAT_sv_t* svArray, uint8_t maxSvs,
                                 uint16_t timeout_ms) {
  // Send poll request
  if (!sendMessage(UBX_CLASS_NAV, UBX_NAV_SAT, NULL, 0)) {
    return 0;
  }

  // Wait for matching response
  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_NAV && _lastMsgId == UBX_NAV_SAT) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        if (usablePayload < sizeof(UBX_NAV_SAT_header_t)) {
          return 0;
        }

        memcpy(header, _buffer + 6, sizeof(UBX_NAV_SAT_header_t));

        uint16_t svBytesAvail = usablePayload - sizeof(UBX_NAV_SAT_header_t);
        uint8_t svsInPayload = svBytesAvail / sizeof(UBX_NAV_SAT_sv_t);

        uint8_t svsToRead = min(svsInPayload, maxSvs);
        svsToRead = min(svsToRead, header->numSvs);

        for (uint8_t i = 0; i < svsToRead; i++) {
          memcpy(&svArray[i],
                 _buffer + 6 + sizeof(UBX_NAV_SAT_header_t) +
                     (i * sizeof(UBX_NAV_SAT_sv_t)),
                 sizeof(UBX_NAV_SAT_sv_t));
        }

        return svsToRead;
      }
    }
    delay(1);
  }

  return 0;
}

bool Adafruit_UBX::resetReceiver(uint16_t navBbrMask, uint8_t resetMode) {
  UBX_CFG_RST_t rst;
  rst.navBbrMask = navBbrMask;
  rst.resetMode = resetMode;
  rst.reserved1 = 0;
  // Don't wait for ACK — module resets immediately
  return sendMessage(UBX_CLASS_CFG, UBX_CFG_RST, (uint8_t*)&rst, sizeof(rst));
}

bool Adafruit_UBX::hotStart() {
  return resetReceiver(0x0000, 0x01);
}

bool Adafruit_UBX::warmStart() {
  return resetReceiver(0x0001, 0x01);
}

bool Adafruit_UBX::coldStart() {
  return resetReceiver(0xFFFF, 0x01);
}

bool Adafruit_UBX::setRate(uint16_t measRateMs, uint16_t navRate,
                           uint16_t timeRef) {
  UBX_CFG_RATE_t rate;
  rate.measRate = measRateMs;
  rate.navRate = navRate;
  rate.timeRef = timeRef;
  UBXSendStatus status = sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_RATE,
                                            (uint8_t*)&rate, sizeof(rate));
  return (status == UBX_SEND_SUCCESS);
}

bool Adafruit_UBX::getRate(UBX_CFG_RATE_t* rate) {
  return poll(UBX_CLASS_CFG, UBX_CFG_RATE, rate, sizeof(UBX_CFG_RATE_t));
}

bool Adafruit_UBX::saveConfig() {
  UBX_CFG_CFG_t cfg;
  cfg.clearMask = 0x00000000;
  cfg.saveMask = 0x0000FFFF; // save all sections
  cfg.loadMask = 0x00000000;
  UBXSendStatus status = sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_CFG,
                                            (uint8_t*)&cfg, sizeof(cfg));
  return (status == UBX_SEND_SUCCESS);
}

bool Adafruit_UBX::loadDefaults() {
  UBX_CFG_CFG_t cfg;
  cfg.clearMask = 0x0000FFFF; // clear all
  cfg.saveMask = 0x00000000;
  cfg.loadMask = 0x0000FFFF; // load all from defaults
  UBXSendStatus status = sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_CFG,
                                            (uint8_t*)&cfg, sizeof(cfg));
  return (status == UBX_SEND_SUCCESS);
}

uint8_t Adafruit_UBX::pollMONVER(UBX_MON_VER_header_t* header,
                                 UBX_MON_VER_ext_t* extArray, uint8_t maxExt,
                                 uint16_t timeout_ms) {
  if (!sendMessage(UBX_CLASS_MON, UBX_MON_VER, NULL, 0)) {
    return 0;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_MON && _lastMsgId == UBX_MON_VER) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        if (usablePayload < sizeof(UBX_MON_VER_header_t)) {
          return 0;
        }

        memcpy(header, _buffer + 6, sizeof(UBX_MON_VER_header_t));

        uint16_t extBytes = usablePayload - sizeof(UBX_MON_VER_header_t);
        uint8_t extCount = extBytes / sizeof(UBX_MON_VER_ext_t);
        extCount = min(extCount, maxExt);

        for (uint8_t i = 0; i < extCount; i++) {
          memcpy(&extArray[i],
                 _buffer + 6 + sizeof(UBX_MON_VER_header_t) +
                     (i * sizeof(UBX_MON_VER_ext_t)),
                 sizeof(UBX_MON_VER_ext_t));
        }
        return extCount;
      }
    }
    delay(1);
  }
  return 0;
}

/*!
 *  @brief  Poll NAV-POSLLH message
 *  @param  posllh Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVPOSLLH(UBX_NAV_POSLLH_t* posllh,
                                 uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_POSLLH, posllh, sizeof(UBX_NAV_POSLLH_t),
              timeout_ms);
}

/*!
 *  @brief  Poll NAV-VELNED message
 *  @param  velned Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVVELNED(UBX_NAV_VELNED_t* velned,
                                 uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_VELNED, velned, sizeof(UBX_NAV_VELNED_t),
              timeout_ms);
}

/*!
 *  @brief  Poll NAV-TIMEUTC message
 *  @param  timeutc Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVTIMEUTC(UBX_NAV_TIMEUTC_t* timeutc,
                                  uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_TIMEUTC, timeutc,
              sizeof(UBX_NAV_TIMEUTC_t), timeout_ms);
}

/*!
 *  @brief  Poll NAV-POSECEF message
 *  @param  posecef Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVPOSECEF(UBX_NAV_POSECEF_t* posecef,
                                  uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_POSECEF, posecef,
              sizeof(UBX_NAV_POSECEF_t), timeout_ms);
}

/*!
 *  @brief  Poll NAV-VELECEF message
 *  @param  velecef Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVVELECEF(UBX_NAV_VELECEF_t* velecef,
                                  uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_VELECEF, velecef,
              sizeof(UBX_NAV_VELECEF_t), timeout_ms);
}

/*!
 *  @brief  Poll NAV-CLOCK message
 *  @param  clock Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVCLOCK(UBX_NAV_CLOCK_t* clock, uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_CLOCK, clock, sizeof(UBX_NAV_CLOCK_t),
              timeout_ms);
}

/*!
 *  @brief  Poll NAV-EOE message
 *  @param  eoe Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVEOE(UBX_NAV_EOE_t* eoe, uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_EOE, eoe, sizeof(UBX_NAV_EOE_t),
              timeout_ms);
}

/*!
 *  @brief  Poll NAV-TIMEGPS message
 *  @param  timegps Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNAVTIMEGPS(UBX_NAV_TIMEGPS_t* timegps,
                                  uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_TIMEGPS, timegps,
              sizeof(UBX_NAV_TIMEGPS_t), timeout_ms);
}

/*!
 *  @brief  Configure the GPS module to output only UBX protocol (disables NMEA)
 *  @param  portID Port identifier (UBX_PORT_DDC, UBX_PORT_UART1, etc.)
 *  @param  checkAck Whether to wait for acknowledgment
 *  @param  timeout_ms Maximum time to wait for acknowledgment in milliseconds
 *  @return UBXSendStatus indicating success, failure, or timeout
 */
UBXSendStatus Adafruit_UBX::setUBXOnly(UBXPortId portID, bool checkAck,
                                       uint16_t timeout_ms) {
  UBX_CFG_PRT_t cfgPrt;

  // Zero out the structure
  memset(&cfgPrt, 0, sizeof(cfgPrt));

  // Set the port ID
  cfgPrt.fields.portID = portID;

  // Configure the port appropriately
  switch (portID) {
    case UBX_PORT_DDC: // I2C/DDC port
      // Set the I2C address to 0x42 (the default)
      // For DDC, the mode field contains the I2C address in bits 7:1
      cfgPrt.fields.mode = 0x42 << 1; // 0x84

      // Set protocol masks to UBX only
      cfgPrt.fields.inProtoMask = UBX_PROTOCOL_UBX;
      cfgPrt.fields.outProtoMask = UBX_PROTOCOL_UBX;
      break;

    case UBX_PORT_UART1: // Fall through
    case UBX_PORT_UART2: // UART ports
      // Keep current baud rate (baudRate = 0 keeps current setting)
      // Set 8N1 mode for binary protocol
      cfgPrt.fields.mode = UBX_UART_MODE_8N1;
      // Set protocol masks to UBX only
      cfgPrt.fields.inProtoMask = UBX_PROTOCOL_UBX;
      cfgPrt.fields.outProtoMask = UBX_PROTOCOL_UBX;
      break;

    case UBX_PORT_USB: // USB port
      // Set protocol masks to UBX only
      cfgPrt.fields.inProtoMask = UBX_PROTOCOL_UBX;
      cfgPrt.fields.outProtoMask = UBX_PROTOCOL_UBX;
      break;

    case UBX_PORT_SPI: // SPI port
      // Set protocol masks to UBX only
      cfgPrt.fields.outProtoMask = UBX_PROTOCOL_UBX;
      break;

    default:
      if (verbose_debug > 0) {
        Serial.println(F("UBX: Invalid port ID"));
      }
      return UBX_SEND_FAIL; // Invalid port ID
  }

  // Send the message and wait for acknowledgment if requested
  if (checkAck) {
    return sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_PRT, cfgPrt.raw,
                              sizeof(cfgPrt), timeout_ms);
  } else {
    if (sendMessage(UBX_CLASS_CFG, UBX_CFG_PRT, cfgPrt.raw, sizeof(cfgPrt))) {
      return UBX_SEND_SUCCESS;
    } else {
      return UBX_SEND_FAIL;
    }
  }
}

/*!
 *  @brief  Sets the callback function for UBX messages
 *  @param  callback Function pointer to call when a complete UBX message is
 * received
 */
void Adafruit_UBX::setMessageCallback(UBXMessageCallback callback) {
  onUBXMessage = callback;
}

/*!
 *  @brief  Reset the parser state machine
 */
void Adafruit_UBX::resetParser() {
  _parserState = WAIT_SYNC_1;
  _payloadCounter = 0;
  _payloadLength = 0;
  _checksumA = 0;
  _checksumB = 0;
}

/*!
 *  @brief  Calculate UBX checksum according to protocol
 *  @param  buffer Pointer to the data buffer
 *  @param  len Length of data to checksum
 *  @param  checksumA Reference to store the first checksum byte
 *  @param  checksumB Reference to store the second checksum byte
 */
void Adafruit_UBX::calculateChecksum(uint8_t* buffer, uint16_t len,
                                     uint8_t& checksumA, uint8_t& checksumB) {
  checksumA = 0;
  checksumB = 0;

  for (uint16_t i = 0; i < len; i++) {
    checksumA += buffer[i];
    checksumB += checksumA;
  }
}

void Adafruit_UBX::printHex(uint8_t val) {
  if (val < 0x10)
    Serial.print(F("0"));
  Serial.print(val, HEX);
}

void Adafruit_UBX::printHexBuffer(const __FlashStringHelper* label,
                                  uint8_t* buf, uint16_t len) {
  Serial.print(label);
  Serial.print(F("["));
  for (uint16_t i = 0; i < len; i++) {
    printHex(buf[i]);
    Serial.print(F(" "));
  }
  Serial.print(F("] "));
}

/*!
 *  @brief  Check for new UBX messages and parse them
 *  @return True if a complete message was parsed
 */
bool Adafruit_UBX::checkMessages() {
  bool messageReceived = false;

  // Process all available bytes
  while (_stream->available()) {
    uint8_t incomingByte = _stream->read();

    // State machine for UBX protocol parsing
    switch (_parserState) {
      case WAIT_SYNC_1:
        if (incomingByte == UBX_SYNC_CHAR_1) {
          _parserState = WAIT_SYNC_2;
          _buffer[0] = incomingByte; // Store for checksum calculation
        }
        break;

      case WAIT_SYNC_2:
        if (incomingByte == UBX_SYNC_CHAR_2) {
          _parserState = GET_CLASS;
          _buffer[1] = incomingByte; // Store for checksum calculation
        } else {
          resetParser(); // Invalid sync char, reset
        }
        break;

      case GET_CLASS:
        _msgClass = incomingByte;
        _buffer[2] = incomingByte; // Store for checksum calculation
        _checksumA = 0;
        _checksumB = 0;
        updateChecksum(incomingByte);
        _parserState = GET_ID;
        break;

      case GET_ID:
        _msgId = incomingByte;
        _buffer[3] = incomingByte; // Store for checksum calculation
        updateChecksum(incomingByte);
        _parserState = GET_LENGTH_1;
        break;

      case GET_LENGTH_1:
        _payloadLength = incomingByte;
        _buffer[4] = incomingByte; // Store for checksum calculation
        updateChecksum(incomingByte);
        _parserState = GET_LENGTH_2;
        break;

      case GET_LENGTH_2:
        _payloadLength |= (incomingByte << 8);
        _buffer[5] = incomingByte; // Store for checksum calculation
        updateChecksum(incomingByte);

        _payloadCounter = 0;
        if (_payloadLength == 0) {
          _parserState = GET_CHECKSUM_A;
        } else {
          _parserState = GET_PAYLOAD;
        }
        break;

      case GET_PAYLOAD:
        if (_payloadCounter < _payloadLength) {
          if (_payloadCounter < MAX_PAYLOAD_SIZE) {
            _buffer[6 + _payloadCounter] = incomingByte;
          }
          _payloadCounter++;
          updateChecksum(incomingByte);

          if (_payloadCounter == _payloadLength) {
            _parserState = GET_CHECKSUM_A;
          }
        }
        break;

      case GET_CHECKSUM_A:
        if (incomingByte == _checksumA) {
          _parserState = GET_CHECKSUM_B; // Checksum A matches
        } else {
          resetParser(); // Invalid checksum, reset
        }
        break;

      case GET_CHECKSUM_B:
        if (incomingByte == _checksumB) {
          // We have a valid message!
          if (onUBXMessage != NULL) {
            onUBXMessage(_msgClass, _msgId, _payloadLength,
                         _buffer + 6); // Call the callback with the message
          }
          messageReceived = true;

          _lastMsgClass = _msgClass;
          _lastMsgId = _msgId;
          _lastPayloadLength = _payloadLength;

          // Store a small copy of the payload if it's within size limits
          if (_payloadLength <= sizeof(_lastPayload)) {
            memcpy(_lastPayload, _buffer + 6, _payloadLength);
          }

          if (verbose_debug > 0) {
            Serial.print(F("UBX RX: "));

            // Print header (sync chars, class, id, length)
            uint8_t hdr[6] = {
                UBX_SYNC_CHAR_1,
                UBX_SYNC_CHAR_2,
                _msgClass,
                _msgId,
                static_cast<uint8_t>(_payloadLength & 0xFF),
                static_cast<uint8_t>((_payloadLength >> 8) & 0xFF)};
            printHexBuffer(F("HDR"), hdr, 6);

            // Print payload if verbose debug is enabled
            if (verbose_debug > 1 && _payloadLength > 0) {
              uint16_t payloadToPrint = min(_payloadLength, MAX_PAYLOAD_SIZE);
              printHexBuffer(F("PL"), _buffer + 6, payloadToPrint);
            }

            // Print checksum
            uint8_t checksum[2] = {_checksumA, _checksumB};
            printHexBuffer(F("CS"), checksum, 2);
            Serial.println();
          }
        }

        resetParser(); // Reset for next message
        break;
    }

    // Stop after first complete message so _last fields aren't overwritten
    if (messageReceived)
      break;
  }

  return messageReceived;
}

/*!
 *  @brief  Send a UBX message and wait for acknowledgment
 *  @param  msgClass Message class
 *  @param  msgId Message ID
 *  @param  payload Pointer to the payload data
 *  @param  length Length of payload
 *  @param  timeout_ms Maximum time to wait for acknowledgment
 *  @return UBXSendStatus indicating success, failure, or timeout
 */
UBXSendStatus Adafruit_UBX::sendMessageWithAck(uint8_t msgClass, uint8_t msgId,
                                               uint8_t* payload,
                                               uint16_t length,
                                               uint16_t timeout_ms) {
  // First send the message
  if (!sendMessage(msgClass, msgId, payload, length)) {
    if (verbose_debug > 0) {
      Serial.println(F("UBX ACK: SEND FAIL"));
    }
    return UBX_SEND_FAIL;
  }

  uint32_t startTime = millis();

  // Check for messages until timeout
  while ((millis() - startTime) < timeout_ms) {
    // Process incoming bytes
    if (checkMessages()) {
      // If we have a message handler, it will be called from checkMessages
      // We need to check our last received message
      if (_lastMsgClass == UBX_CLASS_ACK) {
        if (_lastMsgId == UBX_ACK_ACK && _lastPayloadLength >= 2) {
          // ACK-ACK message
          if (_lastPayload[0] == msgClass && _lastPayload[1] == msgId) {
            if (verbose_debug > 0) {
              Serial.print(F("UBX ACK: SUCCESS for message class 0x"));
              printHex(msgClass);
              Serial.print(F(" ID 0x"));
              printHex(msgId);
              Serial.println();
            }
            return UBX_SEND_SUCCESS;
          }
        } else if (_lastMsgId == UBX_ACK_NAK && _lastPayloadLength >= 2) {
          // ACK-NAK message
          if (_lastPayload[0] == msgClass && _lastPayload[1] == msgId) {
            if (verbose_debug > 0) {
              Serial.print(F("UBX ACK: NAK for message class 0x"));
              printHex(msgClass);
              Serial.print(F(" ID 0x"));
              printHex(msgId);
              Serial.println();
            }
            return UBX_SEND_NAK;
          }
        }
      }
    }

    // Short delay
    delay(1);
  }

  if (verbose_debug > 0) {
    Serial.print(
        F("UBX ACK: TIMEOUT waiting for ACK/NAK for message class 0x"));
    printHex(msgClass);
    Serial.print(F(" ID 0x"));
    printHex(msgId);
    Serial.println();
  }

  return UBX_SEND_TIMEOUT;
}

/*!
 *  @brief  Send a UBX message to the GPS module
 *  @param  msgClass Message class
 *  @param  msgId Message ID
 *  @param  payload Pointer to the payload data (can be NULL for zero-length
 * payload)
 *  @param  length Length of payload
 *  @return True if message was sent successfully
 */
bool Adafruit_UBX::sendMessage(uint8_t msgClass, uint8_t msgId,
                               uint8_t* payload, uint16_t length) {
  // Buffer for message (2 sync chars + class + id + 2 length bytes + payload +
  // 2 checksum bytes)
  uint8_t msgBuffer[length + 8];

  // Sync characters
  msgBuffer[0] = UBX_SYNC_CHAR_1;
  msgBuffer[1] = UBX_SYNC_CHAR_2;

  // Message class and ID
  msgBuffer[2] = msgClass;
  msgBuffer[3] = msgId;

  // Length (little endian)
  msgBuffer[4] = length & 0xFF;
  msgBuffer[5] = (length >> 8) & 0xFF;

  // Payload
  if (payload != NULL && length > 0) {
    memcpy(&msgBuffer[6], payload, length);
  }

  // Calculate checksum
  uint8_t checksumA, checksumB;
  calculateChecksum(&msgBuffer[2], length + 4, checksumA, checksumB);

  msgBuffer[6 + length] = checksumA;
  msgBuffer[7 + length] = checksumB;

  // Debug output
  if (verbose_debug > 0) {
    Serial.print(F("UBX TX: "));
    printHexBuffer(F("HDR"), msgBuffer, 6);
    if (verbose_debug > 1 && length > 0) {
      printHexBuffer(F("PL"), msgBuffer + 6, length);
    }
    // CS is the last 2 bytes
    printHexBuffer(F("CS"), msgBuffer + 6 + length, 2);
    Serial.println();
  }

  // Send the message
  size_t written = _stream->write(msgBuffer, length + 8);

  return (written == length + 8);
}

// =====================================================================
// Phase 3: CFG Message Implementations
// =====================================================================

/*!
 *  @brief  Poll CFG-NAV5 message (navigation engine settings)
 *  @param  nav5 Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgNav5(UBX_CFG_NAV5_t* nav5, uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_NAV5, nav5, sizeof(UBX_CFG_NAV5_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-NAV5 message (navigation engine settings)
 *  @param  nav5 Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgNav5(UBX_CFG_NAV5_t* nav5) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_NAV5, (uint8_t*)nav5, sizeof(UBX_CFG_NAV5_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Set the dynamic platform model
 *  @param  model Dynamic model (see UBX_DYNMODEL_* defines)
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setDynamicModel(uint8_t model) {
  UBX_CFG_NAV5_t nav5;
  if (!pollCfgNav5(&nav5)) {
    return false;
  }
  nav5.mask = UBX_NAV5_MASK_DYN;
  nav5.dynModel = model;
  return setCfgNav5(&nav5);
}

/*!
 *  @brief  Get current dynamic platform model
 *  @return Dynamic model value, or 0xFF on failure
 */
uint8_t Adafruit_UBX::getDynamicModel() {
  UBX_CFG_NAV5_t nav5;
  if (!pollCfgNav5(&nav5)) {
    return 0xFF;
  }
  return nav5.dynModel;
}

/*!
 *  @brief  Set the position fix mode
 *  @param  mode Fix mode (1=2D only, 2=3D only, 3=auto 2D/3D)
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setFixMode(uint8_t mode) {
  UBX_CFG_NAV5_t nav5;
  if (!pollCfgNav5(&nav5)) {
    return false;
  }
  nav5.mask = UBX_NAV5_MASK_POSFIX;
  nav5.fixMode = mode;
  return setCfgNav5(&nav5);
}

/*!
 *  @brief  Poll CFG-GNSS message (GNSS system configuration)
 *  @param  header Pointer to header struct to fill
 *  @param  blocks Array of GNSS config blocks to fill
 *  @param  maxBlocks Maximum number of blocks the array can hold
 *  @param  timeout_ms Timeout in milliseconds
 *  @return Number of blocks read, or 0 on failure
 */
uint8_t Adafruit_UBX::pollCfgGnss(UBX_CFG_GNSS_header_t* header,
                                  UBX_CFG_GNSS_block_t* blocks,
                                  uint8_t maxBlocks, uint16_t timeout_ms) {
  if (!sendMessage(UBX_CLASS_CFG, UBX_CFG_GNSS, NULL, 0)) {
    return 0;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_CFG && _lastMsgId == UBX_CFG_GNSS) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        if (usablePayload < sizeof(UBX_CFG_GNSS_header_t)) {
          return 0;
        }

        memcpy(header, _buffer + 6, sizeof(UBX_CFG_GNSS_header_t));

        uint16_t blockBytes = usablePayload - sizeof(UBX_CFG_GNSS_header_t);
        uint8_t blocksInPayload = blockBytes / sizeof(UBX_CFG_GNSS_block_t);
        uint8_t blocksToRead = min(blocksInPayload, maxBlocks);
        blocksToRead = min(blocksToRead, header->numConfigBlocks);

        for (uint8_t i = 0; i < blocksToRead; i++) {
          memcpy(&blocks[i],
                 _buffer + 6 + sizeof(UBX_CFG_GNSS_header_t) +
                     (i * sizeof(UBX_CFG_GNSS_block_t)),
                 sizeof(UBX_CFG_GNSS_block_t));
        }
        return blocksToRead;
      }
    }
    delay(1);
  }
  return 0;
}

/*!
 *  @brief  Set CFG-GNSS message (GNSS system configuration)
 *  @param  header Pointer to header struct
 *  @param  blocks Array of GNSS config blocks
 *  @param  numBlocks Number of blocks to send
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgGnss(UBX_CFG_GNSS_header_t* header,
                              UBX_CFG_GNSS_block_t* blocks, uint8_t numBlocks) {
  uint16_t totalLen = sizeof(UBX_CFG_GNSS_header_t) +
                      (numBlocks * sizeof(UBX_CFG_GNSS_block_t));
  uint8_t payload[totalLen];

  header->numConfigBlocks = numBlocks;
  memcpy(payload, header, sizeof(UBX_CFG_GNSS_header_t));
  memcpy(payload + sizeof(UBX_CFG_GNSS_header_t), blocks,
         numBlocks * sizeof(UBX_CFG_GNSS_block_t));

  UBXSendStatus status =
      sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_GNSS, payload, totalLen);
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Enable or disable a specific GNSS system
 *  @param  gnssId GNSS identifier (see UBX_GNSS_ID_* defines)
 *  @param  enable True to enable, false to disable
 *  @return True if successful
 */
bool Adafruit_UBX::enableGNSS(uint8_t gnssId, bool enable) {
  UBX_CFG_GNSS_header_t header;
  UBX_CFG_GNSS_block_t blocks[8];
  uint8_t numBlocks = pollCfgGnss(&header, blocks, 8);
  if (numBlocks == 0) {
    return false;
  }

  for (uint8_t i = 0; i < numBlocks; i++) {
    if (blocks[i].gnssId == gnssId) {
      if (enable) {
        blocks[i].flags |= UBX_GNSS_FLAG_ENABLE;
      } else {
        blocks[i].flags &= ~UBX_GNSS_FLAG_ENABLE;
      }
      break;
    }
  }

  return setCfgGnss(&header, blocks, numBlocks);
}

/*!
 *  @brief  Poll CFG-NMEA message (NMEA protocol configuration)
 *  @param  nmea Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgNmea(UBX_CFG_NMEA_t* nmea, uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_NMEA, nmea, sizeof(UBX_CFG_NMEA_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-NMEA message (NMEA protocol configuration)
 *  @param  nmea Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgNmea(UBX_CFG_NMEA_t* nmea) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_NMEA, (uint8_t*)nmea, sizeof(UBX_CFG_NMEA_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Poll CFG-ANT message (antenna control settings)
 *  @param  ant Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgAnt(UBX_CFG_ANT_t* ant, uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_ANT, ant, sizeof(UBX_CFG_ANT_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-ANT message (antenna control settings)
 *  @param  ant Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgAnt(UBX_CFG_ANT_t* ant) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_ANT, (uint8_t*)ant, sizeof(UBX_CFG_ANT_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Poll CFG-SBAS message (SBAS configuration)
 *  @param  sbas Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgSbas(UBX_CFG_SBAS_t* sbas, uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_SBAS, sbas, sizeof(UBX_CFG_SBAS_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-SBAS message (SBAS configuration)
 *  @param  sbas Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgSbas(UBX_CFG_SBAS_t* sbas) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_SBAS, (uint8_t*)sbas, sizeof(UBX_CFG_SBAS_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Enable or disable SBAS
 *  @param  enable True to enable, false to disable
 *  @return True if successful
 */
bool Adafruit_UBX::enableSBAS(bool enable) {
  UBX_CFG_SBAS_t sbas;
  if (!pollCfgSbas(&sbas)) {
    return false;
  }
  if (enable) {
    sbas.mode |= UBX_SBAS_MODE_ENABLED;
  } else {
    sbas.mode &= ~UBX_SBAS_MODE_ENABLED;
  }
  return setCfgSbas(&sbas);
}

/*!
 *  @brief  Poll CFG-INF message (info message configuration) for a protocol
 *  @param  protocolID Protocol ID (0=UBX, 1=NMEA)
 *  @param  blocks Array of INF config blocks to fill
 *  @param  maxBlocks Maximum number of blocks
 *  @param  timeout_ms Timeout in milliseconds
 *  @return Number of blocks read, or 0 on failure
 */
uint8_t Adafruit_UBX::pollCfgInf(uint8_t protocolID,
                                 UBX_CFG_INF_block_t* blocks, uint8_t maxBlocks,
                                 uint16_t timeout_ms) {
  // Poll with protocol ID as 1-byte payload
  if (!sendMessage(UBX_CLASS_CFG, UBX_CFG_INF, &protocolID, 1)) {
    return 0;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_CFG && _lastMsgId == UBX_CFG_INF) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        uint8_t blocksInPayload = usablePayload / sizeof(UBX_CFG_INF_block_t);
        uint8_t blocksToRead = min(blocksInPayload, maxBlocks);

        for (uint8_t i = 0; i < blocksToRead; i++) {
          memcpy(&blocks[i], _buffer + 6 + (i * sizeof(UBX_CFG_INF_block_t)),
                 sizeof(UBX_CFG_INF_block_t));
        }
        return blocksToRead;
      }
    }
    delay(1);
  }
  return 0;
}

/*!
 *  @brief  Set CFG-INF message (info message configuration)
 *  @param  blocks Array of INF config blocks
 *  @param  numBlocks Number of blocks to send
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgInf(UBX_CFG_INF_block_t* blocks, uint8_t numBlocks) {
  uint16_t totalLen = numBlocks * sizeof(UBX_CFG_INF_block_t);
  UBXSendStatus status = sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_INF,
                                            (uint8_t*)blocks, totalLen);
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Poll CFG-RXM message (receiver manager configuration)
 *  @param  rxm Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgRxm(UBX_CFG_RXM_t* rxm, uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_RXM, rxm, sizeof(UBX_CFG_RXM_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-RXM message (receiver manager configuration)
 *  @param  rxm Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgRxm(UBX_CFG_RXM_t* rxm) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_RXM, (uint8_t*)rxm, sizeof(UBX_CFG_RXM_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Enable or disable power save mode
 *  @param  enable True for power save mode, false for continuous
 *  @return True if successful
 */
bool Adafruit_UBX::setPowerSave(bool enable) {
  UBX_CFG_RXM_t rxm;
  rxm.reserved1 = 8; // Must always be 8
  rxm.lpMode = enable ? UBX_RXM_LPMODE_POWERSAVE : UBX_RXM_LPMODE_CONTINUOUS;
  return setCfgRxm(&rxm);
}

// =====================================================================
