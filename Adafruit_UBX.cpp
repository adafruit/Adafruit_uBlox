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
// Phase 4: Power Management Message Implementations
// =====================================================================

/*!
 *  @brief  Poll CFG-PM2 message (extended power management configuration)
 *  @param  pm2 Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgPm2(UBX_CFG_PM2_t* pm2, uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_PM2, pm2, sizeof(UBX_CFG_PM2_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-PM2 message (extended power management configuration)
 *  @param  pm2 Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgPm2(UBX_CFG_PM2_t* pm2) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_PM2, (uint8_t*)pm2, sizeof(UBX_CFG_PM2_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Poll CFG-PMS message (power mode setup)
 *  @param  pms Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgPms(UBX_CFG_PMS_t* pms, uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_PMS, pms, sizeof(UBX_CFG_PMS_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-PMS message (power mode setup)
 *  @param  pms Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgPms(UBX_CFG_PMS_t* pms) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_PMS, (uint8_t*)pms, sizeof(UBX_CFG_PMS_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Set power mode using simple preset values
 *  @param  mode Power mode: 0=Full, 1=Balanced, 2=Interval,
 *               3=Aggressive1Hz, 4=Aggressive2Hz, 5=Aggressive4Hz
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setPowerMode(uint8_t mode) {
  UBX_CFG_PMS_t pms;
  memset(&pms, 0, sizeof(pms));
  pms.version = 0;
  pms.powerSetupValue = mode;
  // period and onTime only used for Interval mode (0x02)
  // For other modes, they must be 0
  pms.period = 0;
  pms.onTime = 0;
  return setCfgPms(&pms);
}

/*!
 *  @brief  Send RXM-PMREQ (v0) to request power management task
 *  @param  duration Duration of task in ms. 0=wake on pin only. Max ~12 days.
 *  @param  flags Task flags (use UBX_PMREQ_FLAG_BACKUP to enter backup mode)
 *  @return True if message was sent (no ACK expected - module enters backup)
 *  @note   This is a send-only command. Do NOT expect ACK/NAK response.
 *          The module will enter backup mode immediately upon receiving this.
 */
bool Adafruit_UBX::sendPmreq(uint32_t duration, uint32_t flags) {
  UBX_RXM_PMREQ_t pmreq;
  pmreq.duration = duration;
  pmreq.flags = flags;
  // Send without waiting for ACK - module enters backup immediately
  return sendMessage(UBX_CLASS_RXM, UBX_RXM_PMREQ, (uint8_t*)&pmreq,
                     sizeof(pmreq));
}

/*!
 *  @brief  Send RXM-PMREQ (v1) with wakeup source configuration
 *  @param  duration Duration of task in ms. 0=wake on pin only. Max ~12 days.
 *  @param  flags Task flags (UBX_PMREQ_FLAG_BACKUP, UBX_PMREQ_FLAG_FORCE)
 *  @param  wakeupSources Wakeup pins (UBX_PMREQ_WAKE_UARTRX, EXTINT0, etc.)
 *  @return True if message was sent (no ACK expected - module enters backup)
 *  @note   This is a send-only command. Do NOT expect ACK/NAK response.
 */
bool Adafruit_UBX::sendPmreqV1(uint32_t duration, uint32_t flags,
                               uint32_t wakeupSources) {
  UBX_RXM_PMREQ_V1_t pmreq;
  pmreq.version = 0;
  memset(pmreq.reserved1, 0, sizeof(pmreq.reserved1));
  pmreq.duration = duration;
  pmreq.flags = flags;
  pmreq.wakeupSources = wakeupSources;
  // Send without waiting for ACK - module enters backup immediately
  return sendMessage(UBX_CLASS_RXM, UBX_RXM_PMREQ, (uint8_t*)&pmreq,
                     sizeof(pmreq));
}

/*!
 *  @brief  Poll MON-HW message (hardware status)
 *  @param  hw Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollMonHw(UBX_MON_HW_t* hw, uint16_t timeout_ms) {
  return poll(UBX_CLASS_MON, UBX_MON_HW, hw, sizeof(UBX_MON_HW_t), timeout_ms);
}

// =====================================================================
// Phase 5: Monitoring & Diagnostics Message Implementations
// =====================================================================

/*!
 *  @brief  Poll MON-GNSS message (GNSS system information)
 *  @param  gnss Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollMonGnss(UBX_MON_GNSS_t* gnss, uint16_t timeout_ms) {
  return poll(UBX_CLASS_MON, UBX_MON_GNSS, gnss, sizeof(UBX_MON_GNSS_t),
              timeout_ms);
}

/*!
 *  @brief  Poll MON-HW2 message (extended hardware status)
 *  @param  hw2 Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollMonHw2(UBX_MON_HW2_t* hw2, uint16_t timeout_ms) {
  return poll(UBX_CLASS_MON, UBX_MON_HW2, hw2, sizeof(UBX_MON_HW2_t),
              timeout_ms);
}

/*!
 *  @brief  Poll MON-IO message (I/O system status)
 *  @param  ports Array of port structs to fill
 *  @param  maxPorts Maximum number of ports the array can hold
 *  @param  timeout_ms Timeout in milliseconds
 *  @return Number of ports read, or 0 on failure
 */
uint8_t Adafruit_UBX::pollMonIo(UBX_MON_IO_port_t* ports, uint8_t maxPorts,
                                uint16_t timeout_ms) {
  if (!sendMessage(UBX_CLASS_MON, UBX_MON_IO, NULL, 0)) {
    return 0;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_MON && _lastMsgId == UBX_MON_IO) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        uint8_t portsInPayload = usablePayload / sizeof(UBX_MON_IO_port_t);
        uint8_t portsToRead = min(portsInPayload, maxPorts);

        for (uint8_t i = 0; i < portsToRead; i++) {
          memcpy(&ports[i], _buffer + 6 + (i * sizeof(UBX_MON_IO_port_t)),
                 sizeof(UBX_MON_IO_port_t));
        }
        return portsToRead;
      }
    }
    delay(1);
  }
  return 0;
}

/*!
 *  @brief  Poll MON-MSGPP message (message parse/process status)
 *  @param  msgpp Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollMonMsgpp(UBX_MON_MSGPP_t* msgpp, uint16_t timeout_ms) {
  return poll(UBX_CLASS_MON, UBX_MON_MSGPP, msgpp, sizeof(UBX_MON_MSGPP_t),
              timeout_ms);
}

/*!
 *  @brief  Poll MON-RXBUF message (receiver buffer status)
 *  @param  rxbuf Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollMonRxbuf(UBX_MON_RXBUF_t* rxbuf, uint16_t timeout_ms) {
  return poll(UBX_CLASS_MON, UBX_MON_RXBUF, rxbuf, sizeof(UBX_MON_RXBUF_t),
              timeout_ms);
}

/*!
 *  @brief  Poll MON-TXBUF message (transmitter buffer status)
 *  @param  txbuf Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollMonTxbuf(UBX_MON_TXBUF_t* txbuf, uint16_t timeout_ms) {
  return poll(UBX_CLASS_MON, UBX_MON_TXBUF, txbuf, sizeof(UBX_MON_TXBUF_t),
              timeout_ms);
}

/*!
 *  @brief  Poll SEC-UNIQID message (unique chip ID)
 *  @param  uniqid Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollSecUniqid(UBX_SEC_UNIQID_t* uniqid,
                                 uint16_t timeout_ms) {
  return poll(UBX_CLASS_SEC, UBX_SEC_UNIQID, uniqid, sizeof(UBX_SEC_UNIQID_t),
              timeout_ms);
}

/*!
 *  @brief  Check if last received message was an INF message
 *  @return True if last message was class INF (0x04)
 */
bool Adafruit_UBX::wasLastMessageINF() {
  return (_lastMsgClass == UBX_CLASS_INF);
}

/*!
 *  @brief  Get the type of the last INF message
 *  @return INF message ID (0x00-0x04) or 0xFF if last message wasn't INF
 */
uint8_t Adafruit_UBX::getLastINFType() {
  if (_lastMsgClass != UBX_CLASS_INF) {
    return 0xFF;
  }
  return _lastMsgId;
}

/*!
 *  @brief  Get the string content of the last INF message
 *  @param  buffer Pointer to buffer to store the string
 *  @param  maxLen Maximum length of buffer (including null terminator)
 *  @return Number of characters copied (not including null terminator)
 *  @note   Buffer will be null-terminated. Returns 0 if last message wasn't INF
 */
uint16_t Adafruit_UBX::getLastINFString(char* buffer, uint16_t maxLen) {
  if (_lastMsgClass != UBX_CLASS_INF || maxLen == 0) {
    if (maxLen > 0) {
      buffer[0] = '\0';
    }
    return 0;
  }

  uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
  uint16_t copyLen = min(usablePayload, (uint16_t)(maxLen - 1));

  memcpy(buffer, _buffer + 6, copyLen);
  buffer[copyLen] = '\0';

  return copyLen;
}

// =====================================================================
// Phase 6: Advanced Feature Message Implementations
// =====================================================================

/*!
 *  @brief  Poll NAV-ODO message (odometer solution)
 *  @param  odo Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNavOdo(UBX_NAV_ODO_t* odo, uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_ODO, odo, sizeof(UBX_NAV_ODO_t),
              timeout_ms);
}

/*!
 *  @brief  Reset the odometer
 *  @return True if acknowledged
 *  @note   This resets the 'distance' field in NAV-ODO; totalDistance is only
 *          reset by a cold start.
 */
bool Adafruit_UBX::resetOdometer() {
  // NAV-RESETODO has no payload
  UBXSendStatus status =
      sendMessageWithAck(UBX_CLASS_NAV, UBX_NAV_RESETODO, NULL, 0);
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Poll NAV-HPPOSLLH message (high precision geodetic position)
 *  @param  hppos Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNavHpposllh(UBX_NAV_HPPOSLLH_t* hppos,
                                   uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, hppos,
              sizeof(UBX_NAV_HPPOSLLH_t), timeout_ms);
}

/*!
 *  @brief  Get high precision position (convenience wrapper)
 *  @param  hppos Pointer to struct to fill
 *  @return True if response received
 */
bool Adafruit_UBX::getHighPrecisionPosition(UBX_NAV_HPPOSLLH_t* hppos) {
  return pollNavHpposllh(hppos, 1000);
}

/*!
 *  @brief  Poll NAV-HPPOSECEF message (high precision ECEF position)
 *  @param  hppos Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNavHpposecef(UBX_NAV_HPPOSECEF_t* hppos,
                                    uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_HPPOSECEF, hppos,
              sizeof(UBX_NAV_HPPOSECEF_t), timeout_ms);
}

/*!
 *  @brief  Poll NAV-RELPOSNED message (relative position NED for RTK)
 *  @param  relpos Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 *  @note   Only available on High Precision GNSS products
 */
bool Adafruit_UBX::pollNavRelposned(UBX_NAV_RELPOSNED_t* relpos,
                                    uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, relpos,
              sizeof(UBX_NAV_RELPOSNED_t), timeout_ms);
}

/*!
 *  @brief  Poll NAV-SVIN message (survey-in status)
 *  @param  svin Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 *  @note   Only available on High Precision GNSS products
 */
bool Adafruit_UBX::pollNavSvin(UBX_NAV_SVIN_t* svin, uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_SVIN, svin, sizeof(UBX_NAV_SVIN_t),
              timeout_ms);
}

/*!
 *  @brief  Poll NAV-GEOFENCE message (geofencing status)
 *  @param  header Pointer to header struct to fill
 *  @param  fences Array of fence status structs to fill
 *  @param  maxFences Maximum number of fences the array can hold
 *  @param  timeout_ms Timeout in milliseconds
 *  @return Number of fences read, or 0 on failure
 */
uint8_t Adafruit_UBX::pollNavGeofence(UBX_NAV_GEOFENCE_header_t* header,
                                      UBX_NAV_GEOFENCE_fence_t* fences,
                                      uint8_t maxFences, uint16_t timeout_ms) {
  if (!sendMessage(UBX_CLASS_NAV, UBX_NAV_GEOFENCE, NULL, 0)) {
    return 0;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_NAV && _lastMsgId == UBX_NAV_GEOFENCE) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        if (usablePayload < sizeof(UBX_NAV_GEOFENCE_header_t)) {
          return 0;
        }

        memcpy(header, _buffer + 6, sizeof(UBX_NAV_GEOFENCE_header_t));

        uint16_t fenceBytes = usablePayload - sizeof(UBX_NAV_GEOFENCE_header_t);
        uint8_t fencesInPayload = fenceBytes / sizeof(UBX_NAV_GEOFENCE_fence_t);
        uint8_t fencesToRead = min(fencesInPayload, maxFences);
        fencesToRead = min(fencesToRead, header->numFences);

        for (uint8_t i = 0; i < fencesToRead; i++) {
          memcpy(&fences[i],
                 _buffer + 6 + sizeof(UBX_NAV_GEOFENCE_header_t) +
                     (i * sizeof(UBX_NAV_GEOFENCE_fence_t)),
                 sizeof(UBX_NAV_GEOFENCE_fence_t));
        }
        return fencesToRead;
      }
    }
    delay(1);
  }
  return 0;
}

/*!
 *  @brief  Poll NAV-TIMELS message (leap second information)
 *  @param  timels Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollNavTimels(UBX_NAV_TIMELS_t* timels,
                                 uint16_t timeout_ms) {
  return poll(UBX_CLASS_NAV, UBX_NAV_TIMELS, timels, sizeof(UBX_NAV_TIMELS_t),
              timeout_ms);
}

/*!
 *  @brief  Poll CFG-TMODE3 message (time mode 3 / RTK base config)
 *  @param  tmode3 Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 *  @note   Only available on High Precision GNSS products
 */
bool Adafruit_UBX::pollCfgTmode3(UBX_CFG_TMODE3_t* tmode3,
                                 uint16_t timeout_ms) {
  return poll(UBX_CLASS_CFG, UBX_CFG_TMODE3, tmode3, sizeof(UBX_CFG_TMODE3_t),
              timeout_ms);
}

/*!
 *  @brief  Set CFG-TMODE3 message (time mode 3 / RTK base config)
 *  @param  tmode3 Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgTmode3(UBX_CFG_TMODE3_t* tmode3) {
  UBXSendStatus status =
      sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_TMODE3, (uint8_t*)tmode3,
                         sizeof(UBX_CFG_TMODE3_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Start survey-in mode for RTK base station
 *  @param  minDurSec Minimum survey-in duration in seconds
 *  @param  accLimitMm Required accuracy in 0.1 mm (e.g. 100000 = 10m)
 *  @return True if acknowledged
 *  @note   Only available on High Precision GNSS products
 */
bool Adafruit_UBX::startSurveyIn(uint32_t minDurSec, uint32_t accLimitMm) {
  UBX_CFG_TMODE3_t tmode3;
  memset(&tmode3, 0, sizeof(tmode3));
  tmode3.version = 0;
  tmode3.flags = UBX_TMODE3_MODE_SURVEY_IN;
  tmode3.svinMinDur = minDurSec;
  tmode3.svinAccLimit = accLimitMm;
  return setCfgTmode3(&tmode3);
}

/*!
 *  @brief  Poll CFG-GEOFENCE message (geofence configuration)
 *  @param  header Pointer to header struct to fill
 *  @param  fences Array of fence definition structs to fill
 *  @param  maxFences Maximum number of fences the array can hold
 *  @param  timeout_ms Timeout in milliseconds
 *  @return Number of fences read, or 0 on failure
 */
uint8_t Adafruit_UBX::pollCfgGeofence(UBX_CFG_GEOFENCE_header_t* header,
                                      UBX_CFG_GEOFENCE_fence_t* fences,
                                      uint8_t maxFences, uint16_t timeout_ms) {
  if (!sendMessage(UBX_CLASS_CFG, UBX_CFG_GEOFENCE, NULL, 0)) {
    return 0;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_CFG && _lastMsgId == UBX_CFG_GEOFENCE) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        if (usablePayload < sizeof(UBX_CFG_GEOFENCE_header_t)) {
          return 0;
        }

        memcpy(header, _buffer + 6, sizeof(UBX_CFG_GEOFENCE_header_t));

        uint16_t fenceBytes = usablePayload - sizeof(UBX_CFG_GEOFENCE_header_t);
        uint8_t fencesInPayload = fenceBytes / sizeof(UBX_CFG_GEOFENCE_fence_t);
        uint8_t fencesToRead = min(fencesInPayload, maxFences);
        fencesToRead = min(fencesToRead, header->numFences);

        for (uint8_t i = 0; i < fencesToRead; i++) {
          memcpy(&fences[i],
                 _buffer + 6 + sizeof(UBX_CFG_GEOFENCE_header_t) +
                     (i * sizeof(UBX_CFG_GEOFENCE_fence_t)),
                 sizeof(UBX_CFG_GEOFENCE_fence_t));
        }
        return fencesToRead;
      }
    }
    delay(1);
  }
  return 0;
}

/*!
 *  @brief  Set CFG-GEOFENCE message (geofence configuration)
 *  @param  header Pointer to header struct
 *  @param  fences Array of fence definition structs
 *  @param  numFences Number of fences to set (max 4)
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgGeofence(UBX_CFG_GEOFENCE_header_t* header,
                                  UBX_CFG_GEOFENCE_fence_t* fences,
                                  uint8_t numFences) {
  if (numFences > 4)
    numFences = 4;

  uint16_t totalLen = sizeof(UBX_CFG_GEOFENCE_header_t) +
                      (numFences * sizeof(UBX_CFG_GEOFENCE_fence_t));
  uint8_t payload[totalLen];

  header->numFences = numFences;
  memcpy(payload, header, sizeof(UBX_CFG_GEOFENCE_header_t));
  memcpy(payload + sizeof(UBX_CFG_GEOFENCE_header_t), fences,
         numFences * sizeof(UBX_CFG_GEOFENCE_fence_t));

  UBXSendStatus status =
      sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_GEOFENCE, payload, totalLen);
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Set a single circular geofence (convenience method)
 *  @param  lat Latitude of center (deg * 1e-7)
 *  @param  lon Longitude of center (deg * 1e-7)
 *  @param  radiusCm Radius in centimeters
 *  @param  confLvl Confidence level: 0=none, 1=68%, 2=95%, 3=99.7%
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setGeofence(int32_t lat, int32_t lon, uint32_t radiusCm,
                               uint8_t confLvl) {
  UBX_CFG_GEOFENCE_header_t header;
  memset(&header, 0, sizeof(header));
  header.version = 0;
  header.numFences = 1;
  header.confLvl = confLvl;
  header.pioEnabled = 0;
  header.pinPolarity = 0;
  header.pin = 0;

  UBX_CFG_GEOFENCE_fence_t fence;
  fence.lat = lat;
  fence.lon = lon;
  fence.radius = radiusCm;

  return setCfgGeofence(&header, &fence, 1);
}

/*!
 *  @brief  Clear all geofences
 *  @return True if acknowledged
 */
bool Adafruit_UBX::clearGeofence() {
  UBX_CFG_GEOFENCE_header_t header;
  memset(&header, 0, sizeof(header));
  header.version = 0;
  header.numFences = 0;

  return setCfgGeofence(&header, NULL, 0);
}

/*!
 *  @brief  Poll CFG-TP5 message (time pulse configuration)
 *  @param  tp5 Pointer to struct to fill
 *  @param  tpIdx Time pulse index (0 or 1)
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollCfgTp5(UBX_CFG_TP5_t* tp5, uint8_t tpIdx,
                              uint16_t timeout_ms) {
  // Poll with tpIdx as 1-byte payload
  if (!sendMessage(UBX_CLASS_CFG, UBX_CFG_TP5, &tpIdx, 1)) {
    return false;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_CFG && _lastMsgId == UBX_CFG_TP5) {
        uint16_t usablePayload = min(_lastPayloadLength, MAX_PAYLOAD_SIZE);
        uint16_t copyLen = min((uint16_t)sizeof(UBX_CFG_TP5_t), usablePayload);
        memcpy(tp5, _buffer + 6, copyLen);
        return true;
      }
    }
    delay(1);
  }
  return false;
}

/*!
 *  @brief  Set CFG-TP5 message (time pulse configuration)
 *  @param  tp5 Pointer to struct with settings to apply
 *  @return True if acknowledged
 */
bool Adafruit_UBX::setCfgTp5(UBX_CFG_TP5_t* tp5) {
  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_CFG, UBX_CFG_TP5, (uint8_t*)tp5, sizeof(UBX_CFG_TP5_t));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Backup receiver state to flash (save on shutdown)
 *  @return True if command was sent successfully
 *  @note   This creates a backup of ephemeris, almanac, and receiver state.
 *          The backup is restored automatically on power-up.
 */
bool Adafruit_UBX::backupToFlash() {
  UBX_UPD_SOS_cmd_t cmd;
  cmd.cmd = UBX_UPD_SOS_CMD_CREATE;
  memset(cmd.reserved1, 0, sizeof(cmd.reserved1));

  UBXSendStatus status = sendMessageWithAck(UBX_CLASS_UPD, UBX_UPD_SOS,
                                            (uint8_t*)&cmd, sizeof(cmd));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Clear backup from flash
 *  @return True if command was sent successfully
 */
bool Adafruit_UBX::clearBackup() {
  UBX_UPD_SOS_cmd_t cmd;
  cmd.cmd = UBX_UPD_SOS_CMD_CLEAR;
  memset(cmd.reserved1, 0, sizeof(cmd.reserved1));

  UBXSendStatus status = sendMessageWithAck(UBX_CLASS_UPD, UBX_UPD_SOS,
                                            (uint8_t*)&cmd, sizeof(cmd));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Poll UPD-SOS to get backup status
 *  @param  response Pointer to struct to fill with response
 *  @param  timeout_ms Timeout in milliseconds
 *  @return Response code (0=unknown, 1=failed, 2=restored, 3=none), or 0xFF on
 * error
 */
uint8_t Adafruit_UBX::pollUpdSos(UBX_UPD_SOS_response_t* response,
                                 uint16_t timeout_ms) {
  if (!sendMessage(UBX_CLASS_UPD, UBX_UPD_SOS, NULL, 0)) {
    return 0xFF;
  }

  uint32_t startTime = millis();
  while (millis() - startTime < timeout_ms) {
    if (checkMessages()) {
      if (_lastMsgClass == UBX_CLASS_UPD && _lastMsgId == UBX_UPD_SOS) {
        if (_lastPayloadLength >= sizeof(UBX_UPD_SOS_response_t)) {
          memcpy(response, _buffer + 6, sizeof(UBX_UPD_SOS_response_t));
          return response->response;
        }
      }
    }
    delay(1);
  }
  return 0xFF;
}

/*!
 *  @brief  Create a log file
 *  @param  logSize Size option: 0=max safe, 1=minimum, 2=user-defined
 *  @param  circular True for circular logging (overwrite oldest entries)
 *  @param  userSize User-defined max size in bytes (only if logSize=2)
 *  @return True if acknowledged
 */
bool Adafruit_UBX::createLog(uint8_t logSize, bool circular,
                             uint32_t userSize) {
  UBX_LOG_CREATE_t create;
  create.version = 0;
  create.logCfg = circular ? UBX_LOG_CREATE_CIRCULAR : 0;
  create.reserved1 = 0;
  create.logSize = logSize;
  create.userDefinedSize = userSize;

  UBXSendStatus status = sendMessageWithAck(UBX_CLASS_LOG, UBX_LOG_CREATE,
                                            (uint8_t*)&create, sizeof(create));
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Erase the log file
 *  @return True if acknowledged
 *  @note   This also deactivates logging
 */
bool Adafruit_UBX::eraseLog() {
  // LOG-ERASE has no payload
  UBXSendStatus status =
      sendMessageWithAck(UBX_CLASS_LOG, UBX_LOG_ERASE, NULL, 0);
  return (status == UBX_SEND_SUCCESS);
}

/*!
 *  @brief  Poll LOG-INFO message (log information)
 *  @param  info Pointer to struct to fill
 *  @param  timeout_ms Timeout in milliseconds
 *  @return True if response received
 */
bool Adafruit_UBX::pollLogInfo(UBX_LOG_INFO_t* info, uint16_t timeout_ms) {
  return poll(UBX_CLASS_LOG, UBX_LOG_INFO, info, sizeof(UBX_LOG_INFO_t),
              timeout_ms);
}

/*!
 *  @brief  Get log information (convenience wrapper)
 *  @param  info Pointer to struct to fill
 *  @return True if response received
 */
bool Adafruit_UBX::getLogInfo(UBX_LOG_INFO_t* info) {
  return pollLogInfo(info, 1000);
}

/*!
 *  @brief  Send LOG-RETRIEVE command to start log retrieval
 *  @param  startIndex Index of first entry to retrieve (0-based)
 *  @param  count Number of entries to retrieve (max 256)
 *  @return True if command was acknowledged
 *  @note   After calling this, monitor for LOG-RETRIEVEPOS, LOG-RETRIEVESTRING,
 *          and LOG-RETRIEVEPOSEXTRA messages using checkMessages().
 */
bool Adafruit_UBX::sendLogRetrieve(uint32_t startIndex, uint32_t count) {
  if (count > 256)
    count = 256;

  UBX_LOG_RETRIEVE_t retrieve;
  retrieve.startNumber = startIndex;
  retrieve.entryCount = count;
  retrieve.version = 0;
  memset(retrieve.reserved1, 0, sizeof(retrieve.reserved1));

  UBXSendStatus status = sendMessageWithAck(
      UBX_CLASS_LOG, UBX_LOG_RETRIEVE, (uint8_t*)&retrieve, sizeof(retrieve));
  return (status == UBX_SEND_SUCCESS);
}

// =====================================================================
