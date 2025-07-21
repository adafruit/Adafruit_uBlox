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
Adafruit_UBX::Adafruit_UBX(Stream &stream) {
  _stream = &stream;
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
}

/*!
 *  @brief  Calculate UBX checksum according to protocol
 *  @param  buffer Pointer to the data buffer
 *  @param  len Length of data to checksum
 *  @param  checksumA Reference to store the first checksum byte
 *  @param  checksumB Reference to store the second checksum byte
 */
void Adafruit_UBX::calculateChecksum(uint8_t *buffer, uint16_t len,
                                     uint8_t &checksumA, uint8_t &checksumB) {
  checksumA = 0;
  checksumB = 0;

  for (uint16_t i = 0; i < len; i++) {
    checksumA += buffer[i];
    checksumB += checksumA;
  }
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
      _parserState = GET_ID;
      break;

    case GET_ID:
      _msgId = incomingByte;
      _buffer[3] = incomingByte; // Store for checksum calculation
      _parserState = GET_LENGTH_1;
      break;

    case GET_LENGTH_1:
      _payloadLength = incomingByte;
      _buffer[4] = incomingByte; // Store for checksum calculation
      _parserState = GET_LENGTH_2;
      break;

    case GET_LENGTH_2:
      _payloadLength |= (incomingByte << 8);
      _buffer[5] = incomingByte; // Store for checksum calculation

      if (_payloadLength > MAX_PAYLOAD_SIZE) {
        resetParser(); // Payload too large, reset
      } else {
        _payloadCounter = 0;
        _parserState = GET_PAYLOAD;
      }
      break;

    case GET_PAYLOAD:
      if (_payloadCounter < _payloadLength) {
        _buffer[6 + _payloadCounter] = incomingByte;
        _payloadCounter++;

        if (_payloadCounter == _payloadLength) {
          _parserState = GET_CHECKSUM_A;
        }
      }
      break;

    case GET_CHECKSUM_A:
      // Calculate expected checksum
      calculateChecksum(_buffer + 2, _payloadLength + 4, _checksumA,
                        _checksumB);

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
          Serial.print("UBX RX: ");

          // Print header (sync chars, class, id, length)
          Serial.print("HDR[B5 62 ");
          if (_msgClass < 0x10)
            Serial.print("0");
          Serial.print(_msgClass, HEX);
          Serial.print(" ");
          if (_msgId < 0x10)
            Serial.print("0");
          Serial.print(_msgId, HEX);
          Serial.print(" ");

          uint8_t lenLSB = _payloadLength & 0xFF;
          uint8_t lenMSB = (_payloadLength >> 8) & 0xFF;
          if (lenLSB < 0x10)
            Serial.print("0");
          Serial.print(lenLSB, HEX);
          Serial.print(" ");
          if (lenMSB < 0x10)
            Serial.print("0");
          Serial.print(lenMSB, HEX);
          Serial.print("] ");

          // Print payload if verbose debug is enabled
          if (verbose_debug > 1 && _payloadLength > 0) {
            Serial.print("PL[");
            for (uint16_t i = 0; i < _payloadLength; i++) {
              if (_buffer[6 + i] < 0x10)
                Serial.print("0");
              Serial.print(_buffer[6 + i], HEX);
              Serial.print(" ");
            }
            Serial.print("] ");
          }

          // Print checksum
          Serial.print("CS[");
          if (_checksumA < 0x10)
            Serial.print("0");
          Serial.print(_checksumA, HEX);
          Serial.print(" ");
          if (_checksumB < 0x10)
            Serial.print("0");
          Serial.print(_checksumB, HEX);
          Serial.println("]");
        }
      }

      resetParser(); // Reset for next message
      break;
    }
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
                                               uint8_t *payload,
                                               uint16_t length,
                                               uint16_t timeout_ms = 500) {
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
              if (msgClass < 0x10)
                Serial.print("0");
              Serial.print(msgClass, HEX);
              Serial.print(" ID 0x");
              if (msgId < 0x10)
                Serial.print("0");
              Serial.println(msgId, HEX);
            }
            return UBX_SEND_SUCCESS;
          }
        } else if (_lastMsgId == UBX_ACK_NAK && _lastPayloadLength >= 2) {
          // ACK-NAK message
          if (_lastPayload[0] == msgClass && _lastPayload[1] == msgId) {
            if (verbose_debug > 0) {
              Serial.print(F("UBX ACK: NAK for message class 0x"));
              if (msgClass < 0x10)
                Serial.print("0");
              Serial.print(msgClass, HEX);
              Serial.print(" ID 0x");
              if (msgId < 0x10)
                Serial.print("0");
              Serial.println(msgId, HEX);
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
    if (msgClass < 0x10)
      Serial.print("0");
    Serial.print(msgClass, HEX);
    Serial.print(" ID 0x");
    if (msgId < 0x10)
      Serial.print("0");
    Serial.println(msgId, HEX);
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
                               uint8_t *payload, uint16_t length) {
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
    Serial.print("UBX TX: ");

    // Print header (sync chars, class, id, length)
    Serial.print("HDR[");
    for (int i = 0; i < 6; i++) {
      if (msgBuffer[i] < 0x10)
        Serial.print("0");
      Serial.print(msgBuffer[i], HEX);
      Serial.print(" ");
    }
    Serial.print("] ");

    // Print payload if verbose debug is enabled
    if (verbose_debug > 1 && length > 0) {
      Serial.print("PL[");
      for (uint16_t i = 0; i < length; i++) {
        if (msgBuffer[6 + i] < 0x10)
          Serial.print("0");
        Serial.print(msgBuffer[6 + i], HEX);
        Serial.print(" ");
      }
      Serial.print("] ");
    }

    // Print checksum
    Serial.print("CS[");
    if (checksumA < 0x10)
      Serial.print("0");
    Serial.print(checksumA, HEX);
    Serial.print(" ");
    if (checksumB < 0x10)
      Serial.print("0");
    Serial.print(checksumB, HEX);
    Serial.println("]");
  }

  // Send the message
  size_t written = _stream->write(msgBuffer, length + 8);

  return (written == length + 8);
}
