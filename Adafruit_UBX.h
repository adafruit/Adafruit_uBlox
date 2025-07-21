/*!
 * @file Adafruit_UBX.h
 *
 * Arduino library for parsing UBX protocol from u-blox GPS/RTK modules.
 *
 * This library can use any Stream object as input (UART, DDC, or other).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef ADAFRUIT_UBX_H
#define ADAFRUIT_UBX_H

#include "Adafruit_uBlox_typedef.h"
#include <Arduino.h>
#include <Stream.h>

// UBX protocol constants
#define UBX_SYNC_CHAR_1 0xB5 // First UBX protocol sync char (�)
#define UBX_SYNC_CHAR_2 0x62 // Second UBX protocol sync char (b)
// UBX ACK Message IDs
#define UBX_ACK_NAK 0x00 // Message Not Acknowledged
#define UBX_ACK_ACK 0x01 // Message Acknowledged

// Callback function type for UBX messages - defined at global scope so other
// classes can use it
typedef void (*UBXMessageCallback)(uint8_t msgClass, uint8_t msgId,
                                   uint16_t payloadLen, uint8_t *payload);

/*!
 * @brief Class for parsing UBX protocol messages from u-blox GPS/RTK modules
 */
class Adafruit_UBX {
public:
  // Constructor
  Adafruit_UBX(Stream &stream);
  uint8_t verbose_debug = 0; // 0=off, 1=basic, 2=verbose

  // Basic methods
  bool begin();

  bool checkMessages(); // Message parsing
  bool sendMessage(uint8_t msgClass, uint8_t msgId, uint8_t *payload,
                   uint16_t length); // Send a UBX message
  UBXSendStatus sendMessageWithAck(uint8_t msgClass, uint8_t msgId,
                                   uint8_t *payload, uint16_t length,
                                   uint16_t timeout_ms = 500);

  // Configure port to use UBX protocol only (disable NMEA)
  UBXSendStatus setUBXOnly(UBXPortId portID, bool checkAck = true,
                           uint16_t timeout_ms = 500);

  void setMessageCallback(UBXMessageCallback callback); // Set callback function
  UBXMessageCallback onUBXMessage; // Callback for message received

private:
  Stream *_stream; // Stream interface for reading data

  // Buffer for reading messages
  static const uint16_t MAX_PAYLOAD_SIZE = 64; // Maximum UBX payload size
  uint8_t _buffer[MAX_PAYLOAD_SIZE +
                  8]; // Buffer for message (header, payload, checksum)

  // Parser state machine
  enum ParserState {
    WAIT_SYNC_1,    // Waiting for first sync char (0xB5)
    WAIT_SYNC_2,    // Waiting for second sync char (0x62)
    GET_CLASS,      // Reading message class
    GET_ID,         // Reading message ID
    GET_LENGTH_1,   // Reading length LSB
    GET_LENGTH_2,   // Reading length MSB
    GET_PAYLOAD,    // Reading payload
    GET_CHECKSUM_A, // Reading checksum A
    GET_CHECKSUM_B  // Reading checksum B
  };

  ParserState _parserState = WAIT_SYNC_1; // Current state of the parser
  uint8_t _msgClass;                      // Message class of current message
  uint8_t _msgId;                         // Message ID of current message
  uint16_t _payloadLength;                // Length of current message payload
  uint16_t _payloadCounter;               // Counter for payload bytes received
  uint8_t _checksumA;                     // Running checksum A
  uint8_t _checksumB;                     // Running checksum B

  // Calculate checksum for a block of data
  void calculateChecksum(uint8_t *buffer, uint16_t len, uint8_t &checksumA,
                         uint8_t &checksumB);

  // Reset parser state
  void resetParser();

  // Add to private section of Adafruit_UBX.h
  uint8_t _lastMsgClass;       // Class of last message
  uint8_t _lastMsgId;          // ID of last message
  uint16_t _lastPayloadLength; // Length of last message payload
  uint8_t _lastPayload[8];     // Buffer for small payloads (like ACK messages)
};

#endif // ADAFRUIT_UBX_H
