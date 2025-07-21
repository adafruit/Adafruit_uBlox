/*!
 * @file Adafruit_UBloxDDC.h
 *
 * Arduino library for interfacing with u-blox GPS/RTK modules over I2C (DDC).
 *
 * This library implements the Stream interface, allowing it to be used with
 * existing GPS parsing libraries like TinyGPS++.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef ADAFRUIT_UBLOXDDC_H
#define ADAFRUIT_UBLOXDDC_H

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Stream.h>

/*!
 * @brief Arduino library for interfacing with u-blox GPS/RTK modules over I2C
 */
class Adafruit_UBloxDDC : public Stream {
private:
  Adafruit_I2CDevice *_i2cDevice; ///< Underlying I2C device

  // Register addresses
  static const uint8_t REG_DATA_STREAM =
      0xFF; ///< Register for reading data stream
  static const uint8_t REG_BYTES_AVAILABLE_MSB =
      0xFD; ///< MSB of bytes available
  static const uint8_t REG_BYTES_AVAILABLE_LSB =
      0xFE; ///< LSB of bytes available

  // Buffer for reading messages
  static const uint16_t MAX_BUFFER_SIZE =
      128;                          ///< Maximum buffer size for messages
  uint8_t _buffer[MAX_BUFFER_SIZE]; ///< Internal buffer for messages

  // Last byte read for peek() implementation
  int _lastByte = -1;      ///< Last byte read by peek()
  bool _hasPeeked = false; ///< Indicates if we have a peeked byte waiting

public:
  // Constructor & destructor
  Adafruit_UBloxDDC(uint8_t address = 0x42, TwoWire *wire = &Wire);
  ~Adafruit_UBloxDDC();

  // Basic methods
  bool begin();

  // Stream interface implementation
  virtual int available() override;
  virtual int read() override;
  virtual int peek() override;
  // Stream interface implementation
  virtual size_t write(uint8_t) override;
  virtual size_t write(const uint8_t *buffer, size_t size) override;

  // Additional methods
  uint16_t readBytes(uint8_t *buffer, uint16_t length);
  uint16_t readMessage(uint8_t *buffer, uint16_t maxLength);
  uint8_t *readMessage(uint16_t *messageLength);
};

#endif // ADAFRUIT_UBLOXDDC_H
