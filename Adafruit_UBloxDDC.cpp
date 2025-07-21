/*!
 * @file Adafruit_UBloxDDC.cpp
 * 
 * @mainpage Arduino library for u-blox GPS/RTK modules over DDC (I2C)
 * 
 * @section intro_sec Introduction
 * 
 * This is a library for the u-blox GPS/RTK modules using I2C interface (DDC)
 * 
 * Designed specifically to work with u-blox GPS/RTK modules 
 * like NEO-M8P, ZED-F9P, etc.
 * 
 * @section dependencies Dependencies
 * 
 * This library depends on:
 * <a href="https://github.com/adafruit/Adafruit_BusIO">Adafruit_BusIO</a>
 * 
 * @section author Author
 * 
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * 
 * @section license License
 * 
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_UBloxDDC.h"

/*!
 *  @brief  Constructor
 *  @param  address
 *          i2c address (default 0x42)
 *  @param  wire
 *          TwoWire instance (default &Wire)
 */
Adafruit_UBloxDDC::Adafruit_UBloxDDC(uint8_t address, TwoWire *wire) {
  _i2cDevice = new Adafruit_I2CDevice(address, wire);
}

/*!
 *  @brief  Destructor - frees allocated resources
 */
Adafruit_UBloxDDC::~Adafruit_UBloxDDC() {
  if (_i2cDevice) {
    delete _i2cDevice;
  }
}

/*!
 *  @brief  Initializes the GPS module and I2C interface
 *  @return True if GPS module responds, false on any failure
 */
bool Adafruit_UBloxDDC::begin() {
  return _i2cDevice->begin();
}

/*!
 *  @brief  Gets the number of bytes available for reading
 *  @return Number of bytes available, or 0 if no data or error
 */
int Adafruit_UBloxDDC::available() {
  uint8_t buffer[2];
  
  // Create a register for reading bytes available
  Adafruit_BusIO_Register bytesAvailableReg = Adafruit_BusIO_Register(_i2cDevice, REG_BYTES_AVAILABLE_MSB, 2);
  
  if (!bytesAvailableReg.read(buffer, 2)) {
    return 0;
  }
  
  uint16_t bytesAvailable = (uint16_t)buffer[0] << 8;
  bytesAvailable |= buffer[1];
  
  return bytesAvailable;
}

/*!
 *  @brief  Reads a single byte from the data stream
 *  @return -1 if no data available or error, otherwise the byte read (0-255)
 */
int Adafruit_UBloxDDC::read() {
  // If we have a peeked byte, return it
  if (_hasPeeked) {
    _hasPeeked = false;
    return _lastByte;
  }
  
  uint8_t value;
  
  // Create a register for the data stream
  Adafruit_BusIO_Register dataStreamReg = Adafruit_BusIO_Register(_i2cDevice, REG_DATA_STREAM, 1);
  
  if (!dataStreamReg.read(&value, 1)) {
    return -1;
  }
  
  return value;
}

/*!
 *  @brief  Peek at the next available byte without removing it from the stream
 *  @return -1 if no data available or error, otherwise the byte (0-255)
 */
int Adafruit_UBloxDDC::peek() {
  // If we've already peeked, return the last byte
  if (_hasPeeked) {
    return _lastByte;
  }
  
  // Otherwise, read a byte and store it
  _lastByte = read();
  if (_lastByte != -1) {
    _hasPeeked = true;
  }
  
  return _lastByte;
}

/*!
 *  @brief  Write a single byte (required by Stream but not suitable for I2C)
 *  @param  val  Byte to write
 *  @return Always returns 0 as single-byte writes aren't supported on I2C
 */
size_t Adafruit_UBloxDDC::write(uint8_t val) {
  // Single-byte writes aren't suitable for I2C/DDC
  // This shouldn't be called if properly using the multi-byte version
  return 0;
}

/*!
 *  @brief  Write multiple bytes at once (required for I2C/DDC)
 *  @param  buffer  Pointer to data buffer
 *  @param  size    Number of bytes to write
 *  @return Number of bytes written
 */
size_t Adafruit_UBloxDDC::write(const uint8_t *buffer, size_t size) {
  // For I2C/DDC, we need at least 2 bytes for a write
  if (size < 2) {
    // Single-byte writes aren't supported
    return 0;
  }
  
  // Use Adafruit_BusIO to handle the I2C transaction
  if (_i2cDevice->write(buffer, size)) {
    return size;
  }
  return 0;
}

/*!
 *  @brief  Read multiple bytes from the data stream
 *  @param  buffer  Pointer to buffer to store data
 *  @param  length  Maximum number of bytes to read
 *  @return Number of bytes actually read, which may be less than requested
 */
uint16_t Adafruit_UBloxDDC::readBytes(uint8_t* buffer, uint16_t length) {
  if (buffer == nullptr || length == 0) {
    return 0;
  }
  
  uint16_t bytesRead = 0;
  uint16_t bytesAvailable = available();
  
  // Don't try to read more bytes than are available
  length = min(length, bytesAvailable);
  
  // Handle any peeked byte first
  if (_hasPeeked && length > 0) {
    buffer[0] = _lastByte;
    _hasPeeked = false;
    bytesRead = 1;
  }
  
  if (bytesRead >= length) {
    return bytesRead;
  }
  
  // Create a register for the data stream
  Adafruit_BusIO_Register dataStreamReg = Adafruit_BusIO_Register(_i2cDevice, REG_DATA_STREAM, 1);
  
  while (bytesRead < length) {
    // Calculate chunk size (I2C has a limit on bytes per transfer)
    uint16_t chunkSize = min(length - bytesRead, (uint16_t)32);
    
    if (!dataStreamReg.read(&buffer[bytesRead], chunkSize)) {
      break;
    }
    
    bytesRead += chunkSize;
  }
  
  return bytesRead;
}

/*!
 *  @brief  Read a complete message from the device
 *  @param  buffer     Pointer to buffer to store message data
 *  @param  maxLength  Maximum length of buffer
 *  @return Number of bytes read into the buffer
 */
uint16_t Adafruit_UBloxDDC::readMessage(uint8_t* buffer, uint16_t maxLength) {
  uint16_t bytesAvailable = available();
  
  if (bytesAvailable == 0) {
    return 0;
  }
  
  // Limit to buffer size
  uint16_t bytesToRead = min(bytesAvailable, maxLength);
  return readBytes(buffer, bytesToRead);
}

/*!
 *  @brief  Read a message into the internal buffer and return a pointer to it
 *  @param  messageLength  Pointer to variable to store message length
 *  @return Pointer to internal buffer containing the message
 */
uint8_t* Adafruit_UBloxDDC::readMessage(uint16_t* messageLength) {
  *messageLength = readMessage(_buffer, MAX_BUFFER_SIZE);
  return _buffer;
}

