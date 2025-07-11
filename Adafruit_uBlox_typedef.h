/*!
 * @file Adafruit_uBlox_typedef.h
 * 
 * Type definitions for u-blox GPS/RTK module messages
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Your Name for Project Name.
 * 
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef ADAFRUIT_UBLOX_TYPEDEF_H
#define ADAFRUIT_UBLOX_TYPEDEF_H

#include <Arduino.h>

// UBX Message Classes
typedef enum {
  UBX_CLASS_NAV = 0x01,   // Navigation Results
  UBX_CLASS_RXM = 0x02,   // Receiver Manager Messages
  UBX_CLASS_INF = 0x04,   // Information Messages
  UBX_CLASS_ACK = 0x05,   // Acknowledgements
  UBX_CLASS_CFG = 0x06,   // Configuration
  UBX_CLASS_UPD = 0x09,   // Firmware Update
  UBX_CLASS_MON = 0x0A,   // Monitoring
  UBX_CLASS_AID = 0x0B,   // AssistNow Aiding
  UBX_CLASS_TIM = 0x0D,   // Timing
  UBX_CLASS_ESF = 0x10,   // External Sensor Fusion
  UBX_CLASS_MGA = 0x13,   // Multiple GNSS Assistance
  UBX_CLASS_LOG = 0x21,   // Logging
  UBX_CLASS_SEC = 0x27,   // Security
  UBX_CLASS_HNR = 0x28,   // High Rate Navigation
  UBX_CLASS_NMEA = 0xF0   // NMEA Standard Messages
} UBXMessageClass;

// UBX CFG Message IDs
typedef enum {
  UBX_CFG_PRT = 0x00,     // Port Configuration
  UBX_CFG_MSG = 0x01,     // Message Configuration
  UBX_CFG_RST = 0x04,     // Reset Receiver
  UBX_CFG_RATE = 0x08,    // Navigation/Measurement Rate Settings
  UBX_CFG_CFG = 0x09,     // Clear, Save, and Load Configurations
  UBX_CFG_NAVX5 = 0x23,   // Navigation Engine Settings
  UBX_CFG_GNSS = 0x3E,    // GNSS Configuration
  UBX_CFG_PMS = 0x86      // Power Mode Setup
} UBXCfgMessageId;

// Return values for functions that wait for acknowledgment
typedef enum {
  UBX_SEND_SUCCESS = 0,  // Message was acknowledged (ACK)
  UBX_SEND_NAK,          // Message was not acknowledged (NAK)
  UBX_SEND_FAIL,         // Failed to send the message
  UBX_SEND_TIMEOUT       // Timed out waiting for ACK/NAK
} UBXSendStatus;

// Port ID enum for different interfaces
typedef enum {
  UBX_PORT_DDC = 0,   // I2C / DDC port
  UBX_PORT_UART1 = 1, // UART1 port
  UBX_PORT_UART2 = 2, // UART2 port
  UBX_PORT_USB = 3,   // USB port
  UBX_PORT_SPI = 4    // SPI port
} UBXPortId;

// UART mode flags (Charlen, Parity & Stop bit settings)
typedef enum {
  UBX_UART_MODE_8N1 = 0x000, // 8-bit, no parity, 1 stop bit
  UBX_UART_MODE_8E1 = 0x100, // 8-bit, even parity, 1 stop bit
  UBX_UART_MODE_8O1 = 0x200, // 8-bit, odd parity, 1 stop bit
  UBX_UART_MODE_7N1 = 0x400, // 7-bit, no parity, 1 stop bit
  UBX_UART_MODE_7E1 = 0x500, // 7-bit, even parity, 1 stop bit
  UBX_UART_MODE_7O1 = 0x600, // 7-bit, odd parity, 1 stop bit
  UBX_UART_MODE_7N2 = 0x800, // 7-bit, no parity, 2 stop bits
  UBX_UART_MODE_7E2 = 0x900, // 7-bit, even parity, 2 stop bits
  UBX_UART_MODE_7O2 = 0xA00, // 7-bit, odd parity, 2 stop bits
  UBX_UART_MODE_8N2 = 0xC00, // 8-bit, no parity, 2 stop bits
  UBX_UART_MODE_8E2 = 0xD00, // 8-bit, even parity, 2 stop bits
  UBX_UART_MODE_8O2 = 0xE00  // 8-bit, odd parity, 2 stop bits
} UBXUARTMode;

// Protocol flags for inProtoMask and outProtoMask
#define UBX_PROTOCOL_UBX   0x0001 // UBX protocol
#define UBX_PROTOCOL_NMEA  0x0002 // NMEA protocol
#define UBX_PROTOCOL_RTCM  0x0004 // RTCM2 protocol (only for inProtoMask)
#define UBX_PROTOCOL_RTCM3 0x0020 // RTCM3 protocol

// CFG-PRT (Port Configuration) Message
// Total size: 20 bytes
typedef union {
  struct {
    uint8_t  portID;       // Port identifier (0=DDC/I2C, 1=UART1, 2=UART2, 3=USB, 4=SPI)
    uint8_t  reserved1;    // Reserved
    uint16_t txReady;      // TX ready PIN configuration
    uint32_t mode;         // UART mode (bit field) or Reserved for non-UART ports
    uint32_t baudRate;     // Baudrate in bits/second (UART only)
    uint16_t inProtoMask;  // Input protocol mask
    uint16_t outProtoMask; // Output protocol mask
    uint16_t flags;        // Flags bit field
    uint16_t reserved2;    // Reserved
  } fields;
  uint8_t raw[20];
} UBX_CFG_PRT_t;

#endif // ADAFRUIT_UBLOX_TYPEDEF_H
