/*!
 * @file Adafruit_uBlox_typedef.h
 *
 * Type definitions for u-blox GPS/RTK module messages
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef ADAFRUIT_UBLOX_TYPEDEF_H
#define ADAFRUIT_UBLOX_TYPEDEF_H

#include <Arduino.h>

/** UBX protocol message class identifiers. */
typedef enum {
  UBX_CLASS_NAV = 0x01, // Navigation Results
  UBX_CLASS_RXM = 0x02, // Receiver Manager Messages
  UBX_CLASS_INF = 0x04, // Information Messages
  UBX_CLASS_ACK = 0x05, // Acknowledgements
  UBX_CLASS_CFG = 0x06, // Configuration
  UBX_CLASS_UPD = 0x09, // Firmware Update
  UBX_CLASS_MON = 0x0A, // Monitoring
  UBX_CLASS_AID = 0x0B, // AssistNow Aiding
  UBX_CLASS_TIM = 0x0D, // Timing
  UBX_CLASS_ESF = 0x10, // External Sensor Fusion
  UBX_CLASS_MGA = 0x13, // Multiple GNSS Assistance
  UBX_CLASS_LOG = 0x21, // Logging
  UBX_CLASS_SEC = 0x27, // Security
  UBX_CLASS_HNR = 0x28, // High Rate Navigation
  UBX_CLASS_NMEA = 0xF0 // NMEA Standard Messages
} UBXMessageClass;

/** UBX CFG Message IDs. */
typedef enum {
  UBX_CFG_PRT = 0x00,   // Port Configuration
  UBX_CFG_MSG = 0x01,   // Message Configuration
  UBX_CFG_RST = 0x04,   // Reset Receiver
  UBX_CFG_RATE = 0x08,  // Navigation/Measurement Rate Settings
  UBX_CFG_CFG = 0x09,   // Clear, Save, and Load Configurations
  UBX_CFG_NAVX5 = 0x23, // Navigation Engine Settings
  UBX_CFG_GNSS = 0x3E,  // GNSS Configuration
  UBX_CFG_PMS = 0x86    // Power Mode Setup
} UBXCfgMessageId;

/** UBX NAV Message IDs. */
typedef enum {
  UBX_NAV_POSECEF = 0x01,   // ECEF Position
  UBX_NAV_POSLLH = 0x02,    // Lat/Lon/Height Position
  UBX_NAV_STATUS = 0x03,    // Receiver Navigation Status
  UBX_NAV_DOP = 0x04,       // Dilution of Precision
  UBX_NAV_PVT = 0x07,       // Position Velocity Time
  UBX_NAV_ODO = 0x09,       // Odometer
  UBX_NAV_RESETODO = 0x10,  // Reset Odometer
  UBX_NAV_VELECEF = 0x11,   // ECEF Velocity
  UBX_NAV_VELNED = 0x12,    // NED Velocity
  UBX_NAV_HPPOSECEF = 0x13, // High-Precision ECEF Position
  UBX_NAV_HPPOSLLH = 0x14,  // High-Precision Lat/Lon/Height
  UBX_NAV_TIMEGPS = 0x20,   // GPS Time
  UBX_NAV_TIMEUTC = 0x21,   // UTC Time
  UBX_NAV_CLOCK = 0x22,     // Clock Solution
  UBX_NAV_SAT = 0x35,       // Satellite Information
  UBX_NAV_EOE = 0x61        // End of Epoch
} UBXNavMessageId;

/**
 * UBX-NAV-PVT (0x01 0x07) - Navigation Position Velocity Time Solution.
 * 92 bytes. This is the primary navigation message combining position,
 * velocity, and time in a single message.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW;        ///< GPS time of week (ms)
  uint16_t year;        ///< Year (UTC)
  uint8_t month;        ///< Month 1..12 (UTC)
  uint8_t day;          ///< Day 1..31 (UTC)
  uint8_t hour;         ///< Hour 0..23 (UTC)
  uint8_t min;          ///< Minute 0..59 (UTC)
  uint8_t sec;          ///< Second 0..60 (UTC)
  uint8_t valid;        ///< Validity flags (bit0=validDate, bit1=validTime,
                        ///< bit2=fullyResolved, bit3=validMag)
  uint32_t tAcc;        ///< Time accuracy estimate (ns)
  int32_t nano;         ///< Fraction of second -1e9..1e9 (ns)
  uint8_t fixType;      ///< Fix type: 0=none, 1=DR, 2=2D, 3=3D, 4=GNSS+DR,
                        ///< 5=time only
  uint8_t flags;        ///< Fix flags (bit0=gnssFixOK, bit1=diffSoln,
                        ///< bit5=headVehValid)
  uint8_t flags2;       ///< Additional flags (bits0-4=psmState/spoofDet,
                        ///< bits6-7=carrSoln)
  uint8_t numSV;        ///< Number of satellites used
  int32_t lon;          ///< Longitude (deg * 1e-7)
  int32_t lat;          ///< Latitude (deg * 1e-7)
  int32_t height;       ///< Height above ellipsoid (mm)
  int32_t hMSL;         ///< Height above mean sea level (mm)
  uint32_t hAcc;        ///< Horizontal accuracy estimate (mm)
  uint32_t vAcc;        ///< Vertical accuracy estimate (mm)
  int32_t velN;         ///< NED north velocity (mm/s)
  int32_t velE;         ///< NED east velocity (mm/s)
  int32_t velD;         ///< NED down velocity (mm/s)
  int32_t gSpeed;       ///< Ground speed 2-D (mm/s)
  int32_t headMot;      ///< Heading of motion (deg * 1e-5)
  uint32_t sAcc;        ///< Speed accuracy estimate (mm/s)
  uint32_t headAcc;     ///< Heading accuracy estimate (deg * 1e-5)
  uint16_t pDOP;        ///< Position DOP (scale 0.01)
  uint16_t flags3;      ///< Additional flags (bit0=invalidLlh)
  uint8_t reserved1[4]; ///< Reserved
  int32_t headVeh;      ///< Heading of vehicle (deg * 1e-5)
  int16_t magDec;       ///< Magnetic declination (deg * 1e-2)
  uint16_t magAcc;      ///< Magnetic declination accuracy (deg * 1e-2)
} UBX_NAV_PVT_t;

#if __cplusplus >= 201103L
static_assert(sizeof(UBX_NAV_PVT_t) == 92, "UBX_NAV_PVT_t must be 92 bytes");
#endif

/** Return values for functions that wait for acknowledgment. */
typedef enum {
  UBX_SEND_SUCCESS = 0, // Message was acknowledged (ACK)
  UBX_SEND_NAK,         // Message was not acknowledged (NAK)
  UBX_SEND_FAIL,        // Failed to send the message
  UBX_SEND_TIMEOUT      // Timed out waiting for ACK/NAK
} UBXSendStatus;

/** Port ID enum for different interfaces. */
typedef enum {
  UBX_PORT_DDC = 0,   // I2C / DDC port
  UBX_PORT_UART1 = 1, // UART1 port
  UBX_PORT_UART2 = 2, // UART2 port
  UBX_PORT_USB = 3,   // USB port
  UBX_PORT_SPI = 4    // SPI port
} UBXPortId;

/** UART mode flags (Charlen, Parity & Stop bit settings). */
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
#define UBX_PROTOCOL_UBX 0x0001   ///< UBX protocol
#define UBX_PROTOCOL_NMEA 0x0002  ///< NMEA protocol
#define UBX_PROTOCOL_RTCM 0x0004  ///< RTCM2 protocol (only for inProtoMask)
#define UBX_PROTOCOL_RTCM3 0x0020 ///< RTCM3 protocol

/** UBX CFG-PRT (Port Configuration) message structure. 20 bytes total.
 */
typedef union {
  struct {
    uint8_t
        portID; ///< Port identifier (0=DDC/I2C, 1=UART1, 2=UART2, 3=USB, 4=SPI)
    uint8_t reserved1; ///< Reserved
    uint16_t txReady;  ///< TX ready PIN configuration
    uint32_t mode;     ///< UART mode (bit field) or Reserved for non-UART ports
    uint32_t baudRate; ///< Baudrate in bits/second (UART only)
    uint16_t inProtoMask;  ///< Input protocol mask
    uint16_t outProtoMask; ///< Output protocol mask
    uint16_t flags;        ///< Flags bit field
    uint16_t reserved2;    ///< Reserved
  } fields;                ///< Fields for CFG-PRT message
  uint8_t raw[20];         ///< Raw byte array for CFG-PRT message
} UBX_CFG_PRT_t;

#endif // ADAFRUIT_UBLOX_TYPEDEF_H
