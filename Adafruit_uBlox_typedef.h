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
  UBX_CFG_PRT = 0x00,      // Port Configuration
  UBX_CFG_MSG = 0x01,      // Message Configuration
  UBX_CFG_INF = 0x02,      // Information Message Configuration
  UBX_CFG_RST = 0x04,      // Reset Receiver
  UBX_CFG_RATE = 0x08,     // Navigation/Measurement Rate Settings
  UBX_CFG_CFG = 0x09,      // Clear, Save, and Load Configurations
  UBX_CFG_RXM = 0x11,      // Receiver Manager Configuration
  UBX_CFG_ANT = 0x13,      // Antenna Control Settings
  UBX_CFG_SBAS = 0x16,     // SBAS Configuration
  UBX_CFG_NMEA = 0x17,     // NMEA Protocol Configuration
  UBX_CFG_NAVX5 = 0x23,    // Navigation Engine Expert Settings
  UBX_CFG_NAV5 = 0x24,     // Navigation Engine Settings
  UBX_CFG_TP5 = 0x31,      // Time Pulse Configuration
  UBX_CFG_PM2 = 0x3B,      // Extended Power Management Configuration
  UBX_CFG_GNSS = 0x3E,     // GNSS Configuration
  UBX_CFG_GEOFENCE = 0x69, // Geofencing Configuration
  UBX_CFG_TMODE3 = 0x71,   // Time Mode 3 (RTK Base)
  UBX_CFG_PMS = 0x86       // Power Mode Setup
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
  UBX_NAV_TIMELS = 0x26,    // Leap Second Event
  UBX_NAV_SAT = 0x35,       // Satellite Information
  UBX_NAV_GEOFENCE = 0x39,  // Geofencing Status
  UBX_NAV_SVIN = 0x3B,      // Survey-in Data
  UBX_NAV_RELPOSNED = 0x3C, // Relative Position NED
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

/** UBX-NAV-STATUS (0x01 0x03) - Receiver Navigation Status.
 *  16 bytes. Provides fix type, fix status flags, TTFF, and uptime.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW; ///< GPS time of week (ms)
  uint8_t
      gpsFix;    ///< Fix type: 0=none, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
  uint8_t flags; ///< Fix flags (bit0=gpsFixOk, bit1=diffSoln, bit2=wknSet,
                 ///< bit3=towSet)
  uint8_t fixStat; ///< Fix status (bit0=diffCorr, bit1=carrSolnValid,
                   ///< bits6-7=mapMatching)
  uint8_t flags2;  ///< Additional flags (bits0-2=psmState,
                   ///< bits3-4=spoofDetState, bits6-7=carrSoln)
  uint32_t ttff;   ///< Time to first fix (ms)
  uint32_t msss;   ///< Milliseconds since startup/reset (ms)
} UBX_NAV_STATUS_t;

static_assert(sizeof(UBX_NAV_STATUS_t) == 16,
              "UBX_NAV_STATUS_t must be 16 bytes");

/** UBX-NAV-DOP (0x01 0x04) - Dilution of Precision.
 *  18 bytes. All DOP values scaled by 0.01.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW; ///< GPS time of week (ms)
  uint16_t gDOP; ///< Geometric DOP (scale 0.01)
  uint16_t pDOP; ///< Position DOP (scale 0.01)
  uint16_t tDOP; ///< Time DOP (scale 0.01)
  uint16_t vDOP; ///< Vertical DOP (scale 0.01)
  uint16_t hDOP; ///< Horizontal DOP (scale 0.01)
  uint16_t nDOP; ///< Northing DOP (scale 0.01)
  uint16_t eDOP; ///< Easting DOP (scale 0.01)
} UBX_NAV_DOP_t;

static_assert(sizeof(UBX_NAV_DOP_t) == 18, "UBX_NAV_DOP_t must be 18 bytes");

/** UBX-NAV-SAT (0x01 0x35) - Satellite Information header.
 *  8 bytes fixed, followed by 12 bytes per satellite.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW;        ///< GPS time of week (ms)
  uint8_t version;      ///< Message version (0x01)
  uint8_t numSvs;       ///< Number of satellites
  uint8_t reserved1[2]; ///< Reserved
} UBX_NAV_SAT_header_t;

static_assert(sizeof(UBX_NAV_SAT_header_t) == 8,
              "UBX_NAV_SAT_header_t must be 8 bytes");

/** UBX-NAV-SAT repeated block - per satellite data. 12 bytes each. */
typedef struct __attribute__((packed)) {
  uint8_t gnssId; ///< GNSS identifier (0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou,
                  ///< 5=IMES, 6=GLONASS)
  uint8_t svId;   ///< Satellite identifier
  uint8_t cno;    ///< Carrier-to-noise ratio (dBHz)
  int8_t elev;    ///< Elevation -90..+90 (deg)
  int16_t azim;   ///< Azimuth 0..360 (deg)
  int16_t prRes;  ///< Pseudorange residual (dm, scale 0.1 m)
  uint32_t flags; ///< Bitmask (see protocol spec for details)
} UBX_NAV_SAT_sv_t;

static_assert(sizeof(UBX_NAV_SAT_sv_t) == 12,
              "UBX_NAV_SAT_sv_t must be 12 bytes");

/** UBX-CFG-RST (0x06 0x04) - Reset Receiver.
 *  4 bytes. Send-only command, do NOT expect ACK.
 *  navBbrMask: 0x0000=Hot start, 0x0001=Warm start, 0xFFFF=Cold start
 *  resetMode: 0x00=HW reset, 0x01=SW reset, 0x02=SW reset (GNSS only),
 *             0x04=HW reset after shutdown, 0x08=GNSS stop, 0x09=GNSS start
 */
typedef struct __attribute__((packed)) {
  uint16_t navBbrMask; ///< BBR sections to clear
  uint8_t resetMode;   ///< Reset type
  uint8_t reserved1;   ///< Reserved
} UBX_CFG_RST_t;

static_assert(sizeof(UBX_CFG_RST_t) == 4, "UBX_CFG_RST_t must be 4 bytes");

/** UBX-CFG-RATE (0x06 0x08) - Navigation/Measurement Rate Settings.
 *  6 bytes. Controls measurement and navigation rate.
 */
typedef struct __attribute__((packed)) {
  uint16_t measRate; ///< Measurement rate (ms). 1000=1Hz, 200=5Hz, 100=10Hz
  uint16_t navRate;  ///< Navigation rate (cycles). Ratio of meas to nav
                     ///< solutions. Usually 1.
  uint16_t timeRef;  ///< Time reference: 0=UTC, 1=GPS, 2=GLONASS, 3=BeiDou,
                     ///< 4=Galileo
} UBX_CFG_RATE_t;

static_assert(sizeof(UBX_CFG_RATE_t) == 6, "UBX_CFG_RATE_t must be 6 bytes");

/** UBX-CFG-CFG (0x06 0x09) - Clear, Save and Load Configurations.
 *  12 bytes (or 13 with optional deviceMask).
 *  Mask bits: bit0=ioPort, bit1=msgConf, bit2=infMsg, bit3=navConf,
 *             bit4=rxmConf, bit8=senConf, bit9=rinvConf, bit10=antConf,
 *             bit11=logConf
 */
typedef struct __attribute__((packed)) {
  uint32_t clearMask; ///< Sections to clear (load defaults)
  uint32_t saveMask;  ///< Sections to save to non-volatile memory
  uint32_t loadMask;  ///< Sections to load from non-volatile memory
} UBX_CFG_CFG_t;

static_assert(sizeof(UBX_CFG_CFG_t) == 12, "UBX_CFG_CFG_t must be 12 bytes");

/** UBX RXM Message IDs. */
typedef enum {
  UBX_RXM_PMREQ = 0x41, // Power Management Request
  UBX_RXM_RAWX = 0x15,  // Multi-GNSS Raw Measurements
  UBX_RXM_SFRBX = 0x13, // Broadcast Navigation Data Subframe
  UBX_RXM_SVSI = 0x20   // SV Status Info
} UBXRxmMessageId;

/** UBX MON Message IDs. */
typedef enum {
  UBX_MON_IO = 0x02,    // I/O System Status
  UBX_MON_VER = 0x04,   // Receiver/Software Version
  UBX_MON_MSGPP = 0x06, // Message Parse/Process Status
  UBX_MON_RXBUF = 0x07, // Receiver Buffer Status
  UBX_MON_TXBUF = 0x08, // Transmitter Buffer Status
  UBX_MON_HW = 0x09,    // Hardware Status
  UBX_MON_HW2 = 0x0B,   // Extended Hardware Status
  UBX_MON_GNSS = 0x28   // GNSS System Info
} UBXMonMessageId;

/** UBX-MON-VER (0x0A 0x04) - Receiver/Software Version header.
 *  40 bytes fixed, followed by 30-byte extension strings.
 */
typedef struct __attribute__((packed)) {
  char swVersion[30]; ///< Software version string (nul-terminated)
  char hwVersion[10]; ///< Hardware version string (nul-terminated)
} UBX_MON_VER_header_t;

static_assert(sizeof(UBX_MON_VER_header_t) == 40,
              "UBX_MON_VER_header_t must be 40 bytes");

/** UBX-MON-VER extension block. 30 bytes each. */
typedef struct __attribute__((packed)) {
  char extension[30]; ///< Extended software info (nul-terminated)
} UBX_MON_VER_ext_t;

static_assert(sizeof(UBX_MON_VER_ext_t) == 30,
              "UBX_MON_VER_ext_t must be 30 bytes");

/** Return values for functions that wait for acknowledgment. */
typedef enum {
  UBX_SEND_SUCCESS = 0, // Message was acknowledged (ACK)
  UBX_SEND_NAK,         // Message was not acknowledged (NAK)
  UBX_SEND_FAIL,        // Failed to send the message
  UBX_SEND_TIMEOUT      // Timed out waiting for ACK/NAK
} UBXSendStatus;

/** UBX-NAV-POSLLH (0x01 0x02) - Geodetic Position Solution.
 *  28 bytes. Lat/Lon/Height position.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW;  ///< GPS time of week (ms)
  int32_t lon;    ///< Longitude (deg, scale 1e-7)
  int32_t lat;    ///< Latitude (deg, scale 1e-7)
  int32_t height; ///< Height above ellipsoid (mm)
  int32_t hMSL;   ///< Height above mean sea level (mm)
  uint32_t hAcc;  ///< Horizontal accuracy estimate (mm)
  uint32_t vAcc;  ///< Vertical accuracy estimate (mm)
} UBX_NAV_POSLLH_t;

static_assert(sizeof(UBX_NAV_POSLLH_t) == 28,
              "UBX_NAV_POSLLH_t must be 28 bytes");

/** UBX-NAV-VELNED (0x01 0x12) - Velocity Solution in NED Frame.
 *  36 bytes. North/East/Down velocity components.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW;   ///< GPS time of week (ms)
  int32_t velN;    ///< North velocity component (cm/s)
  int32_t velE;    ///< East velocity component (cm/s)
  int32_t velD;    ///< Down velocity component (cm/s)
  uint32_t speed;  ///< Speed (3-D) (cm/s)
  uint32_t gSpeed; ///< Ground speed (2-D) (cm/s)
  int32_t heading; ///< Heading of motion 2-D (deg, scale 1e-5)
  uint32_t sAcc;   ///< Speed accuracy estimate (cm/s)
  uint32_t cAcc;   ///< Course / Heading accuracy estimate (deg, scale 1e-5)
} UBX_NAV_VELNED_t;

static_assert(sizeof(UBX_NAV_VELNED_t) == 36,
              "UBX_NAV_VELNED_t must be 36 bytes");

/** UBX-NAV-TIMEUTC (0x01 0x21) - UTC Time Solution.
 *  20 bytes. UTC date and time.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW; ///< GPS time of week (ms)
  uint32_t tAcc; ///< Time accuracy estimate (ns)
  int32_t nano;  ///< Fraction of second -1e9..1e9 (ns)
  uint16_t year; ///< Year (UTC)
  uint8_t month; ///< Month 1..12 (UTC)
  uint8_t day;   ///< Day 1..31 (UTC)
  uint8_t hour;  ///< Hour 0..23 (UTC)
  uint8_t min;   ///< Minute 0..59 (UTC)
  uint8_t sec;   ///< Second 0..60 (UTC)
  uint8_t valid; ///< Validity flags (bit0=validTOW, bit1=validWKN,
                 ///< bit2=validUTC, bits6-7=utcStandard)
} UBX_NAV_TIMEUTC_t;

static_assert(sizeof(UBX_NAV_TIMEUTC_t) == 20,
              "UBX_NAV_TIMEUTC_t must be 20 bytes");

/** UBX-NAV-POSECEF (0x01 0x01) - Position Solution in ECEF.
 *  20 bytes. Earth-Centered Earth-Fixed coordinates.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW; ///< GPS time of week (ms)
  int32_t ecefX; ///< ECEF X coordinate (cm)
  int32_t ecefY; ///< ECEF Y coordinate (cm)
  int32_t ecefZ; ///< ECEF Z coordinate (cm)
  uint32_t pAcc; ///< Position accuracy estimate (cm)
} UBX_NAV_POSECEF_t;

static_assert(sizeof(UBX_NAV_POSECEF_t) == 20,
              "UBX_NAV_POSECEF_t must be 20 bytes");

/** UBX-NAV-VELECEF (0x01 0x11) - Velocity Solution in ECEF.
 *  20 bytes. Earth-Centered Earth-Fixed velocity.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW;  ///< GPS time of week (ms)
  int32_t ecefVX; ///< ECEF X velocity (cm/s)
  int32_t ecefVY; ///< ECEF Y velocity (cm/s)
  int32_t ecefVZ; ///< ECEF Z velocity (cm/s)
  uint32_t sAcc;  ///< Speed accuracy estimate (cm/s)
} UBX_NAV_VELECEF_t;

static_assert(sizeof(UBX_NAV_VELECEF_t) == 20,
              "UBX_NAV_VELECEF_t must be 20 bytes");

/** UBX-NAV-CLOCK (0x01 0x22) - Clock Solution.
 *  20 bytes. Receiver clock bias and drift.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW; ///< GPS time of week (ms)
  int32_t clkB;  ///< Clock bias (ns)
  int32_t clkD;  ///< Clock drift (ns/s)
  uint32_t tAcc; ///< Time accuracy estimate (ns)
  uint32_t fAcc; ///< Frequency accuracy estimate (ps/s)
} UBX_NAV_CLOCK_t;

static_assert(sizeof(UBX_NAV_CLOCK_t) == 20,
              "UBX_NAV_CLOCK_t must be 20 bytes");

/** UBX-NAV-EOE (0x01 0x61) - End of Epoch.
 *  4 bytes. Marks end of navigation epoch.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW; ///< GPS time of week (ms)
} UBX_NAV_EOE_t;

static_assert(sizeof(UBX_NAV_EOE_t) == 4, "UBX_NAV_EOE_t must be 4 bytes");

/** UBX-NAV-TIMEGPS (0x01 0x20) - GPS Time Solution.
 *  16 bytes. Precise GPS time.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW; ///< GPS time of week (ms)
  int32_t fTOW;  ///< Fractional part of iTOW +/-500000 (ns)
  int16_t week;  ///< GPS week number
  int8_t leapS;  ///< GPS leap seconds (GPS-UTC) (s)
  uint8_t valid; ///< Validity flags (bit0=towValid, bit1=weekValid,
                 ///< bit2=leapSValid)
  uint32_t tAcc; ///< Time accuracy estimate (ns)
} UBX_NAV_TIMEGPS_t;

static_assert(sizeof(UBX_NAV_TIMEGPS_t) == 16,
              "UBX_NAV_TIMEGPS_t must be 16 bytes");

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

/** UBX-CFG-NAV5 (0x06 0x24) - Navigation Engine Settings.
 *  36 bytes. Controls dynamic model, fix mode, masks, etc.
 *  Dynamic models: 0=portable, 2=stationary, 3=pedestrian, 4=automotive,
 *                  5=sea, 6=airborne1g, 7=airborne2g, 8=airborne4g
 *  Fix modes: 1=2D only, 2=3D only, 3=auto 2D/3D
 */
typedef struct __attribute__((packed)) {
  uint16_t mask;              ///< Parameters bitmask (see UBX_NAV5_MASK_*)
  uint8_t dynModel;           ///< Dynamic platform model
  uint8_t fixMode;            ///< Position fixing mode
  int32_t fixedAlt;           ///< Fixed altitude for 2D mode (m * 0.01)
  uint32_t fixedAltVar;       ///< Fixed altitude variance (m^2 * 0.0001)
  int8_t minElev;             ///< Minimum elevation for satellite (deg)
  uint8_t drLimit;            ///< Reserved
  uint16_t pDop;              ///< Position DOP mask (* 0.1)
  uint16_t tDop;              ///< Time DOP mask (* 0.1)
  uint16_t pAcc;              ///< Position accuracy mask (m)
  uint16_t tAcc;              ///< Time accuracy mask (m)
  uint8_t staticHoldThresh;   ///< Static hold threshold (cm/s)
  uint8_t dgnssTimeout;       ///< DGNSS timeout (s)
  uint8_t cnoThreshNumSVs;    ///< Number of satellites for C/N0 threshold
  uint8_t cnoThresh;          ///< C/N0 threshold (dBHz)
  uint8_t reserved1[2];       ///< Reserved
  uint16_t staticHoldMaxDist; ///< Static hold distance threshold (m)
  uint8_t utcStandard;  ///< UTC standard (0=auto, 3=USNO, 5=EU, 6=SU, 7=NTSC)
  uint8_t reserved2[5]; ///< Reserved
} UBX_CFG_NAV5_t;

static_assert(sizeof(UBX_CFG_NAV5_t) == 36, "UBX_CFG_NAV5_t must be 36 bytes");

// CFG-NAV5 mask bits
#define UBX_NAV5_MASK_DYN 0x0001          ///< Apply dynamic model
#define UBX_NAV5_MASK_MINEL 0x0002        ///< Apply minimum elevation
#define UBX_NAV5_MASK_POSFIX 0x0004       ///< Apply fix mode
#define UBX_NAV5_MASK_POSMASK 0x0010      ///< Apply position mask
#define UBX_NAV5_MASK_TIMEMASK 0x0020     ///< Apply time mask
#define UBX_NAV5_MASK_STATICHOLD 0x0040   ///< Apply static hold settings
#define UBX_NAV5_MASK_DGPS 0x0080         ///< Apply DGPS settings
#define UBX_NAV5_MASK_CNOTHRESHOLD 0x0100 ///< Apply CNO threshold
#define UBX_NAV5_MASK_UTC 0x0200          ///< Apply UTC settings

// CFG-NAV5 dynamic model values
#define UBX_DYNMODEL_PORTABLE 0   ///< Portable
#define UBX_DYNMODEL_STATIONARY 2 ///< Stationary
#define UBX_DYNMODEL_PEDESTRIAN 3 ///< Pedestrian
#define UBX_DYNMODEL_AUTOMOTIVE 4 ///< Automotive
#define UBX_DYNMODEL_SEA 5        ///< Sea
#define UBX_DYNMODEL_AIRBORNE1G 6 ///< Airborne <1g
#define UBX_DYNMODEL_AIRBORNE2G 7 ///< Airborne <2g
#define UBX_DYNMODEL_AIRBORNE4G 8 ///< Airborne <4g

/** UBX-CFG-GNSS (0x06 0x3E) - GNSS Configuration header.
 *  4 bytes fixed header, followed by 8 bytes per config block.
 */
typedef struct __attribute__((packed)) {
  uint8_t msgVer;          ///< Message version (0x00)
  uint8_t numTrkChHw;      ///< Number of tracking channels in hardware (ro)
  uint8_t numTrkChUse;     ///< Number of tracking channels to use
  uint8_t numConfigBlocks; ///< Number of config blocks following
} UBX_CFG_GNSS_header_t;

static_assert(sizeof(UBX_CFG_GNSS_header_t) == 4,
              "UBX_CFG_GNSS_header_t must be 4 bytes");

/** UBX-CFG-GNSS config block. 8 bytes per GNSS system. */
typedef struct __attribute__((packed)) {
  uint8_t gnssId;    ///< GNSS identifier (0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou,
                     ///< 5=QZSS, 6=GLONASS)
  uint8_t resTrkCh;  ///< Number of reserved tracking channels
  uint8_t maxTrkCh;  ///< Maximum number of tracking channels
  uint8_t reserved1; ///< Reserved
  uint32_t flags;    ///< Flags (bit0=enable, bits16-23=sigCfgMask)
} UBX_CFG_GNSS_block_t;

static_assert(sizeof(UBX_CFG_GNSS_block_t) == 8,
              "UBX_CFG_GNSS_block_t must be 8 bytes");

// GNSS IDs
#define UBX_GNSS_ID_GPS 0     ///< GPS
#define UBX_GNSS_ID_SBAS 1    ///< SBAS
#define UBX_GNSS_ID_GALILEO 2 ///< Galileo
#define UBX_GNSS_ID_BEIDOU 3  ///< BeiDou
#define UBX_GNSS_ID_IMES 4    ///< IMES
#define UBX_GNSS_ID_QZSS 5    ///< QZSS
#define UBX_GNSS_ID_GLONASS 6 ///< GLONASS

// GNSS flags
#define UBX_GNSS_FLAG_ENABLE 0x00000001 ///< Enable this GNSS

/** UBX-CFG-NMEA (0x06 0x17) - NMEA Protocol Configuration V1.
 *  20 bytes. Controls NMEA output formatting.
 */
typedef struct __attribute__((packed)) {
  uint8_t filter;        ///< Filter flags (see bitfield)
  uint8_t nmeaVersion;   ///< NMEA version (0x21=2.1, 0x23=2.3, 0x40=4.0,
                         ///< 0x41=4.10, 0x4B=4.11)
  uint8_t numSV;         ///< Max SVs per TalkerId (0=unlimited, 8, 12, 16)
  uint8_t flags;         ///< Flags (bit0=compat, bit1=consider, bit2=limit82,
                         ///< bit3=highPrec)
  uint32_t gnssToFilter; ///< GNSS filter mask (bit0=GPS, bit1=SBAS, etc.)
  uint8_t svNumbering;   ///< SV numbering (0=strict, 1=extended)
  uint8_t mainTalkerId;  ///< Main Talker ID (0=not overridden, 1=GP, 2=GL,
                         ///< 3=GN, 4=GA, 5=GB, 6=GQ)
  uint8_t gsvTalkerId;   ///< GSV Talker ID (0=GNSS specific, 1=main)
  uint8_t version;       ///< Message version (0x01)
  char bdsTalkerId[2];   ///< BeiDou Talker ID (2 chars)
  uint8_t reserved1[6];  ///< Reserved
} UBX_CFG_NMEA_t;

static_assert(sizeof(UBX_CFG_NMEA_t) == 20, "UBX_CFG_NMEA_t must be 20 bytes");

// CFG-NMEA filter bits
#define UBX_NMEA_FILTER_POSFILT 0x01     ///< Position output for failed fixes
#define UBX_NMEA_FILTER_MSKPOSFILT 0x02  ///< Position output for invalid fixes
#define UBX_NMEA_FILTER_TIMEFILT 0x04    ///< Time output for invalid times
#define UBX_NMEA_FILTER_DATEFILT 0x08    ///< Date output for invalid dates
#define UBX_NMEA_FILTER_GPSONLYFILT 0x10 ///< Restrict to GPS only
#define UBX_NMEA_FILTER_TRACKFILT 0x20   ///< COG output when frozen

// CFG-NMEA flags bits
#define UBX_NMEA_FLAGS_COMPAT 0x01   ///< Compatibility mode
#define UBX_NMEA_FLAGS_CONSIDER 0x02 ///< Considering mode
#define UBX_NMEA_FLAGS_LIMIT82 0x04  ///< Limit to 82 chars
#define UBX_NMEA_FLAGS_HIGHPREC 0x08 ///< High precision mode

/** UBX-CFG-ANT (0x06 0x13) - Antenna Control Settings.
 *  4 bytes. Controls antenna supervisor.
 */
typedef struct __attribute__((packed)) {
  uint16_t flags; ///< Antenna flags (bit0=svcs, bit1=scd, bit2=ocd,
                  ///< bit3=pdwnOnSCD, bit4=recovery)
  uint16_t pins;  ///< Pin configuration (bits0-4=pinSwitch, bits5-9=pinSCD,
                  ///< bits10-14=pinOCD, bit15=reconfig)
} UBX_CFG_ANT_t;

static_assert(sizeof(UBX_CFG_ANT_t) == 4, "UBX_CFG_ANT_t must be 4 bytes");

// CFG-ANT flags bits
#define UBX_ANT_FLAG_SVCS 0x0001      ///< Enable supply voltage control
#define UBX_ANT_FLAG_SCD 0x0002       ///< Enable short circuit detection
#define UBX_ANT_FLAG_OCD 0x0004       ///< Enable open circuit detection
#define UBX_ANT_FLAG_PDWNONSCD 0x0008 ///< Power down on short circuit
#define UBX_ANT_FLAG_RECOVERY 0x0010  ///< Enable auto recovery

/** UBX-CFG-SBAS (0x06 0x16) - SBAS Configuration.
 *  8 bytes. Controls SBAS settings.
 */
typedef struct __attribute__((packed)) {
  uint8_t mode;      ///< SBAS mode (bit0=enabled, bit1=test)
  uint8_t usage;     ///< SBAS usage (bit0=range, bit1=diffCorr, bit2=integrity)
  uint8_t maxSBAS;   ///< Maximum number of SBAS search channels
  uint8_t scanmode2; ///< Scanmode bitmask (see PRN details in spec)
  uint32_t scanmode1; ///< Scanmode bitmask (PRN 120-158 bits)
} UBX_CFG_SBAS_t;

static_assert(sizeof(UBX_CFG_SBAS_t) == 8, "UBX_CFG_SBAS_t must be 8 bytes");

// CFG-SBAS mode bits
#define UBX_SBAS_MODE_ENABLED 0x01 ///< SBAS enabled
#define UBX_SBAS_MODE_TEST 0x02    ///< SBAS test mode

// CFG-SBAS usage bits
#define UBX_SBAS_USAGE_RANGE 0x01     ///< Use SBAS for ranging
#define UBX_SBAS_USAGE_DIFFCORR 0x02  ///< Use SBAS diff corrections
#define UBX_SBAS_USAGE_INTEGRITY 0x04 ///< Use SBAS integrity info

/** UBX-CFG-INF (0x06 0x02) - Information Message Configuration block.
 *  10 bytes per protocol.
 */
typedef struct __attribute__((packed)) {
  uint8_t protocolID;    ///< Protocol ID (0=UBX, 1=NMEA)
  uint8_t reserved1[3];  ///< Reserved
  uint8_t infMsgMask[6]; ///< Info message mask per port (bit0=ERROR,
                         ///< bit1=WARNING, bit2=NOTICE, bit3=TEST, bit4=DEBUG)
} UBX_CFG_INF_block_t;

static_assert(sizeof(UBX_CFG_INF_block_t) == 10,
              "UBX_CFG_INF_block_t must be 10 bytes");

// CFG-INF protocol IDs
#define UBX_INF_PROTOCOL_UBX 0  ///< UBX protocol
#define UBX_INF_PROTOCOL_NMEA 1 ///< NMEA protocol

// CFG-INF message mask bits
#define UBX_INF_MSG_ERROR 0x01   ///< ERROR messages
#define UBX_INF_MSG_WARNING 0x02 ///< WARNING messages
#define UBX_INF_MSG_NOTICE 0x04  ///< NOTICE messages
#define UBX_INF_MSG_TEST 0x08    ///< TEST messages
#define UBX_INF_MSG_DEBUG 0x10   ///< DEBUG messages

/** UBX-CFG-RXM (0x06 0x11) - Receiver Manager Configuration.
 *  2 bytes. Controls power mode.
 */
typedef struct __attribute__((packed)) {
  uint8_t reserved1; ///< Reserved (always 8)
  uint8_t lpMode;    ///< Low power mode (0=continuous, 1=power save)
} UBX_CFG_RXM_t;

static_assert(sizeof(UBX_CFG_RXM_t) == 2, "UBX_CFG_RXM_t must be 2 bytes");

// CFG-RXM low power mode values
#define UBX_RXM_LPMODE_CONTINUOUS 0 ///< Continuous mode
#define UBX_RXM_LPMODE_POWERSAVE 1  ///< Power save mode

/** UBX-CFG-PM2 (0x06 0x3B) - Extended Power Management Configuration.
 *  44 bytes. Controls power save mode parameters.
 *  Note: Not supported on ADR, FTS, or HPG products.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;            ///< Message version (0x02 for this version)
  uint8_t reserved1;          ///< Reserved
  uint8_t maxStartupStateDur; ///< Max time in acquisition state (s), 0=disabled
  uint8_t reserved2;          ///< Reserved
  uint32_t flags;             ///< PSM configuration flags (see defines below)
  uint32_t updatePeriod;      ///< Position update period (ms), 0=no retry
  uint32_t searchPeriod;      ///< Acquisition retry period (ms), 0=no retry
  uint32_t gridOffset;   ///< Grid offset relative to GPS start of week (ms)
  uint16_t onTime;       ///< Time to stay in tracking state (s)
  uint16_t minAcqTime;   ///< Minimal search time (s)
  uint8_t reserved3[20]; ///< Reserved
} UBX_CFG_PM2_t;

static_assert(sizeof(UBX_CFG_PM2_t) == 44, "UBX_CFG_PM2_t must be 44 bytes");

// CFG-PM2 flags bitfield
#define UBX_PM2_FLAG_EXTINTSEL \
  0x00000010 ///< EXTINT pin select (0=EXTINT0, 1=EXTINT1)
#define UBX_PM2_FLAG_EXTINTWAKE 0x00000020     ///< EXTINT wake enable
#define UBX_PM2_FLAG_EXTINTBACKUP 0x00000040   ///< EXTINT backup enable
#define UBX_PM2_FLAG_EXTINTINACTIVE 0x00000080 ///< EXTINT inactivity enable
#define UBX_PM2_FLAG_LIMITPEAKCURR_DIS \
  0x00000000 ///< Limit peak current disabled
#define UBX_PM2_FLAG_LIMITPEAKCURR_EN 0x00000400 ///< Limit peak current enabled
#define UBX_PM2_FLAG_WAITTIMEFIX \
  0x00010000                              ///< Wait for time fix (vs normal fix)
#define UBX_PM2_FLAG_UPDATERTC 0x00020000 ///< Update RTC
#define UBX_PM2_FLAG_UPDATEEPH 0x00040000 ///< Update ephemeris
#define UBX_PM2_FLAG_DONOTENTEROFF \
  0x00200000                                ///< Don't enter inactive on no fix
#define UBX_PM2_FLAG_MODE_ONOFF 0x00000000  ///< ON/OFF mode (PSMOO)
#define UBX_PM2_FLAG_MODE_CYCLIC 0x00400000 ///< Cyclic tracking mode (PSMCT)

/** UBX-CFG-PMS (0x06 0x86) - Power Mode Setup.
 *  8 bytes. Simple power mode configuration.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;         ///< Message version (0x00)
  uint8_t powerSetupValue; ///< Power mode: 0=Full, 1=Balanced, 2=Interval,
                           ///< 3=Aggressive1Hz, 4=Aggressive2Hz,
                           ///< 5=Aggressive4Hz, 0xFF=Invalid
  uint16_t period; ///< Position update/search period (s). Min 10s recommended.
                   ///< Only for Interval mode, else 0.
  uint16_t onTime; ///< ON phase duration (s). Must be < period.
                   ///< Only for Interval mode, else 0.
  uint8_t reserved1[2]; ///< Reserved
} UBX_CFG_PMS_t;

static_assert(sizeof(UBX_CFG_PMS_t) == 8, "UBX_CFG_PMS_t must be 8 bytes");

// CFG-PMS power setup values
#define UBX_PMS_FULLPOWER 0x00      ///< Full power mode
#define UBX_PMS_BALANCED 0x01       ///< Balanced power mode
#define UBX_PMS_INTERVAL 0x02       ///< Interval power mode
#define UBX_PMS_AGGRESSIVE_1HZ 0x03 ///< Aggressive 1 Hz mode
#define UBX_PMS_AGGRESSIVE_2HZ 0x04 ///< Aggressive 2 Hz mode
#define UBX_PMS_AGGRESSIVE_4HZ 0x05 ///< Aggressive 4 Hz mode
#define UBX_PMS_INVALID 0xFF        ///< Invalid (poll response only)

/** UBX-RXM-PMREQ (0x02 0x41) - Power Management Request (v0).
 *  8 bytes. Send-only command to request power management task.
 *  Do NOT expect ACK - module enters backup mode immediately.
 */
typedef struct __attribute__((packed)) {
  uint32_t
      duration;   ///< Duration of task (ms). Max ~12 days. 0=wake on pin only.
  uint32_t flags; ///< Task flags (see defines below)
} UBX_RXM_PMREQ_t;

static_assert(sizeof(UBX_RXM_PMREQ_t) == 8, "UBX_RXM_PMREQ_t must be 8 bytes");

/** UBX-RXM-PMREQ (0x02 0x41) - Power Management Request (v1).
 *  16 bytes. Extended version with wakeup source configuration.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;      ///< Message version (0x00)
  uint8_t reserved1[3]; ///< Reserved
  uint32_t duration;    ///< Duration of task (ms). Max ~12 days. 0=wake on pin.
  uint32_t flags;       ///< Task flags (see defines below)
  uint32_t wakeupSources; ///< Wakeup source configuration (see defines below)
} UBX_RXM_PMREQ_V1_t;

static_assert(sizeof(UBX_RXM_PMREQ_V1_t) == 16,
              "UBX_RXM_PMREQ_V1_t must be 16 bytes");

// RXM-PMREQ flags
#define UBX_PMREQ_FLAG_BACKUP 0x00000002 ///< Enter backup mode
#define UBX_PMREQ_FLAG_FORCE 0x00000004  ///< Force backup even on USB (v1 only)

// RXM-PMREQ wakeup sources (v1 only)
#define UBX_PMREQ_WAKE_UARTRX 0x00000008  ///< Wake on UART RX edge
#define UBX_PMREQ_WAKE_EXTINT0 0x00000020 ///< Wake on EXTINT0 edge
#define UBX_PMREQ_WAKE_EXTINT1 0x00000040 ///< Wake on EXTINT1 edge
#define UBX_PMREQ_WAKE_SPICS 0x00000080   ///< Wake on SPI CS edge

/** UBX-MON-HW (0x0A 0x09) - Hardware Status.
 *  60 bytes. Reports antenna, PIO, noise level, and AGC status.
 */
typedef struct __attribute__((packed)) {
  uint32_t pinSel;      ///< Mask of pins set as peripheral/PIO
  uint32_t pinBank;     ///< Mask of pins set as bank A/B
  uint32_t pinDir;      ///< Mask of pins set as input/output
  uint32_t pinVal;      ///< Mask of pins value low/high
  uint16_t noisePerMS;  ///< Noise level as measured by GPS core
  uint16_t agcCnt;      ///< AGC monitor (0-8191 = 0-100%)
  uint8_t aStatus;      ///< Antenna supervisor state: 0=INIT, 1=DONTKNOW,
                        ///< 2=OK, 3=SHORT, 4=OPEN
  uint8_t aPower;       ///< Antenna power: 0=OFF, 1=ON, 2=DONTKNOW
  uint8_t flags;        ///< Flags (see defines below)
  uint8_t reserved1;    ///< Reserved
  uint32_t usedMask;    ///< Mask of pins used by virtual pin manager
  uint8_t VP[17];       ///< Pin mappings for 17 physical pins
  uint8_t jamInd;       ///< CW jamming indicator (0=none, 255=strong)
  uint8_t reserved2[2]; ///< Reserved
  uint32_t pinIrq;      ///< Mask of pins using PIO IRQ
  uint32_t pullH;       ///< Mask of pins using pull-high resistor
  uint32_t pullL;       ///< Mask of pins using pull-low resistor
} UBX_MON_HW_t;

static_assert(sizeof(UBX_MON_HW_t) == 60, "UBX_MON_HW_t must be 60 bytes");

// MON-HW flags
#define UBX_MON_HW_FLAG_RTCCALIB 0x01      ///< RTC is calibrated
#define UBX_MON_HW_FLAG_SAFEBOOT 0x02      ///< Safeboot mode active
#define UBX_MON_HW_FLAG_JAMSTATE_MASK 0x0C ///< Jamming state mask (bits 2-3)
#define UBX_MON_HW_FLAG_XTALABSENT 0x10    ///< RTC xtal absent

// MON-HW antenna status values
#define UBX_MON_HW_ASTATUS_INIT 0     ///< Antenna supervisor initializing
#define UBX_MON_HW_ASTATUS_DONTKNOW 1 ///< Status unknown
#define UBX_MON_HW_ASTATUS_OK 2       ///< Antenna OK
#define UBX_MON_HW_ASTATUS_SHORT 3    ///< Antenna short circuit
#define UBX_MON_HW_ASTATUS_OPEN 4     ///< Antenna open/not connected

// MON-HW antenna power values
#define UBX_MON_HW_APOWER_OFF 0      ///< Antenna power off
#define UBX_MON_HW_APOWER_ON 1       ///< Antenna power on
#define UBX_MON_HW_APOWER_DONTKNOW 2 ///< Antenna power unknown

/** UBX-MON-GNSS (0x0A 0x28) - GNSS System Information.
 *  8 bytes. Reports major GNSS selection capabilities and status.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;      ///< Message version (0x00)
  uint8_t supported;    ///< Bit mask of supported GNSS (bit0=GPS, bit1=GLONASS,
                        ///< bit2=BeiDou, bit3=Galileo)
  uint8_t defaultGnss;  ///< Bit mask of default GNSS selection
  uint8_t enabled;      ///< Bit mask of currently enabled GNSS
  uint8_t simultaneous; ///< Max concurrent major GNSS supported
  uint8_t reserved1[3]; ///< Reserved
} UBX_MON_GNSS_t;

static_assert(sizeof(UBX_MON_GNSS_t) == 8, "UBX_MON_GNSS_t must be 8 bytes");

// MON-GNSS bit masks (apply to supported, defaultGnss, enabled fields)
#define UBX_MON_GNSS_GPS 0x01     ///< GPS supported/enabled
#define UBX_MON_GNSS_GLONASS 0x02 ///< GLONASS supported/enabled
#define UBX_MON_GNSS_BEIDOU 0x04  ///< BeiDou supported/enabled
#define UBX_MON_GNSS_GALILEO 0x08 ///< Galileo supported/enabled

/** UBX-MON-HW2 (0x0A 0x0B) - Extended Hardware Status.
 *  28 bytes. IQ imbalance, config source, POST status.
 */
typedef struct __attribute__((packed)) {
  int8_t ofsI;       ///< I-part imbalance (-128 to 127)
  uint8_t magI;      ///< I-part magnitude (0=no signal, 255=max)
  int8_t ofsQ;       ///< Q-part imbalance (-128 to 127)
  uint8_t magQ;      ///< Q-part magnitude (0=no signal, 255=max)
  uint8_t cfgSource; ///< Config source: 114=ROM, 111=OTP, 112=pins, 102=flash
  uint8_t reserved1[3]; ///< Reserved
  uint32_t lowLevCfg;   ///< Low-level configuration (obsolete after v15)
  uint8_t reserved2[8]; ///< Reserved
  uint32_t postStatus;  ///< POST status word
  uint8_t reserved3[4]; ///< Reserved
} UBX_MON_HW2_t;

static_assert(sizeof(UBX_MON_HW2_t) == 28, "UBX_MON_HW2_t must be 28 bytes");

// MON-HW2 config source values
#define UBX_MON_HW2_CFG_ROM 114   ///< Configuration from ROM
#define UBX_MON_HW2_CFG_OTP 111   ///< Configuration from OTP
#define UBX_MON_HW2_CFG_PINS 112  ///< Configuration from pins
#define UBX_MON_HW2_CFG_FLASH 102 ///< Configuration from flash

/** UBX-MON-IO (0x0A 0x02) - I/O System Status per port.
 *  20 bytes per port. Variable length message (20*N ports).
 */
typedef struct __attribute__((packed)) {
  uint32_t rxBytes;     ///< Number of bytes ever received
  uint32_t txBytes;     ///< Number of bytes ever sent
  uint16_t parityErrs;  ///< Number of 100ms slots with parity errors
  uint16_t framingErrs; ///< Number of 100ms slots with framing errors
  uint16_t overrunErrs; ///< Number of 100ms slots with overrun errors
  uint16_t breakCond;   ///< Number of 100ms slots with break conditions
  uint8_t reserved1[4]; ///< Reserved
} UBX_MON_IO_port_t;

static_assert(sizeof(UBX_MON_IO_port_t) == 20,
              "UBX_MON_IO_port_t must be 20 bytes");

/** UBX-MON-MSGPP (0x0A 0x06) - Message Parse and Process Status.
 *  120 bytes. Message counts per protocol per port.
 */
typedef struct __attribute__((packed)) {
  uint16_t msg[6][8];  ///< Parsed message counts [port][protocol]
  uint32_t skipped[6]; ///< Skipped bytes per port
} UBX_MON_MSGPP_t;

static_assert(sizeof(UBX_MON_MSGPP_t) == 120,
              "UBX_MON_MSGPP_t must be 120 bytes");

/** UBX-MON-RXBUF (0x0A 0x07) - Receiver Buffer Status.
 *  24 bytes. Buffer pending/usage per port.
 */
typedef struct __attribute__((packed)) {
  uint16_t pending[6];  ///< Bytes pending in RX buffer per port
  uint8_t usage[6];     ///< Current usage % per port
  uint8_t peakUsage[6]; ///< Peak usage % per port
} UBX_MON_RXBUF_t;

static_assert(sizeof(UBX_MON_RXBUF_t) == 24,
              "UBX_MON_RXBUF_t must be 24 bytes");

/** UBX-MON-TXBUF (0x0A 0x08) - Transmitter Buffer Status.
 *  28 bytes. TX buffer pending/usage per port plus errors.
 */
typedef struct __attribute__((packed)) {
  uint16_t pending[6];  ///< Bytes pending in TX buffer per port
  uint8_t usage[6];     ///< Current usage % per port
  uint8_t peakUsage[6]; ///< Peak usage % per port
  uint8_t tUsage;       ///< Total current usage % across all ports
  uint8_t tPeakUsage;   ///< Total peak usage % across all ports
  uint8_t errors;       ///< Error flags (bit0=limit, bit1=mem, bit2=alloc)
  uint8_t reserved1;    ///< Reserved
} UBX_MON_TXBUF_t;

static_assert(sizeof(UBX_MON_TXBUF_t) == 28,
              "UBX_MON_TXBUF_t must be 28 bytes");

// MON-TXBUF error flags
#define UBX_MON_TXBUF_ERR_LIMIT 0x01 ///< Buffer limit reached
#define UBX_MON_TXBUF_ERR_MEM 0x02   ///< Memory allocation error
#define UBX_MON_TXBUF_ERR_ALLOC 0x04 ///< TX buffer full allocation error

/** UBX SEC Message IDs. */
typedef enum {
  UBX_SEC_UNIQID = 0x03 // Unique Chip ID
} UBXSecMessageId;

/** UBX-SEC-UNIQID (0x27 0x03) - Unique Chip ID.
 *  9 bytes. 40-bit unique chip identifier.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;      ///< Message version (0x01)
  uint8_t reserved1[3]; ///< Reserved
  uint8_t uniqueId[5];  ///< Unique 40-bit chip ID
} UBX_SEC_UNIQID_t;

static_assert(sizeof(UBX_SEC_UNIQID_t) == 9,
              "UBX_SEC_UNIQID_t must be 9 bytes");

/** UBX INF Message IDs. */
typedef enum {
  UBX_INF_ERROR = 0x00,   // Error message
  UBX_INF_WARNING = 0x01, // Warning message
  UBX_INF_NOTICE = 0x02,  // Notice message
  UBX_INF_TEST = 0x03,    // Test message
  UBX_INF_DEBUG = 0x04    // Debug message
} UBXInfMessageId;

// =====================================================================
// Phase 6: Advanced Feature Messages
// =====================================================================

/** UBX-NAV-ODO (0x01 0x09) - Odometer Solution.
 *  20 bytes. Distance traveled since reset.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;        ///< Message version (0x00)
  uint8_t reserved1[3];   ///< Reserved
  uint32_t iTOW;          ///< GPS time of week (ms)
  uint32_t distance;      ///< Ground distance since last reset (m)
  uint32_t totalDistance; ///< Total cumulative ground distance (m)
  uint32_t distanceStd;   ///< Ground distance accuracy 1-sigma (m)
} UBX_NAV_ODO_t;

static_assert(sizeof(UBX_NAV_ODO_t) == 20, "UBX_NAV_ODO_t must be 20 bytes");

/** UBX-NAV-HPPOSLLH (0x01 0x14) - High Precision Geodetic Position.
 *  36 bytes. High precision lat/lon/height with mm residuals.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;      ///< Message version (0x00)
  uint8_t reserved1[2]; ///< Reserved
  uint8_t flags;        ///< Flags (bit0=invalidLlh)
  uint32_t iTOW;        ///< GPS time of week (ms)
  int32_t lon;          ///< Longitude (deg, scale 1e-7)
  int32_t lat;          ///< Latitude (deg, scale 1e-7)
  int32_t height;       ///< Height above ellipsoid (mm)
  int32_t hMSL;         ///< Height above mean sea level (mm)
  int8_t lonHp; ///< High precision longitude (deg, scale 1e-9). Range -99..+99
  int8_t latHp; ///< High precision latitude (deg, scale 1e-9). Range -99..+99
  int8_t heightHp; ///< High precision height (mm, scale 0.1). Range -9..+9
  int8_t hMSLHp;   ///< High precision hMSL (mm, scale 0.1). Range -9..+9
  uint32_t hAcc;   ///< Horizontal accuracy estimate (mm, scale 0.1)
  uint32_t vAcc;   ///< Vertical accuracy estimate (mm, scale 0.1)
} UBX_NAV_HPPOSLLH_t;

static_assert(sizeof(UBX_NAV_HPPOSLLH_t) == 36,
              "UBX_NAV_HPPOSLLH_t must be 36 bytes");

// NAV-HPPOSLLH flags
#define UBX_NAV_HPPOSLLH_FLAG_INVALID 0x01 ///< LLH data invalid

/** UBX-NAV-HPPOSECEF (0x01 0x13) - High Precision ECEF Position.
 *  28 bytes. High precision ECEF coordinates with mm residuals.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;      ///< Message version (0x00)
  uint8_t reserved1[3]; ///< Reserved
  uint32_t iTOW;        ///< GPS time of week (ms)
  int32_t ecefX;        ///< ECEF X coordinate (cm)
  int32_t ecefY;        ///< ECEF Y coordinate (cm)
  int32_t ecefZ;        ///< ECEF Z coordinate (cm)
  int8_t ecefXHp;       ///< High precision X (mm, scale 0.1). Range -99..+99
  int8_t ecefYHp;       ///< High precision Y (mm, scale 0.1). Range -99..+99
  int8_t ecefZHp;       ///< High precision Z (mm, scale 0.1). Range -99..+99
  uint8_t flags;        ///< Flags (bit0=invalidEcef)
  uint32_t pAcc;        ///< Position accuracy estimate (mm, scale 0.1)
} UBX_NAV_HPPOSECEF_t;

static_assert(sizeof(UBX_NAV_HPPOSECEF_t) == 28,
              "UBX_NAV_HPPOSECEF_t must be 28 bytes");

// NAV-HPPOSECEF flags
#define UBX_NAV_HPPOSECEF_FLAG_INVALID 0x01 ///< ECEF data invalid

/** UBX-NAV-RELPOSNED (0x01 0x3C) - Relative Position NED (v0).
 *  40 bytes. Relative position from base to rover for RTK.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;       ///< Message version (0x00)
  uint8_t reserved1;     ///< Reserved
  uint16_t refStationId; ///< Reference station ID (0-4095)
  uint32_t iTOW;         ///< GPS time of week (ms)
  int32_t relPosN;       ///< North component (cm)
  int32_t relPosE;       ///< East component (cm)
  int32_t relPosD;       ///< Down component (cm)
  int8_t relPosHPN;      ///< HP North (mm, scale 0.1). Range -99..+99
  int8_t relPosHPE;      ///< HP East (mm, scale 0.1). Range -99..+99
  int8_t relPosHPD;      ///< HP Down (mm, scale 0.1). Range -99..+99
  uint8_t reserved2;     ///< Reserved
  uint32_t accN;         ///< North accuracy (mm, scale 0.1)
  uint32_t accE;         ///< East accuracy (mm, scale 0.1)
  uint32_t accD;         ///< Down accuracy (mm, scale 0.1)
  uint32_t flags;        ///< Flags (see defines below)
} UBX_NAV_RELPOSNED_t;

static_assert(sizeof(UBX_NAV_RELPOSNED_t) == 40,
              "UBX_NAV_RELPOSNED_t must be 40 bytes");

// NAV-RELPOSNED flags
#define UBX_NAV_RELPOSNED_FLAG_GNSS_FIX_OK 0x00000001 ///< Valid fix
#define UBX_NAV_RELPOSNED_FLAG_DIFF_SOLN 0x00000002   ///< Diff corrections used
#define UBX_NAV_RELPOSNED_FLAG_REL_POS_VALID 0x00000004  ///< Rel pos valid
#define UBX_NAV_RELPOSNED_FLAG_CARR_SOLN_MASK 0x00000018 ///< Carrier solution
#define UBX_NAV_RELPOSNED_FLAG_IS_MOVING 0x00000020    ///< Moving baseline mode
#define UBX_NAV_RELPOSNED_FLAG_REF_POS_MISS 0x00000040 ///< Ref pos extrapolated
#define UBX_NAV_RELPOSNED_FLAG_REF_OBS_MISS 0x00000080 ///< Ref obs extrapolated

/** UBX-NAV-SVIN (0x01 0x3B) - Survey-in Data.
 *  40 bytes. Survey-in status for RTK base.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;      ///< Message version (0x00)
  uint8_t reserved1[3]; ///< Reserved
  uint32_t iTOW;        ///< GPS time of week (ms)
  uint32_t dur;         ///< Passed survey-in observation time (s)
  int32_t meanX;        ///< Mean ECEF X coordinate (cm)
  int32_t meanY;        ///< Mean ECEF Y coordinate (cm)
  int32_t meanZ;        ///< Mean ECEF Z coordinate (cm)
  int8_t meanXHP;       ///< HP mean X (0.1 mm). Range -99..+99
  int8_t meanYHP;       ///< HP mean Y (0.1 mm). Range -99..+99
  int8_t meanZHP;       ///< HP mean Z (0.1 mm). Range -99..+99
  uint8_t reserved2;    ///< Reserved
  uint32_t meanAcc;     ///< Mean position accuracy (0.1 mm)
  uint32_t obs;         ///< Number of observations used
  uint8_t valid;        ///< Survey-in valid flag
  uint8_t active;       ///< Survey-in in progress flag
  uint8_t reserved3[2]; ///< Reserved
} UBX_NAV_SVIN_t;

static_assert(sizeof(UBX_NAV_SVIN_t) == 40, "UBX_NAV_SVIN_t must be 40 bytes");

/** UBX-NAV-GEOFENCE (0x01 0x39) - Geofencing Status header.
 *  8 bytes fixed + 2 bytes per fence.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW;     ///< GPS time of week (ms)
  uint8_t version;   ///< Message version (0x00)
  uint8_t status;    ///< Geofencing status: 0=not available, 1=active
  uint8_t numFences; ///< Number of geofences
  uint8_t combState; ///< Combined state: 0=Unknown, 1=Inside, 2=Outside
} UBX_NAV_GEOFENCE_header_t;

static_assert(sizeof(UBX_NAV_GEOFENCE_header_t) == 8,
              "UBX_NAV_GEOFENCE_header_t must be 8 bytes");

/** UBX-NAV-GEOFENCE per-fence state. 2 bytes each. */
typedef struct __attribute__((packed)) {
  uint8_t state; ///< Geofence state: 0=Unknown, 1=Inside, 2=Outside
  uint8_t id;    ///< Geofence ID (0 = not available)
} UBX_NAV_GEOFENCE_fence_t;

static_assert(sizeof(UBX_NAV_GEOFENCE_fence_t) == 2,
              "UBX_NAV_GEOFENCE_fence_t must be 2 bytes");

// NAV-GEOFENCE states
#define UBX_GEOFENCE_STATE_UNKNOWN 0 ///< Unknown state
#define UBX_GEOFENCE_STATE_INSIDE 1  ///< Inside geofence
#define UBX_GEOFENCE_STATE_OUTSIDE 2 ///< Outside geofence

/** UBX-NAV-TIMELS (0x01 0x26) - Leap Second Event Information.
 *  24 bytes. Information about leap seconds.
 */
typedef struct __attribute__((packed)) {
  uint32_t iTOW;          ///< GPS time of week (ms)
  uint8_t version;        ///< Message version (0x00)
  uint8_t reserved1[3];   ///< Reserved
  uint8_t srcOfCurrLs;    ///< Source of current leap seconds
  int8_t currLs;          ///< Current leap seconds (GPS-UTC) (s)
  uint8_t srcOfLsChange;  ///< Source of leap second change info
  int8_t lsChange;        ///< Upcoming leap second change (s)
  int32_t timeToLsEvent;  ///< Seconds until next leap second event (s)
  uint16_t dateOfLsGpsWn; ///< GPS week of leap second event
  uint16_t dateOfLsGpsDn; ///< GPS day of leap second event (1-7)
  uint8_t reserved2[3];   ///< Reserved
  uint8_t valid;          ///< Validity flags
} UBX_NAV_TIMELS_t;

static_assert(sizeof(UBX_NAV_TIMELS_t) == 24,
              "UBX_NAV_TIMELS_t must be 24 bytes");

// NAV-TIMELS source values
#define UBX_TIMELS_SRC_DEFAULT 0   ///< Default (fw or SBAS)
#define UBX_TIMELS_SRC_GPS 1       ///< GPS
#define UBX_TIMELS_SRC_SBAS 2      ///< SBAS
#define UBX_TIMELS_SRC_BEIDOU 3    ///< BeiDou
#define UBX_TIMELS_SRC_GALILEO 4   ///< Galileo
#define UBX_TIMELS_SRC_GLONASS 5   ///< GLONASS
#define UBX_TIMELS_SRC_UNKNOWN 255 ///< Unknown

// NAV-TIMELS valid flags
#define UBX_TIMELS_VALID_CURR_LS 0x01       ///< currLs valid
#define UBX_TIMELS_VALID_TIME_TO_EVENT 0x02 ///< timeToLsEvent valid

/** UBX-CFG-TMODE3 (0x06 0x71) - Time Mode 3 Configuration.
 *  40 bytes. Configure RTK base station mode.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;       ///< Message version (0x00)
  uint8_t reserved1;     ///< Reserved
  uint16_t flags;        ///< Receiver mode flags (see defines)
  int32_t ecefXOrLat;    ///< ECEF X or Latitude (cm or deg*1e-7)
  int32_t ecefYOrLon;    ///< ECEF Y or Longitude (cm or deg*1e-7)
  int32_t ecefZOrAlt;    ///< ECEF Z or Altitude (cm)
  int8_t ecefXOrLatHP;   ///< HP ECEF X or Lat (0.1mm or deg*1e-9)
  int8_t ecefYOrLonHP;   ///< HP ECEF Y or Lon (0.1mm or deg*1e-9)
  int8_t ecefZOrAltHP;   ///< HP ECEF Z or Alt (0.1mm)
  uint8_t reserved2;     ///< Reserved
  uint32_t fixedPosAcc;  ///< Fixed position 3D accuracy (0.1mm)
  uint32_t svinMinDur;   ///< Survey-in min duration (s)
  uint32_t svinAccLimit; ///< Survey-in accuracy limit (0.1mm)
  uint8_t reserved3[8];  ///< Reserved
} UBX_CFG_TMODE3_t;

static_assert(sizeof(UBX_CFG_TMODE3_t) == 40,
              "UBX_CFG_TMODE3_t must be 40 bytes");

// CFG-TMODE3 flags
#define UBX_TMODE3_MODE_DISABLED 0x0000  ///< Disabled
#define UBX_TMODE3_MODE_SURVEY_IN 0x0001 ///< Survey-in mode
#define UBX_TMODE3_MODE_FIXED 0x0002     ///< Fixed mode
#define UBX_TMODE3_FLAG_LLA 0x0100       ///< Position is LLA (not ECEF)

/** UBX-CFG-GEOFENCE (0x06 0x69) - Geofencing Configuration header.
 *  8 bytes fixed + 12 bytes per fence.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;     ///< Message version (0x00)
  uint8_t numFences;   ///< Number of geofences (max 4)
  uint8_t confLvl;     ///< Confidence level: 0=none, 1=68%, 2=95%, 3=99.7%
  uint8_t reserved1;   ///< Reserved
  uint8_t pioEnabled;  ///< Enable PIO output for fence state
  uint8_t pinPolarity; ///< PIO polarity: 0=Low inside, 1=Low outside
  uint8_t pin;         ///< PIO pin number
  uint8_t reserved2;   ///< Reserved
} UBX_CFG_GEOFENCE_header_t;

static_assert(sizeof(UBX_CFG_GEOFENCE_header_t) == 8,
              "UBX_CFG_GEOFENCE_header_t must be 8 bytes");

/** UBX-CFG-GEOFENCE per-fence definition. 12 bytes each. */
typedef struct __attribute__((packed)) {
  int32_t lat;     ///< Latitude of center (deg * 1e-7)
  int32_t lon;     ///< Longitude of center (deg * 1e-7)
  uint32_t radius; ///< Radius of geofence (cm)
} UBX_CFG_GEOFENCE_fence_t;

static_assert(sizeof(UBX_CFG_GEOFENCE_fence_t) == 12,
              "UBX_CFG_GEOFENCE_fence_t must be 12 bytes");

/** UBX-CFG-TP5 (0x06 0x31) - Time Pulse Configuration.
 *  32 bytes. Configure timepulse output.
 */
typedef struct __attribute__((packed)) {
  uint8_t tpIdx;              ///< Timepulse index (0 or 1)
  uint8_t version;            ///< Message version (0x01)
  uint8_t reserved1[2];       ///< Reserved
  int16_t antCableDelay;      ///< Antenna cable delay (ns)
  int16_t rfGroupDelay;       ///< RF group delay (ns)
  uint32_t freqPeriod;        ///< Frequency or period (Hz or us)
  uint32_t freqPeriodLock;    ///< Freq/period when locked to GNSS
  uint32_t pulseLenRatio;     ///< Pulse length or duty cycle (us or 2^-32)
  uint32_t pulseLenRatioLock; ///< Pulse len/duty when locked
  int32_t userConfigDelay;    ///< User-configurable delay (ns)
  uint32_t flags;             ///< Configuration flags (see defines)
} UBX_CFG_TP5_t;

static_assert(sizeof(UBX_CFG_TP5_t) == 32, "UBX_CFG_TP5_t must be 32 bytes");

// CFG-TP5 flags
#define UBX_TP5_FLAG_ACTIVE 0x00000001           ///< Timepulse active
#define UBX_TP5_FLAG_LOCK_GNSS_FREQ 0x00000002   ///< Lock to GNSS frequency
#define UBX_TP5_FLAG_LOCKED_OTHER_SET 0x00000004 ///< Use locked params
#define UBX_TP5_FLAG_IS_FREQ 0x00000008          ///< Interpret as frequency
#define UBX_TP5_FLAG_IS_LENGTH 0x00000010        ///< Interpret as length
#define UBX_TP5_FLAG_ALIGN_TO_TOW 0x00000020     ///< Align to top of second
#define UBX_TP5_FLAG_POLARITY 0x00000040 ///< Rising edge at top of second
#define UBX_TP5_FLAG_GRID_UTC_GNSS_MASK 0x00000780 ///< Time grid (bits 7-10)

/** UBX-UPD-SOS (0x09 0x14) - Save on Shutdown command.
 *  4 bytes send, 8 bytes response.
 */
typedef struct __attribute__((packed)) {
  uint8_t cmd;          ///< Command: 0=create backup, 1=clear backup
  uint8_t reserved1[3]; ///< Reserved
} UBX_UPD_SOS_cmd_t;

static_assert(sizeof(UBX_UPD_SOS_cmd_t) == 4,
              "UBX_UPD_SOS_cmd_t must be 4 bytes");

typedef struct __attribute__((packed)) {
  uint8_t cmd;          ///< Command echo
  uint8_t reserved1[3]; ///< Reserved
  uint8_t response;     ///< Response: 0=unknown, 1=failed, 2=restored, 3=none
  uint8_t reserved2[3]; ///< Reserved
} UBX_UPD_SOS_response_t;

static_assert(sizeof(UBX_UPD_SOS_response_t) == 8,
              "UBX_UPD_SOS_response_t must be 8 bytes");

// UPD-SOS commands
#define UBX_UPD_SOS_CMD_CREATE 0 ///< Create backup in flash
#define UBX_UPD_SOS_CMD_CLEAR 1  ///< Clear backup from flash

// UPD-SOS response values
#define UBX_UPD_SOS_RESP_UNKNOWN 0  ///< Unknown
#define UBX_UPD_SOS_RESP_FAILED 1   ///< Backup failed
#define UBX_UPD_SOS_RESP_RESTORED 2 ///< Backup restored
#define UBX_UPD_SOS_RESP_NONE 3     ///< No backup

/** UBX LOG Message IDs. */
typedef enum {
  UBX_LOG_ERASE = 0x03,          // Erase log
  UBX_LOG_STRING = 0x04,         // Store string
  UBX_LOG_CREATE = 0x07,         // Create log
  UBX_LOG_INFO = 0x08,           // Log info
  UBX_LOG_RETRIEVE = 0x09,       // Retrieve log
  UBX_LOG_RETRIEVEPOS = 0x0B,    // Retrieve position
  UBX_LOG_RETRIEVESTRING = 0x0D, // Retrieve string
  UBX_LOG_FINDTIME = 0x0E        // Find time in log
} UBXLogMessageId;

/** UBX UPD Message IDs. */
typedef enum {
  UBX_UPD_SOS = 0x14 // Save on shutdown
} UBXUpdMessageId;

/** UBX-LOG-CREATE (0x21 0x07) - Create Log File.
 *  8 bytes. Command to create logging file.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;          ///< Message version (0x00)
  uint8_t logCfg;           ///< Config flags (bit0=circular)
  uint8_t reserved1;        ///< Reserved
  uint8_t logSize;          ///< Size: 0=max safe, 1=min, 2=user-defined
  uint32_t userDefinedSize; ///< User-defined max size (bytes)
} UBX_LOG_CREATE_t;

static_assert(sizeof(UBX_LOG_CREATE_t) == 8,
              "UBX_LOG_CREATE_t must be 8 bytes");

// LOG-CREATE logCfg flags
#define UBX_LOG_CREATE_CIRCULAR 0x01 ///< Circular log

// LOG-CREATE logSize values
#define UBX_LOG_SIZE_MAX_SAFE 0     ///< Maximum safe size
#define UBX_LOG_SIZE_MINIMUM 1      ///< Minimum size
#define UBX_LOG_SIZE_USER_DEFINED 2 ///< User-defined size

/** UBX-LOG-INFO (0x21 0x08) - Log Information.
 *  48 bytes. Information about current log.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;            ///< Message version (0x01)
  uint8_t reserved1[3];       ///< Reserved
  uint32_t filestoreCapacity; ///< Filestore capacity (bytes)
  uint8_t reserved2[8];       ///< Reserved
  uint32_t currentMaxLogSize; ///< Max log size allowed (bytes)
  uint32_t currentLogSize;    ///< Current log size (bytes)
  uint32_t entryCount;        ///< Number of entries in log
  uint16_t oldestYear;        ///< Oldest entry year (0 if no entries)
  uint8_t oldestMonth;        ///< Oldest entry month (1-12)
  uint8_t oldestDay;          ///< Oldest entry day (1-31)
  uint8_t oldestHour;         ///< Oldest entry hour (0-23)
  uint8_t oldestMinute;       ///< Oldest entry minute (0-59)
  uint8_t oldestSecond;       ///< Oldest entry second (0-60)
  uint8_t reserved3;          ///< Reserved
  uint16_t newestYear;        ///< Newest entry year (0 if no entries)
  uint8_t newestMonth;        ///< Newest entry month (1-12)
  uint8_t newestDay;          ///< Newest entry day (1-31)
  uint8_t newestHour;         ///< Newest entry hour (0-23)
  uint8_t newestMinute;       ///< Newest entry minute (0-59)
  uint8_t newestSecond;       ///< Newest entry second (0-60)
  uint8_t reserved4;          ///< Reserved
  uint8_t status;             ///< Status flags (see defines)
  uint8_t reserved5[3];       ///< Reserved
} UBX_LOG_INFO_t;

static_assert(sizeof(UBX_LOG_INFO_t) == 48, "UBX_LOG_INFO_t must be 48 bytes");

// LOG-INFO status flags
#define UBX_LOG_INFO_STATUS_RECORDING 0x01 ///< Recording enabled
#define UBX_LOG_INFO_STATUS_INACTIVE 0x02  ///< Log inactive (no log present)
#define UBX_LOG_INFO_STATUS_CIRCULAR 0x04  ///< Log is circular

/** UBX-LOG-RETRIEVE (0x21 0x09) - Request Log Data.
 *  12 bytes. Command to retrieve log entries.
 */
typedef struct __attribute__((packed)) {
  uint32_t startNumber; ///< Index of first entry to retrieve
  uint32_t entryCount;  ///< Number of entries to retrieve (max 256)
  uint8_t version;      ///< Message version (0x00)
  uint8_t reserved1[3]; ///< Reserved
} UBX_LOG_RETRIEVE_t;

static_assert(sizeof(UBX_LOG_RETRIEVE_t) == 12,
              "UBX_LOG_RETRIEVE_t must be 12 bytes");

/** UBX-LOG-RETRIEVEPOS (0x21 0x0B) - Position Fix Log Entry.
 *  40 bytes. Output message for log position entries.
 */
typedef struct __attribute__((packed)) {
  uint32_t entryIndex; ///< Index of this log entry
  int32_t lon;         ///< Longitude (deg * 1e-7)
  int32_t lat;         ///< Latitude (deg * 1e-7)
  int32_t hMSL;        ///< Height above MSL (mm)
  uint32_t hAcc;       ///< Horizontal accuracy (mm)
  uint32_t gSpeed;     ///< Ground speed (mm/s)
  uint32_t heading;    ///< Heading (deg * 1e-5)
  uint8_t version;     ///< Message version (0x00)
  uint8_t fixType;     ///< Fix type: 1=DR, 2=2D, 3=3D, 4=GNSS+DR
  uint16_t year;       ///< Year (UTC)
  uint8_t month;       ///< Month (1-12)
  uint8_t day;         ///< Day (1-31)
  uint8_t hour;        ///< Hour (0-23)
  uint8_t minute;      ///< Minute (0-59)
  uint8_t second;      ///< Second (0-60)
  uint8_t reserved1;   ///< Reserved
  uint8_t numSV;       ///< Number of satellites
  uint8_t reserved2;   ///< Reserved
} UBX_LOG_RETRIEVEPOS_t;

static_assert(sizeof(UBX_LOG_RETRIEVEPOS_t) == 40,
              "UBX_LOG_RETRIEVEPOS_t must be 40 bytes");

/** UBX-LOG-RETRIEVESTRING (0x21 0x0D) - Byte String Log Entry header.
 *  16 bytes fixed + variable string bytes.
 */
typedef struct __attribute__((packed)) {
  uint32_t entryIndex; ///< Index of this log entry
  uint8_t version;     ///< Message version (0x00)
  uint8_t reserved1;   ///< Reserved
  uint16_t year;       ///< Year (UTC) or 0 if unknown
  uint8_t month;       ///< Month (1-12)
  uint8_t day;         ///< Day (1-31)
  uint8_t hour;        ///< Hour (0-23)
  uint8_t minute;      ///< Minute (0-59)
  uint8_t second;      ///< Second (0-60)
  uint8_t reserved2;   ///< Reserved
  uint16_t byteCount;  ///< Size of string in bytes
} UBX_LOG_RETRIEVESTRING_header_t;

static_assert(sizeof(UBX_LOG_RETRIEVESTRING_header_t) == 16,
              "UBX_LOG_RETRIEVESTRING_header_t must be 16 bytes");

/** UBX-LOG-FINDTIME request (0x21 0x0E) - Find Log Entry by Time.
 *  12 bytes. Request to find entry index by time.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;   ///< Message version (0x00)
  uint8_t type;      ///< Message type: 0=request
  uint16_t year;     ///< Year (UTC)
  uint8_t month;     ///< Month (1-12)
  uint8_t day;       ///< Day (1-31)
  uint8_t hour;      ///< Hour (0-23)
  uint8_t minute;    ///< Minute (0-59)
  uint8_t second;    ///< Second (0-60)
  uint8_t reserved1; ///< Reserved
} UBX_LOG_FINDTIME_req_t;

static_assert(sizeof(UBX_LOG_FINDTIME_req_t) == 10,
              "UBX_LOG_FINDTIME_req_t must be 10 bytes");

/** UBX-LOG-FINDTIME response (0x21 0x0E) - Find Log Entry Response.
 *  8 bytes. Response with entry index.
 */
typedef struct __attribute__((packed)) {
  uint8_t version;      ///< Message version (0x01)
  uint8_t type;         ///< Message type: 1=response
  uint8_t reserved1[2]; ///< Reserved
  uint32_t entryNumber; ///< Entry index (0xFFFFFFFF if not found)
} UBX_LOG_FINDTIME_resp_t;

static_assert(sizeof(UBX_LOG_FINDTIME_resp_t) == 8,
              "UBX_LOG_FINDTIME_resp_t must be 8 bytes");

#endif // ADAFRUIT_UBLOX_TYPEDEF_H
