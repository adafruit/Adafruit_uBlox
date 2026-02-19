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

#include <Arduino.h>
#include <Stream.h>

#include "Adafruit_uBlox_Ubx_Messages.h"
#include "Adafruit_uBlox_typedef.h"

// UBX protocol constants
#define UBX_SYNC_CHAR_1 0xB5 ///< First UBX protocol sync char (�)
#define UBX_SYNC_CHAR_2 0x62 ///< Second UBX protocol sync char (b)
// UBX ACK Message IDs
#define UBX_ACK_NAK 0x00 ///< Message Not Acknowledged
#define UBX_ACK_ACK 0x01 ///< Message Acknowledged

/*!
 *  @brief  Callback function type for UBX messages - defined at global scope so
 * other classes can use it
 *  @param  msgClass Message class
 *  @param  msgId Message ID
 *  @param  payloadLen Length of payload data
 *  @param  payload Pointer to payload data
 */
typedef void (*UBXMessageCallback)(uint8_t msgClass, uint8_t msgId,
                                   uint16_t payloadLen, uint8_t* payload);

/*!
 * @brief Class for parsing UBX protocol messages from u-blox GPS/RTK modules
 */
class Adafruit_UBX {
 public:
  Adafruit_UBX(Stream& stream);
  ~Adafruit_UBX();
  uint8_t verbose_debug = 0; ///<  0=off, 1=basic, 2=verbose
  // Basic methods
  bool begin();
  bool checkMessages(); // Message parsing
  bool sendMessage(uint8_t msgClass, uint8_t msgId, uint8_t* payload,
                   uint16_t length); // Send a UBX message
  UBXSendStatus sendMessageWithAck(uint8_t msgClass, uint8_t msgId,
                                   uint8_t* payload, uint16_t length,
                                   uint16_t timeout_ms = 500);
  bool poll(uint8_t msgClass, uint8_t msgId, void* response,
            uint16_t responseSize, uint16_t timeout_ms = 1000);
  uint8_t pollNAVSAT(UBX_NAV_SAT_header_t* header, UBX_NAV_SAT_sv_t* svArray,
                     uint8_t maxSvs, uint16_t timeout_ms = 1000);
  bool resetReceiver(uint16_t navBbrMask, uint8_t resetMode);
  bool hotStart();
  bool warmStart();
  bool coldStart();
  bool setRate(uint16_t measRateMs, uint16_t navRate = 1, uint16_t timeRef = 1);
  bool getRate(UBX_CFG_RATE_t* rate);
  bool saveConfig();
  bool loadDefaults();
  uint8_t pollMONVER(UBX_MON_VER_header_t* header, UBX_MON_VER_ext_t* extArray,
                     uint8_t maxExt, uint16_t timeout_ms = 2000);

  // Phase 2 NAV message poll methods
  bool pollNAVPOSLLH(UBX_NAV_POSLLH_t* posllh, uint16_t timeout_ms = 1000);
  bool pollNAVVELNED(UBX_NAV_VELNED_t* velned, uint16_t timeout_ms = 1000);
  bool pollNAVTIMEUTC(UBX_NAV_TIMEUTC_t* timeutc, uint16_t timeout_ms = 1000);
  bool pollNAVPOSECEF(UBX_NAV_POSECEF_t* posecef, uint16_t timeout_ms = 1000);
  bool pollNAVVELECEF(UBX_NAV_VELECEF_t* velecef, uint16_t timeout_ms = 1000);
  bool pollNAVCLOCK(UBX_NAV_CLOCK_t* clock, uint16_t timeout_ms = 1000);
  bool pollNAVEOE(UBX_NAV_EOE_t* eoe, uint16_t timeout_ms = 1000);
  bool pollNAVTIMEGPS(UBX_NAV_TIMEGPS_t* timegps, uint16_t timeout_ms = 1000);

  // Configure port to use UBX protocol only (disable NMEA)
  UBXSendStatus setUBXOnly(UBXPortId portID, bool checkAck = true,
                           uint16_t timeout_ms = 500);

  // Phase 3: CFG message poll/set methods
  // CFG-NAV5: Navigation engine settings
  bool pollCfgNav5(UBX_CFG_NAV5_t* nav5, uint16_t timeout_ms = 1000);
  bool setCfgNav5(UBX_CFG_NAV5_t* nav5);
  bool setDynamicModel(uint8_t model);
  uint8_t getDynamicModel();
  bool setFixMode(uint8_t mode);

  // CFG-GNSS: GNSS configuration
  uint8_t pollCfgGnss(UBX_CFG_GNSS_header_t* header,
                      UBX_CFG_GNSS_block_t* blocks, uint8_t maxBlocks,
                      uint16_t timeout_ms = 1000);
  bool setCfgGnss(UBX_CFG_GNSS_header_t* header, UBX_CFG_GNSS_block_t* blocks,
                  uint8_t numBlocks);
  bool enableGNSS(uint8_t gnssId, bool enable);

  // CFG-NMEA: NMEA protocol configuration
  bool pollCfgNmea(UBX_CFG_NMEA_t* nmea, uint16_t timeout_ms = 1000);
  bool setCfgNmea(UBX_CFG_NMEA_t* nmea);

  // CFG-ANT: Antenna control settings
  bool pollCfgAnt(UBX_CFG_ANT_t* ant, uint16_t timeout_ms = 1000);
  bool setCfgAnt(UBX_CFG_ANT_t* ant);

  // CFG-SBAS: SBAS configuration
  bool pollCfgSbas(UBX_CFG_SBAS_t* sbas, uint16_t timeout_ms = 1000);
  bool setCfgSbas(UBX_CFG_SBAS_t* sbas);
  bool enableSBAS(bool enable);

  // CFG-INF: Info message configuration
  uint8_t pollCfgInf(uint8_t protocolID, UBX_CFG_INF_block_t* blocks,
                     uint8_t maxBlocks, uint16_t timeout_ms = 1000);
  bool setCfgInf(UBX_CFG_INF_block_t* blocks, uint8_t numBlocks);

  // CFG-RXM: Receiver manager configuration
  bool pollCfgRxm(UBX_CFG_RXM_t* rxm, uint16_t timeout_ms = 1000);
  bool setCfgRxm(UBX_CFG_RXM_t* rxm);
  bool setPowerSave(bool enable);

  // Phase 4: Power Management messages
  // CFG-PM2: Extended power management configuration
  bool pollCfgPm2(UBX_CFG_PM2_t* pm2, uint16_t timeout_ms = 1000);
  bool setCfgPm2(UBX_CFG_PM2_t* pm2);

  // CFG-PMS: Power mode setup
  bool pollCfgPms(UBX_CFG_PMS_t* pms, uint16_t timeout_ms = 1000);
  bool setCfgPms(UBX_CFG_PMS_t* pms);
  bool setPowerMode(uint8_t mode);

  // RXM-PMREQ: Power management request (send-only, no ACK)
  bool sendPmreq(uint32_t duration, uint32_t flags);
  bool sendPmreqV1(uint32_t duration, uint32_t flags, uint32_t wakeupSources);

  // MON-HW: Hardware status
  bool pollMonHw(UBX_MON_HW_t* hw, uint16_t timeout_ms = 1000);

  // Phase 5: Monitoring & Diagnostics messages
  // MON-GNSS: GNSS system info
  bool pollMonGnss(UBX_MON_GNSS_t* gnss, uint16_t timeout_ms = 1000);

  // MON-HW2: Extended hardware status
  bool pollMonHw2(UBX_MON_HW2_t* hw2, uint16_t timeout_ms = 1000);

  // MON-IO: I/O system status (variable length, returns port count)
  uint8_t pollMonIo(UBX_MON_IO_port_t* ports, uint8_t maxPorts,
                    uint16_t timeout_ms = 1000);

  // MON-MSGPP: Message parse/process status
  bool pollMonMsgpp(UBX_MON_MSGPP_t* msgpp, uint16_t timeout_ms = 1000);

  // MON-RXBUF: Receiver buffer status
  bool pollMonRxbuf(UBX_MON_RXBUF_t* rxbuf, uint16_t timeout_ms = 1000);

  // MON-TXBUF: Transmitter buffer status
  bool pollMonTxbuf(UBX_MON_TXBUF_t* txbuf, uint16_t timeout_ms = 1000);

  // SEC-UNIQID: Unique chip ID
  bool pollSecUniqid(UBX_SEC_UNIQID_t* uniqid, uint16_t timeout_ms = 1000);

  // INF message support (unsolicited)
  bool wasLastMessageINF();
  uint8_t getLastINFType();
  uint16_t getLastINFString(char* buffer, uint16_t maxLen);

  // Phase 6: Advanced feature messages
  // NAV-ODO: Odometer
  bool pollNavOdo(UBX_NAV_ODO_t* odo, uint16_t timeout_ms = 1000);
  bool resetOdometer();

  // NAV-HPPOSLLH: High precision position (LLH)
  bool pollNavHpposllh(UBX_NAV_HPPOSLLH_t* hppos, uint16_t timeout_ms = 1000);
  bool getHighPrecisionPosition(UBX_NAV_HPPOSLLH_t* hppos);

  // NAV-HPPOSECEF: High precision position (ECEF)
  bool pollNavHpposecef(UBX_NAV_HPPOSECEF_t* hppos, uint16_t timeout_ms = 1000);

  // NAV-RELPOSNED: Relative position NED (RTK)
  bool pollNavRelposned(UBX_NAV_RELPOSNED_t* relpos,
                        uint16_t timeout_ms = 1000);

  // NAV-SVIN: Survey-in status
  bool pollNavSvin(UBX_NAV_SVIN_t* svin, uint16_t timeout_ms = 1000);

  // NAV-GEOFENCE: Geofence status
  uint8_t pollNavGeofence(UBX_NAV_GEOFENCE_header_t* header,
                          UBX_NAV_GEOFENCE_fence_t* fences, uint8_t maxFences,
                          uint16_t timeout_ms = 1000);

  // NAV-TIMELS: Leap second information
  bool pollNavTimels(UBX_NAV_TIMELS_t* timels, uint16_t timeout_ms = 1000);

  // CFG-TMODE3: Time mode 3 (RTK base station)
  bool pollCfgTmode3(UBX_CFG_TMODE3_t* tmode3, uint16_t timeout_ms = 1000);
  bool setCfgTmode3(UBX_CFG_TMODE3_t* tmode3);
  bool startSurveyIn(uint32_t minDurSec, uint32_t accLimitMm);

  // CFG-GEOFENCE: Geofence configuration
  uint8_t pollCfgGeofence(UBX_CFG_GEOFENCE_header_t* header,
                          UBX_CFG_GEOFENCE_fence_t* fences, uint8_t maxFences,
                          uint16_t timeout_ms = 1000);
  bool setCfgGeofence(UBX_CFG_GEOFENCE_header_t* header,
                      UBX_CFG_GEOFENCE_fence_t* fences, uint8_t numFences);
  bool setGeofence(int32_t lat, int32_t lon, uint32_t radiusCm,
                   uint8_t confLvl = 2);
  bool clearGeofence();

  // CFG-TP5: Time pulse configuration
  bool pollCfgTp5(UBX_CFG_TP5_t* tp5, uint8_t tpIdx = 0,
                  uint16_t timeout_ms = 1000);
  bool setCfgTp5(UBX_CFG_TP5_t* tp5);

  // UPD-SOS: Save on shutdown
  bool backupToFlash();
  bool clearBackup();
  uint8_t pollUpdSos(UBX_UPD_SOS_response_t* response,
                     uint16_t timeout_ms = 1000);

  // LOG messages
  bool createLog(uint8_t logSize = 0, bool circular = false,
                 uint32_t userSize = 0);
  bool eraseLog();
  bool pollLogInfo(UBX_LOG_INFO_t* info, uint16_t timeout_ms = 1000);
  bool getLogInfo(UBX_LOG_INFO_t* info);
  bool sendLogRetrieve(uint32_t startIndex, uint32_t count);

  void setMessageCallback(UBXMessageCallback callback); // Set callback function
  UBXMessageCallback onUBXMessage; ///< Callback for message received

 private:
  Stream* _stream; // Stream interface for reading data

  // Buffer for reading messages
  // Platform-adaptive payload buffer size
  // AVR (ATmega328 etc): keep small to save RAM
  // Everything else (ARM, RP2040, ESP32): large enough for NAV-SAT with max
  // sats
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || \
    defined(__AVR_ATmega32U4__)
  static const uint16_t MAX_PAYLOAD_SIZE =
      100; ///< AVR: fits ~7 sats in NAV-SAT
#else
  static const uint16_t MAX_PAYLOAD_SIZE =
      920; ///< ARM/RP2040/ESP32: fits 72 sats in NAV-SAT
#endif
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
  void calculateChecksum(uint8_t* buffer, uint16_t len, uint8_t& checksumA,
                         uint8_t& checksumB);
  void updateChecksum(uint8_t incomingByte);

  // Reset parser state
  void resetParser();

  void printHex(uint8_t val);
  void printHexBuffer(const __FlashStringHelper* label, uint8_t* buf,
                      uint16_t len);

  // Add to private section of Adafruit_UBX.h
  uint8_t _lastMsgClass;       // Class of last message
  uint8_t _lastMsgId;          // ID of last message
  uint16_t _lastPayloadLength; // Length of last message payload
  uint8_t _lastPayload[8];     // Buffer for small payloads (like ACK messages)
};

#endif // ADAFRUIT_UBX_H
