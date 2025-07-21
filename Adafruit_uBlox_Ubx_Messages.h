/*!
 * @file Adafruit_uBlox_Ubx_Messages.h
 * 
 * Pre-built UBX message payloads for common tasks
 * 
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
 * 
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef ADAFRUIT_UBLOX_UBX_MESSAGES
#define ADAFRUIT_UBLOX_UBX_MESSAGES

#include <Arduino.h>

// NMEA Message Configuration Commands (CFG-MSG)
// Format: {msgClass, msgID, rate_I2C, rate_UART1, rate_UART2, rate_USB, rate_SPI, reserved}

// Enable specific NMEA message sentences to output on the DDC interface
static uint8_t UBX_CFG_MSG_NMEA_GGA_ENABLE[]  = {0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_GLL_ENABLE[] = {0xF0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_GSA_ENABLE[] = {0xF0, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_GSV_ENABLE[] = {0xF0, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_RMC_ENABLE[]  = {0xF0, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_VTG_ENABLE[] = {0xF0, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
// Disable specific NMEA message sentences from outputting on the DDC interface
static uint8_t UBX_CFG_MSG_NMEA_GGA_DISABLE[]  = {0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_GLL_DISABLE[] = {0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_GSA_DISABLE[] = {0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_GSV_DISABLE[] = {0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_RMC_DISABLE[]  = {0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t UBX_CFG_MSG_NMEA_VTG_DISABLE[] = {0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Navigation Rate Configuration Commands (CFG-RATE)
// Format: {measRate_ms (2 bytes), navRate, timeRef}
static uint8_t UBX_CFG_RATE_1HZ[]  = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};
static uint8_t UBX_CFG_RATE_2HZ[]  = {0xF4, 0x01, 0x01, 0x00, 0x01, 0x00}; 
static uint8_t UBX_CFG_RATE_5HZ[]  = {0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
static uint8_t UBX_CFG_RATE_10HZ[] = {0x64, 0x00, 0x01, 0x00, 0x01, 0x00};

#endif // ADAFRUIT_UBLOX_UBX_MESSAGES