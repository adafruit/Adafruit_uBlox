# Adafruit_uBlox Implementation Plan

## Overview
Prioritized plan for implementing UBX message support in the Adafruit_uBlox library.
Target: u-blox M8 series (SAM-M8Q first), extensible to all GNSS/RTK modules.

Reference docs: `docs/UBX-*.md` (parsed from u-blox8-M8 protocol spec)

## Status Legend
- 🔲 Not started
- 🔨 In progress
- ✅ Implemented
- 🧪 Implemented + tested (hw_test sketch exists and passes)

---

## Phase 1: Core Infrastructure (Must Have)
Essential messages for any GNSS application.

### Already Implemented
| Message | Status | Description | Notes |
|---------|--------|-------------|-------|
| UBX-ACK-ACK (0x05 0x01) | ✅ | Message acknowledged | In `sendMessageWithAck()` |
| UBX-ACK-NAK (0x05 0x00) | ✅ | Message not acknowledged | In `sendMessageWithAck()` |
| UBX-CFG-PRT (0x06 0x00) | ✅ | Port configuration | `setUBXOnly()` uses this |
| UBX-CFG-MSG (0x06 0x01) | ✅ | Set message rate | Used in NMEA enable/disable |

### To Implement
| Message | Status | Priority | Description | Notes |
|---------|--------|----------|-------------|-------|
| UBX-NAV-PVT (0x01 0x07) | 🔲 | **P0** | Position/Velocity/Time | The single most important message. 92 bytes. Replaces NAV-SOL, NAV-POSLLH, NAV-VELNED, NAV-TIMEUTC combined |
| UBX-NAV-STATUS (0x01 0x03) | 🔲 | **P0** | Receiver navigation status | Fix type, DGPS, time validity flags |
| UBX-NAV-DOP (0x01 0x04) | 🔲 | **P0** | Dilution of Precision | GDOP, PDOP, TDOP, VDOP, HDOP, NDOP, EDOP |
| UBX-NAV-SAT (0x01 0x35) | 🔲 | **P0** | Satellite information | Per-satellite CNO, elevation, azimuth, flags |
| UBX-CFG-RST (0x06 0x04) | 🔲 | **P0** | Reset receiver | Hot/warm/cold start, hardware/software reset |
| UBX-CFG-RATE (0x06 0x08) | 🔲 | **P0** | Navigation/measurement rate | Set update rate (1Hz, 5Hz, 10Hz) |
| UBX-CFG-CFG (0x06 0x09) | 🔲 | **P0** | Save/load/clear config | Save to flash, restore defaults |
| UBX-MON-VER (0x0A 0x04) | 🔲 | **P0** | Firmware/protocol version | Identify module type and capabilities |

---

## Phase 2: Navigation Extras (High Value)
Additional navigation data for richer applications.

| Message | Status | Priority | Description | Notes |
|---------|--------|----------|-------------|-------|
| UBX-NAV-POSLLH (0x01 0x02) | 🔲 | P1 | Lat/Lon/Height position | Simpler than PVT, 28 bytes |
| UBX-NAV-VELNED (0x01 0x12) | 🔲 | P1 | NED velocity | Speed, heading, 36 bytes |
| UBX-NAV-TIMEUTC (0x01 0x21) | 🔲 | P1 | UTC time | Standalone time solution |
| UBX-NAV-POSECEF (0x01 0x01) | 🔲 | P1 | ECEF position | For 3D positioning math |
| UBX-NAV-VELECEF (0x01 0x11) | 🔲 | P1 | ECEF velocity | For 3D velocity math |
| UBX-NAV-CLOCK (0x01 0x22) | 🔲 | P1 | Clock solution | Clock bias and drift |
| UBX-NAV-EOE (0x01 0x61) | 🔲 | P1 | End of epoch | Marks end of nav data set |
| UBX-NAV-TIMEGPS (0x01 0x20) | 🔲 | P1 | GPS time | GPS week number + TOW |

---

## Phase 3: Configuration (Important)
Configure receiver behavior.

| Message | Status | Priority | Description | Notes |
|---------|--------|----------|-------------|-------|
| UBX-CFG-NAV5 (0x06 0x24) | 🔲 | P2 | Navigation engine settings | Dynamic model (portable, stationary, pedestrian, automotive, sea, airborne), fix mode (2D/3D/auto) |
| UBX-CFG-GNSS (0x06 0x3E) | 🔲 | P2 | GNSS system configuration | Enable/disable GPS, GLONASS, Galileo, BeiDou, SBAS |
| UBX-CFG-NMEA (0x06 0x17) | 🔲 | P2 | NMEA protocol config | NMEA version, sentence filtering |
| UBX-CFG-ANT (0x06 0x13) | 🔲 | P2 | Antenna control | Antenna power, short/open detect |
| UBX-CFG-SBAS (0x06 0x16) | 🔲 | P2 | SBAS settings | WAAS/EGNOS/MSAS config |
| UBX-CFG-INF (0x06 0x02) | 🔲 | P2 | Information message config | Control debug/warning/error output |
| UBX-CFG-RXM (0x06 0x11) | 🔲 | P2 | Receiver mode | Continuous vs power save |

---

## Phase 4: Power Management
For battery-powered applications.

| Message | Status | Priority | Description | Notes |
|---------|--------|----------|-------------|-------|
| UBX-CFG-PM2 (0x06 0x3B) | 🔲 | P3 | Extended power management | Update period, search period, on/off control |
| UBX-CFG-PMS (0x06 0x86) | 🔲 | P3 | Power mode setup | Full power, balanced, 1Hz/2Hz/4Hz modes |
| UBX-RXM-PMREQ (0x02 0x41) | 🔲 | P3 | Power management request | Enter backup/sleep mode |
| UBX-MON-HW (0x0A 0x09) | 🔲 | P3 | Hardware status | Antenna status, jamming, noise level |

---

## Phase 5: Monitoring & Diagnostics
For debugging and health monitoring.

| Message | Status | Priority | Description | Notes |
|---------|--------|----------|-------------|-------|
| UBX-MON-GNSS (0x0A 0x28) | 🔲 | P4 | GNSS system info | Supported/default/enabled GNSS |
| UBX-MON-HW2 (0x0A 0x0B) | 🔲 | P4 | Extended hardware status | Noise, AGC, CW jamming |
| UBX-MON-IO (0x0A 0x02) | 🔲 | P4 | I/O system status | Bytes pending, RX/TX counts |
| UBX-MON-MSGPP (0x0A 0x06) | 🔲 | P4 | Message parse/process status | Per-protocol message counts |
| UBX-MON-RXBUF (0x0A 0x07) | 🔲 | P4 | Receiver buffer status | Buffer usage per port |
| UBX-MON-TXBUF (0x0A 0x08) | 🔲 | P4 | Transmitter buffer status | Buffer usage per port |
| UBX-SEC-UNIQID (0x27 0x03) | 🔲 | P4 | Unique chip ID | 5-byte unique identifier |
| UBX-INF-* (0x04) | 🔲 | P4 | Information messages | Debug/Warning/Error/Notice/Test strings |

---

## Phase 6: Advanced Features (Nice to Have)
Specialized features for specific use cases.

| Message | Status | Priority | Description | Notes |
|---------|--------|----------|-------------|-------|
| UBX-NAV-HPPOSLLH (0x01 0x14) | 🔲 | P5 | High-precision position | mm-level, for RTK-capable modules |
| UBX-NAV-HPPOSECEF (0x01 0x13) | 🔲 | P5 | High-precision ECEF | mm-level ECEF |
| UBX-NAV-RELPOSNED (0x01 0x3C) | 🔲 | P5 | Relative position | For RTK moving baseline |
| UBX-NAV-SVIN (0x01 0x3B) | 🔲 | P5 | Survey-in data | RTK base station setup |
| UBX-CFG-TMODE3 (0x06 0x71) | 🔲 | P5 | Time mode 3 | RTK base station configuration |
| UBX-NAV-GEOFENCE (0x01 0x39) | 🔲 | P5 | Geofencing status | Up to 4 geofence regions |
| UBX-CFG-GEOFENCE (0x06 0x69) | 🔲 | P5 | Geofence config | Set circular geofence regions |
| UBX-NAV-ODO (0x01 0x09) | 🔲 | P5 | Odometer | Ground distance since reset |
| UBX-NAV-RESETODO (0x01 0x10) | 🔲 | P5 | Reset odometer | Command message |
| UBX-CFG-TP5 (0x06 0x31) | 🔲 | P5 | Time pulse config | PPS output configuration |
| UBX-LOG-* (0x21) | 🔲 | P5 | Data logging | Flash logging features |
| UBX-NAV-TIMELS (0x01 0x26) | 🔲 | P5 | Leap second info | Current and upcoming leap seconds |
| UBX-UPD-SOS (0x09 0x14) | 🔲 | P5 | Save on shutdown | Backup/restore receiver state |

---

## Explicitly Skipped
These are product-specific or rarely needed. Won't implement unless requested.

| Message Class | Reason |
|---------------|--------|
| UBX-ESF (0x10) | ADR/UDR only (dead reckoning sensors) |
| UBX-HNR (0x28) | ADR only (high navigation rate) |
| UBX-MGA (0x13) | Assistance data (online/offline A-GPS) — complex, niche |
| UBX-AID (0x0B) | Deprecated (replaced by UBX-MGA) |
| UBX-NAV-ATT (0x01 0x05) | ADR only (attitude: roll/pitch/heading) |
| UBX-NAV-NMI (0x01 0x28) | Undocumented/internal |
| UBX-NAV-AOPSTATUS (0x01 0x60) | AssistNow Autonomous status — niche |
| UBX-NAV-SLAS (0x01 0x42) | QZSS SLAS only (Japan) |
| UBX-NAV-COV (0x01 0x36) | Covariance matrix — advanced |
| UBX-NAV-EELL (0x01 0x3d) | Error ellipse — advanced |
| UBX-CFG-DOSC (0x06 0x61) | Time & Freq Sync products only |
| UBX-CFG-ESFALG/ESFA/ESFG/ESFWT | ADR/UDR sensor fusion only |
| UBX-CFG-HNR (0x06 0x5C) | ADR only |
| UBX-CFG-SENIF (0x06 0x88) | ADR only |
| UBX-CFG-SMGR (0x06 0x62) | Time & Freq Sync only |
| UBX-CFG-SPT (0x06 0x64) | Sensor interface port config |
| UBX-CFG-TXSLOT (0x06 0x53) | TX timing slots — advanced |
| UBX-CFG-BATCH (0x06 0x93) | Data batching — niche |
| UBX-CFG-DGNSS (0x06 0x70) | HPG (RTK) only |
| UBX-CFG-ESRC (0x06 0x60) | External source ranking — niche |
| UBX-CFG-LOGFILTER (0x06 0x47) | Data logging filter — with LOG class |
| UBX-CFG-TMODE2 (0x06 0x3D) | Superseded by TMODE3 |
| UBX-CFG-RINV (0x06 0x34) | Remote inventory — niche |
| UBX-CFG-USB (0x06 0x1B) | USB port config — niche |
| UBX-CFG-DAT (0x06 0x06) | Datum config — rarely changed |
| UBX-CFG-NAVX5 (0x06 0x23) | Expert nav settings — advanced |
| UBX-TIM-* (0x0D) | Timing products only (except TIM-TP for PPS) |
| UBX-RXM-RAWX/MEASX/SFRBX | Raw measurement — very advanced/niche |
| UBX-NAV-SOL (0x01 0x06) | Deprecated — use NAV-PVT instead |
| UBX-NAV-SVINFO (0x01 0x30) | Deprecated — use NAV-SAT instead |
| UBX-NAV-DGPS (0x01 0x31) | DGPS corrections — niche |
| UBX-NAV-SBAS (0x01 0x32) | SBAS corrections detail — niche |
| UBX-NAV-ORB (0x01 0x34) | Orbit database info — niche |

---

## Implementation Pattern

For each message, implementation involves:

1. **Define message struct** in `Adafruit_uBlox_Ubx_Messages.h`
   - Packed struct matching the payload layout
   - Named constants for class/ID

2. **Add parse method** in `Adafruit_UBX`
   - Register callback or poll-and-parse
   - Populate the struct from raw payload

3. **Add convenience getters** (where useful)
   - e.g., `getLatitude()` returns scaled float from NAV-PVT

4. **Create hw_test sketch** in `hw_tests/`
   - Poll the message, print parsed fields
   - Verify against known values where possible

5. **Update examples** as features complete

## Test Sketch Pattern

Each hw_test should:
```
hw_tests/
  ubx_nav_pvt/ubx_nav_pvt.ino
  ubx_nav_status/ubx_nav_status.ino
  ubx_nav_dop/ubx_nav_dop.ino
  ubx_nav_sat/ubx_nav_sat.ino
  ubx_cfg_rst/ubx_cfg_rst.ino
  ubx_cfg_rate/ubx_cfg_rate.ino
  ubx_cfg_cfg/ubx_cfg_cfg.ino
  ubx_mon_ver/ubx_mon_ver.ino
  ...
```

Each test:
1. Initialize DDC + UBX
2. Poll or subscribe to the message
3. Parse and print all fields in human-readable format
4. Run for N seconds, report success/fail

---

## Architecture Notes

### Struct-based vs Callback-based
Current library uses callbacks. For parsed messages, we should add:
- **Typed structs** for each message (zero-copy parse from buffer)
- **Poll method** that sends a poll request and blocks for response
- **Periodic subscription** via CFG-MSG rate setting

### Buffer Size
Current `MAX_PAYLOAD_SIZE = 64` is too small for:
- NAV-PVT (92 bytes)
- NAV-SAT (8 + 12*numSV, up to ~400+ bytes)
- MON-VER (40 + 30*N, variable)

**Action needed:** Increase buffer or add dynamic allocation strategy.

### Platform Considerations
- AVR (328P): 2KB RAM — only smallest messages feasible, skip SAT/SVINFO
- RP2040/SAMD/ESP32: plenty of RAM, full support
- Library should compile on all platforms; large-payload messages can be conditionally excluded on AVR
