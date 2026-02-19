# UBX-RXM (0x02) - Receiver Manager Messages

Satellite Status, RTC Status

Messages in the RXM class are used to output status and result data from the Receiver Manager.

---

## UBX-RXM-IMES (0x02 0x61)
Indoor Messaging System information

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 4 + 44*numTx bytes

This message shows the IMES stations the receiver is currently tracking, their data rate, the signal level, the Doppler (with respect to 1575.4282MHz) and what data (without protocol specific overhead) it has received from these stations so far. This message is sent out at the navigation rate the receiver is currently set to. Therefore it allows users to get an overview on the receiver's current state from the IMES perspective.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | numTx | U1 | Number of transmitters contained in the message |
| 1 | 1 | version | U1 | Message version (0x01 for this version) |
| 2 | 2 | reserved1 | U1[2] | Reserved |

**Repeated block (numTx times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 4 + 44*N | 1 | reserved2 | U1 | Reserved |
| 5 + 44*N | 1 | txId | U1 | Transmitter identifier |
| 6 + 44*N | 3 | reserved3 | U1[3] | Reserved |
| 9 + 44*N | 1 | cno | U1 | Carrier to Noise Ratio (Signal Strength) (dBHz) |
| 10 + 44*N | 2 | reserved4 | U1[2] | Reserved |
| 12 + 44*N | 4 | doppler | I4 | Doppler frequency with respect to 1575.4282MHz [IIIII.FFF Hz] (Hz * 2^-12) |
| 16 + 44*N | 4 | position1_1 | X4 | Position 1 Frame (part 1/2) (see bitfield below) |
| 20 + 44*N | 4 | position1_2 | X4 | Position 1 Frame (part 2/2) (see bitfield below) |
| 24 + 44*N | 4 | position2_1 | X4 | Position 2 Frame (part 1/3) (see bitfield below) |
| 28 + 44*N | 4 | lat | I4 | Latitude, Position 2 Frame (part 2/3) (deg * 180 * 2^-24) |
| 32 + 44*N | 4 | lon | I4 | Longitude, Position 2 Frame (part 3/3) (deg * 360 * 2^-25) |
| 36 + 44*N | 4 | shortIdFrame | X4 | Short ID Frame (see bitfield below) |
| 40 + 44*N | 4 | mediumIdLSB | U4 | Medium ID LSB, Medium ID Frame (part 1/2) |
| 44 + 44*N | 4 | mediumId_2 | X4 | Medium ID Frame (part 2/2) (see bitfield below) |

### Bitfield: position1_1
| Name | Description |
|------|-------------|
| pos1Floor | Floor number [1.0 floor resolution] (Offset: -50 floor) |
| pos1Lat | Latitude [deg * (180 / 2^23)] |

### Bitfield: position1_2
| Name | Description |
|------|-------------|
| pos1Lon | Longitude [deg * (360 / 2^24)] |
| pos1Valid | Position 1 Frame valid |

### Bitfield: position2_1
| Name | Description |
|------|-------------|
| pos2Floor | Floor number [0.5 floor resolution] (Offset: -50 floor) |
| pos2Alt | Altitude [m] (Offset: -95m) |
| pos2Acc | Accuracy Index (0:undef, 1:<7m, 2:<15m, 3:>15m) |
| pos2Valid | Position 2 Frame valid |

### Bitfield: shortIdFrame
| Name | Description |
|------|-------------|
| shortId | Short ID |
| shortValid | Short ID Frame valid |
| shortBoundary | Boundary Bit |

### Bitfield: mediumId_2
| Name | Description |
|------|-------------|
| mediumIdMSB | Medium ID MSB |
| mediumValid | Medium ID Frame valid |
| mediumboundary | Boundary Bit |

---

## UBX-RXM-MEASX (0x02 0x14)
Satellite measurements for RRLP

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 44 + 24*numSV bytes

The message payload data is, where possible and appropriate, according to the Radio Resource LCS (Location Services) Protocol (RRLP). One exception is the satellite and GNSS IDs, which here are given according to the Satellite Numbering scheme. The correct satellites have to be selected and their satellite ID translated accordingly for use in a RRLP Measure Position Response Component. Similarly, the measurement reference time of week has to be forwarded correctly (modulo 14400000 for the 24 LSB GPS measurements variant, modulo 3600000 for the 22 LSB Galileo and Additional Navigation Satellite Systems (GANSS) measurements variant) of the RRLP measure position response to the SMLC.

**Reference:** ETSI TS 144 031 V11.0.0 (2012-10)

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version, currently 0x01 |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | gpsTOW | U4 | GPS measurement reference time (ms) |
| 8 | 4 | gloTOW | U4 | GLONASS measurement reference time (ms) |
| 12 | 4 | bdsTOW | U4 | BeiDou measurement reference time (ms) |
| 16 | 4 | reserved2 | U1[4] | Reserved |
| 20 | 4 | qzssTOW | U4 | QZSS measurement reference time (ms) |
| 24 | 2 | gpsTOWacc | U2 | GPS measurement reference time accuracy (ms * 2^-4) (0xffff = > 4s) |
| 26 | 2 | gloTOWacc | U2 | GLONASS measurement reference time accuracy (ms * 2^-4) (0xffff = > 4s) |
| 28 | 2 | bdsTOWacc | U2 | BeiDou measurement reference time accuracy (ms * 2^-4) (0xffff = > 4s) |
| 30 | 2 | reserved3 | U1[2] | Reserved |
| 32 | 2 | qzssTOWacc | U2 | QZSS measurement reference time accuracy (ms * 2^-4) (0xffff = > 4s) |
| 34 | 1 | numSV | U1 | Number of satellites in repeated block |
| 35 | 1 | flags | U1 | Flags (see bitfield below) |
| 36 | 8 | reserved4 | U1[8] | Reserved |

**Repeated block (numSV times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 44 + 24*N | 1 | gnssId | U1 | GNSS ID (see Satellite Numbering) |
| 45 + 24*N | 1 | svId | U1 | Satellite ID (see Satellite Numbering) |
| 46 + 24*N | 1 | cNo | U1 | carrier noise ratio (0..63) |
| 47 + 24*N | 1 | mpathIndic | U1 | multipath index (according to RRLP) (0 = not measured, 1 = low, 2 = medium, 3 = high) |
| 48 + 24*N | 4 | dopplerMS | I4 | Doppler measurement (m/s * 0.04) |
| 52 + 24*N | 4 | dopplerHz | I4 | Doppler measurement (Hz * 0.2) |
| 56 + 24*N | 2 | wholeChips | U2 | whole value of the code phase measurement (0..1022 for GPS) |
| 58 + 24*N | 2 | fracChips | U2 | fractional value of the code phase measurement (0..1023) |
| 60 + 24*N | 4 | codePhase | U4 | Code phase (ms * 2^-21) |
| 64 + 24*N | 1 | intCodePhase | U1 | Integer (part of the) code phase (ms) |
| 65 + 24*N | 1 | pseuRangeRMSErr | U1 | pseudorange RMS error index (according to RRLP) (0..63) |
| 66 + 24*N | 2 | reserved5 | U1[2] | Reserved |

### Bitfield: flags
| Name | Description |
|------|-------------|
| towSet | TOW set (0 = no, 1 or 2 = yes) |

---

## UBX-RXM-PMREQ (0x02 0x41)
### Version 1 (8 bytes)
Power management request

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 8 bytes

This message requests a power management related task of the receiver.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | duration | U4 | Duration of the requested task (ms). The maximum supported value is 12 days. Set to 0 to wait for a wakeup signal on a pin |
| 4 | 4 | flags | X4 | task flags (see bitfield below) |

#### Bitfield: flags
| Name | Description |
|------|-------------|
| backup | The receiver goes into backup mode for a time period defined by duration, provided that it is not connected to USB |

### Version 2 (16 bytes)
Power management request

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 16 bytes

This message requests a power management related task of the receiver.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | duration | U4 | Duration of the requested task (ms). The maximum supported value is 12 days. Set to 0 to wait for a wakeup signal on a pin |
| 8 | 4 | flags | X4 | task flags (see bitfield below) |
| 12 | 4 | wakeupSources | X4 | Configure pins to wake up the receiver. The receiver wakes up if there is either a falling or a rising edge on one of the configured pins. (see bitfield below) |

#### Bitfield: flags
| Name | Description |
|------|-------------|
| backup | The receiver goes into backup mode for a time period defined by duration, provided that it is not connected to USB |
| force | Force receiver backup while USB is connected. USB interface will be disabled. |

#### Bitfield: wakeupSources
| Name | Description |
|------|-------------|
| uartrx | Wake up the receiver if there is an edge on the UART RX pin |
| extint0 | Wake up the receiver if there is an edge on the EXTINT0 pin |
| extint1 | Wake up the receiver if there is an edge on the EXTINT1 pin |
| spics | Wake up the receiver if there is an edge on the SPI CS pin |

---

## UBX-RXM-RAWX (0x02 0x15)
### Version DATA0 (Protocol version 17)
Multi-GNSS raw measurement data

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 with protocol version 17 (only with Time Sync products)  
**Length:** 16 + 32*numMeas bytes

This message contains the information needed to be able to generate a RINEX 3 multi-GNSS observation file. This message contains pseudorange, Doppler, carrier phase, phase lock and signal quality information for GNSS satellites once signals have been synchronized. This message supports all active GNSS.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 8 | rcvTow | R8 | Measurement time of week in receiver local time approximately aligned to the GPS time system (s). The receiver local time of week, week number and leap second information can be used to translate the time to other time systems. More information about the difference in time systems can be found in the RINEX 3 format documentation. For a receiver operating in GLONASS only mode, UTC time can be determined by subtracting the leapS field from GPS time regardless of whether the GPS leap seconds are valid. |
| 8 | 2 | week | U2 | GPS week number in receiver local time (weeks) |
| 10 | 1 | leapS | I1 | GPS leap seconds (GPS-UTC) (s). This field represents the receiver's best knowledge of the leap seconds offset. A flag is given in the recStat bitfield to indicate if the leap seconds are known. |
| 11 | 1 | numMeas | U1 | Number of measurements to follow |
| 12 | 1 | recStat | X1 | Receiver tracking status bitfield (see bitfield below) |
| 13 | 3 | reserved1 | U1[3] | Reserved |

**Repeated block (numMeas times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 16 + 32*N | 8 | prMes | R8 | Pseudorange measurement (m). GLONASS inter frequency channel delays are compensated with an internal calibration table. |
| 24 + 32*N | 8 | cpMes | R8 | Carrier phase measurement (cycles). The carrier phase initial ambiguity is initialized using an approximate value to make the magnitude of the phase close to the pseudorange measurement. Clock resets are applied to both phase and code measurements in accordance with the RINEX specification. |
| 32 + 32*N | 4 | doMes | R4 | Doppler measurement (positive sign for approaching satellites) (Hz) |
| 36 + 32*N | 1 | gnssId | U1 | GNSS identifier (see Satellite Numbering for a list of identifiers) |
| 37 + 32*N | 1 | svId | U1 | Satellite identifier (see Satellite Numbering) |
| 38 + 32*N | 1 | reserved2 | U1 | Reserved |
| 39 + 32*N | 1 | freqId | U1 | Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13) |
| 40 + 32*N | 2 | locktime | U2 | Carrier phase locktime counter (maximum 64500ms) (ms) |
| 42 + 32*N | 1 | cno | U1 | Carrier-to-noise density ratio (signal strength) (dB-Hz) |
| 43 + 32*N | 1 | prStdev | X1 | Estimated pseudorange measurement standard deviation (see bitfield below) (m * 0.01 * 2^n) |
| 44 + 32*N | 1 | cpStdev | X1 | Estimated carrier phase measurement standard deviation (note a raw value of 0x0F indicates the value is invalid) (see bitfield below) (cycles * 0.004) |
| 45 + 32*N | 1 | doStdev | X1 | Estimated Doppler measurement standard deviation (see bitfield below) (Hz * 0.002 * 2^n) |
| 46 + 32*N | 1 | trkStat | X1 | Tracking status bitfield (see bitfield below) |
| 47 + 32*N | 1 | reserved3 | U1 | Reserved |

### Version DATA1 (Protocol versions 18+)
Multi-GNSS raw measurements

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with ADR or High Precision GNSS or Time Sync products)  
**Length:** 16 + 32*numMeas bytes

This message contains the information needed to be able to generate a RINEX 3 multi-GNSS observation file. This message contains pseudorange, Doppler, carrier phase, phase lock and signal quality information for GNSS satellites once signals have been synchronized. This message supports all active GNSS.

The only difference between this version of the message and the previous version (UBX-RXM-RAWX-DATA0) is the addition of the version field and sigId field.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 8 | rcvTow | R8 | Measurement time of week in receiver local time approximately aligned to the GPS time system (s) |
| 8 | 2 | week | U2 | GPS week number in receiver local time (weeks) |
| 10 | 1 | leapS | I1 | GPS leap seconds (GPS-UTC) (s). This field represents the receiver's best knowledge of the leap seconds offset. A flag is given in the recStat bitfield to indicate if the leap seconds are known. |
| 11 | 1 | numMeas | U1 | Number of measurements to follow |
| 12 | 1 | recStat | X1 | Receiver tracking status bitfield (see bitfield below) |
| 13 | 1 | version | U1 | Message version (0x01 for this version) |
| 14 | 2 | reserved1 | U1[2] | Reserved |

**Repeated block (numMeas times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 16 + 32*N | 8 | prMes | R8 | Pseudorange measurement (m) |
| 24 + 32*N | 8 | cpMes | R8 | Carrier phase measurement (cycles) |
| 32 + 32*N | 4 | doMes | R4 | Doppler measurement (positive sign for approaching satellites) (Hz) |
| 36 + 32*N | 1 | gnssId | U1 | GNSS identifier (see Satellite Numbering) |
| 37 + 32*N | 1 | svId | U1 | Satellite identifier (see Satellite Numbering) |
| 38 + 32*N | 1 | sigId | U1 | New style signal identifier (see Signal Identifiers).(not supported in protocol versions less than 27) |
| 39 + 32*N | 1 | freqId | U1 | Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13) |
| 40 + 32*N | 2 | locktime | U2 | Carrier phase locktime counter (maximum 64500ms) (ms) |
| 42 + 32*N | 1 | cno | U1 | Carrier-to-noise density ratio (signal strength) (dB-Hz) |
| 43 + 32*N | 1 | prStdev | X1 | Estimated pseudorange measurement standard deviation (see bitfield below) (m * 0.01 * 2^n) |
| 44 + 32*N | 1 | cpStdev | X1 | Estimated carrier phase measurement standard deviation (cycles * 0.004) |
| 45 + 32*N | 1 | doStdev | X1 | Estimated Doppler measurement standard deviation (Hz * 0.002 * 2^n) |
| 46 + 32*N | 1 | trkStat | X1 | Tracking status bitfield (see bitfield below) |
| 47 + 32*N | 1 | reserved2 | U1 | Reserved |

### Bitfield: recStat
| Name | Description |
|------|-------------|
| leapSec | Leap seconds have been determined |
| clkReset | Clock reset applied. Typically the receiver clock is changed in increments of integer milliseconds. |

### Bitfield: prStdev
| Name | Description |
|------|-------------|
| prStd | Estimated pseudorange standard deviation |

### Bitfield: cpStdev
| Name | Description |
|------|-------------|
| cpStd | Estimated carrier phase standard deviation |

### Bitfield: doStdev
| Name | Description |
|------|-------------|
| doStd | Estimated Doppler standard deviation |

### Bitfield: trkStat
| Name | Description |
|------|-------------|
| prValid | Pseudorange valid |
| cpValid | Carrier phase valid |
| halfCyc | Half cycle valid |
| subHalfCyc | Half cycle subtracted from phase |

---

## UBX-RXM-RLM (0x02 0x59)
### Short-RLM
Galileo SAR short-RLM report

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 16 bytes

This message contains the contents of any Galileo Search and Rescue (SAR) Short Return Link Message detected by the receiver.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | type | U1 | Message type (0x01 for Short-RLM) |
| 2 | 1 | svId | U1 | Identifier of transmitting satellite (see Satellite Numbering) |
| 3 | 1 | reserved1 | U1 | Reserved |
| 4 | 8 | beacon | U1[8] | Beacon identifier (60 bits), with bytes ordered by earliest transmitted (most significant) first. Top four bits of first byte are zero. |
| 12 | 1 | message | U1 | Message code (4 bits) |
| 13 | 2 | params | U1[2] | Parameters (16 bits), with bytes ordered by earliest transmitted (most significant) first. |
| 15 | 1 | reserved2 | U1 | Reserved |

### Long-RLM
Galileo SAR long-RLM report

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 28 bytes

This message contains the contents of any Galileo Search and Rescue (SAR) Long Return Link Message detected by the receiver.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | type | U1 | Message type (0x02 for Long-RLM) |
| 2 | 1 | svId | U1 | Identifier of transmitting satellite (see Satellite Numbering) |
| 3 | 1 | reserved1 | U1 | Reserved |
| 4 | 8 | beacon | U1[8] | Beacon identifier (60 bits), with bytes ordered by earliest transmitted (most significant) first. Top four bits of first byte are zero. |
| 12 | 1 | message | U1 | Message code (4 bits) |
| 13 | 12 | params | U1[12] | Parameters (96 bits), with bytes ordered by earliest transmitted (most significant) first. |
| 25 | 3 | reserved2 | U1[3] | Reserved |

---

## UBX-RXM-RTCM (0x02 0x32)
RTCM input status

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 20.01, 20.1, 20.2 and 20.3  
**Length:** 8 bytes

This message shows info on a received RTCM input message. It is output upon successful parsing of an RTCM input message, irrespective of whether the RTCM message is supported or not by the receiver.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x02 for this version) |
| 1 | 1 | flags | X1 | RTCM input status flags (see bitfield below) |
| 2 | 2 | subType | U2 | Message subtype, only applicable to u-blox proprietary RTCM message 4072 (not available on all products) |
| 4 | 2 | refStation | U2 | Reference station ID: For RTCM 2.3: Valid range 0-1023. For RTCM 3.3: Reference station ID (DF003) Valid range 0-4095. Reported only for the standard RTCM messages that include the DF003 field and for the u-blox proprietary RTCM messages 4072.x. For all other messages, reports 0xFFFF. |
| 6 | 2 | msgType | U2 | Message type |

### Bitfield: flags
| Name | Description |
|------|-------------|
| crcFailed | 0 when RTCM message received and passed CRC check, 1 when failed, in which case refStation and msgType might be corrupted and misleading |
| msgUsed | 2 = RTCM message used successfully by the receiver, 1 = not used, 0 = do not know |

---

## UBX-RXM-SFRBX (0x02 0x13)
### Version 1 (Protocol version 17)
Broadcast navigation data subframe

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 with protocol version 17 (only with Time Sync products)  
**Length:** 8 + 4*numWords bytes

This message reports a complete subframe of broadcast navigation data decoded from a single signal. The number of data words reported in each message depends on the nature of the signal. See section Broadcast Navigation Data for further details.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | gnssId | U1 | GNSS identifier (see Satellite Numbering) |
| 1 | 1 | svId | U1 | Satellite identifier (see Satellite Numbering) |
| 2 | 1 | reserved1 | U1 | Reserved |
| 3 | 1 | freqId | U1 | Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13) |
| 4 | 1 | numWords | U1 | The number of data words contained in this message (0..16) |
| 5 | 1 | reserved2 | U1 | Reserved |
| 6 | 1 | version | U1 | Message version (0x01 for this version) |
| 7 | 1 | reserved3 | U1 | Reserved |

**Repeated block (numWords times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 8 + 4*N | 4 | dwrd | U4 | The data words |

### Version 2 (Protocol versions 18+)
Broadcast navigation data subframe

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 8 + 4*numWords bytes

This message reports a complete subframe of broadcast navigation data decoded from a single signal. The number of data words reported in each message depends on the nature of the signal. See section Broadcast Navigation Data for further details.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | gnssId | U1 | GNSS identifier (see Satellite Numbering) |
| 1 | 1 | svId | U1 | Satellite identifier (see Satellite Numbering) |
| 2 | 1 | sigId | U1 | Signal identifier (see Signal Identifiers) |
| 3 | 1 | freqId | U1 | Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13) |
| 4 | 1 | numWords | U1 | The number of data words contained in this message (up to 10, for currently supported signals) |
| 5 | 1 | chn | U1 | The tracking channel number the message was received on |
| 6 | 1 | version | U1 | Message version, (0x02 for this version) |
| 7 | 1 | reserved1 | U1 | Reserved |

**Repeated block (numWords times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 8 + 4*N | 4 | dwrd | U4 | The data words |

---

## UBX-RXM-SVSI (0x02 0x20)
SV status info

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 8 + 6*numSV bytes

Status of the receiver manager knowledge about GPS Orbit Validity. This message has only been retained for backwards compatibility; users are recommended to use the UBX-NAV-ORB message in preference.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms). See the description of iTOW for details. |
| 4 | 2 | week | I2 | GPS week number of the navigation epoch (weeks) |
| 6 | 1 | numVis | U1 | Number of visible satellites |
| 7 | 1 | numSV | U1 | Number of per-SV data blocks following |

**Repeated block (numSV times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 8 + 6*N | 1 | svid | U1 | Satellite ID |
| 9 + 6*N | 1 | svFlag | X1 | Information Flags (see bitfield below) |
| 10 + 6*N | 2 | azim | I2 | Azimuth |
| 12 + 6*N | 1 | elev | I1 | Elevation |
| 13 + 6*N | 1 | age | X1 | Age of Almanac and Ephemeris (see bitfield below) |

### Bitfield: svFlag
| Name | Description |
|------|-------------|
| ura | Figure of Merit (URA) range 0..15 |
| healthy | SV healthy flag |
| ephVal | Ephemeris valid |
| almVal | Almanac valid |
| notAvail | SV not available |

### Bitfield: age
| Name | Description |
|------|-------------|
| almAge | Age of ALM in days offset by 4 i.e. the reference time may be in the future: ageOfAlm = (age & 0x0f) - 4 |
| ephAge | Age of EPH in hours offset by 4. i.e. the reference time may be in the future: ageOfEph = ((age & 0xf0) >> 4) - 4 |
