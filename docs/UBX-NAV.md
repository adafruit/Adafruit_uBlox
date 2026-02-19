# UBX-NAV (0x01) - Navigation Results Messages

Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used.

Messages in the NAV class output navigation data such as position, altitude and velocity in various formats. Additionally, status flags and accuracy figures are output. The messages are generated with the configured navigation/measurement rate.

---

## UBX-NAV-AOPSTATUS (0x01 0x60)
AssistNow Autonomous Status

**Type:** Periodic/Polled  
**Length:** 16 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message provides information on the status of the AssistNow Autonomous subsystem on the receiver. A host application can determine the optimal time to shut down the receiver by monitoring the status field for a steady 0.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | aopCfg | U1 | AssistNow Autonomous configuration (see bitfield below) |
| 5 | 1 | status | U1 | AssistNow Autonomous subsystem is idle (0) or running (not 0) |
| 6 | 10 | reserved1 | U1[10] | Reserved |

### aopCfg bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | useAOP | AOP enabled flag |

---

## UBX-NAV-ATT (0x01 0x05)
Attitude Solution

**Type:** Periodic/Polled  
**Length:** 32 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with ADR or UDR products)

**Description:** This message outputs the attitude solution as roll, pitch and heading angles. More details about vehicle attitude can be found in the Vehicle Attitude Output (ADR/UDR) section.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 3 | reserved1 | U1[3] | Reserved |
| 8 | 4 | roll | I4 | Vehicle roll (deg, scale 1e-5) |
| 12 | 4 | pitch | I4 | Vehicle pitch (deg, scale 1e-5) |
| 16 | 4 | heading | I4 | Vehicle heading (deg, scale 1e-5) |
| 20 | 4 | accRoll | U4 | Vehicle roll accuracy (deg, scale 1e-5). If null, roll angle is not available |
| 24 | 4 | accPitch | U4 | Vehicle pitch accuracy (deg, scale 1e-5). If null, pitch angle is not available |
| 28 | 4 | accHeading | U4 | Vehicle heading accuracy (deg, scale 1e-5). If null, heading angle is not available |

---

## UBX-NAV-CLOCK (0x01 0x22)
Clock Solution

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | clkB | I4 | Clock bias (ns) |
| 8 | 4 | clkD | I4 | Clock drift (ns/s) |
| 12 | 4 | tAcc | U4 | Time accuracy estimate (ns) |
| 16 | 4 | fAcc | U4 | Frequency accuracy estimate (ps/s) |

---

## UBX-NAV-COV (0x01 0x36)
Covariance Matrices

**Type:** Periodic/Polled  
**Length:** 64 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message outputs the covariance matrices for the position and velocity solutions in the topocentric coordinate system defined as the local-level North (N), East (E), Down (D) frame. As the covariance matrices are symmetric, only the upper triangular part is output.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 1 | posCovValid | U1 | Position covariance matrix validity flag |
| 6 | 1 | velCovValid | U1 | Velocity covariance matrix validity flag |
| 7 | 9 | reserved1 | U1[9] | Reserved |
| 16 | 4 | posCovNN | R4 | Position covariance matrix value p_NN (m²) |
| 20 | 4 | posCovNE | R4 | Position covariance matrix value p_NE (m²) |
| 24 | 4 | posCovND | R4 | Position covariance matrix value p_ND (m²) |
| 28 | 4 | posCovEE | R4 | Position covariance matrix value p_EE (m²) |
| 32 | 4 | posCovED | R4 | Position covariance matrix value p_ED (m²) |
| 36 | 4 | posCovDD | R4 | Position covariance matrix value p_DD (m²) |
| 40 | 4 | velCovNN | R4 | Velocity covariance matrix value v_NN (m²/s²) |
| 44 | 4 | velCovNE | R4 | Velocity covariance matrix value v_NE (m²/s²) |
| 48 | 4 | velCovND | R4 | Velocity covariance matrix value v_ND (m²/s²) |
| 52 | 4 | velCovEE | R4 | Velocity covariance matrix value v_EE (m²/s²) |
| 56 | 4 | velCovED | R4 | Velocity covariance matrix value v_ED (m²/s²) |
| 60 | 4 | velCovDD | R4 | Velocity covariance matrix value v_DD (m²/s²) |

---

## UBX-NAV-DGPS (0x01 0x31)
DGPS Data Used for NAV

**Type:** Periodic/Polled  
**Length:** 16 + 12×numCh bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message outputs the DGPS correction data that has been applied to the current NAV Solution. See also the notes on the RTCM protocol.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | age | I4 | Age of newest correction data (ms) |
| 8 | 2 | baseId | I2 | DGPS base station identifier |
| 10 | 2 | baseHealth | I2 | DGPS base station health status |
| 12 | 1 | numCh | U1 | Number of channels for which correction data is following |
| 13 | 1 | status | U1 | DGPS correction type status: 0x00=none, 0x01=PR+PRR correction |
| 14 | 2 | reserved1 | U1[2] | Reserved |

**Repeated block (numCh times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 16+12×N | 1 | svid | U1 | Satellite ID |
| 17+12×N | 1 | flags | X1 | Channel number and usage (see bitfield below) |
| 18+12×N | 2 | ageC | U2 | Age of latest correction data (ms) |
| 20+12×N | 4 | prc | R4 | Pseudorange correction (m) |
| 24+12×N | 4 | prrc | R4 | Pseudorange rate correction (m/s) |

### flags bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-3 | channel | GPS channel number this SV is on (channel numbers >15 displayed as 15) |
| 4 | dgpsUsed | 1 = DGPS used for this SV |

---

## UBX-NAV-DOP (0x01 0x04)
Dilution of Precision

**Type:** Periodic/Polled  
**Length:** 18 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Note:** DOP values are dimensionless. All DOP values are scaled by a factor of 100 (e.g., value of 156 = DOP of 1.56).

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 2 | gDOP | U2 | Geometric DOP (scale 0.01) |
| 6 | 2 | pDOP | U2 | Position DOP (scale 0.01) |
| 8 | 2 | tDOP | U2 | Time DOP (scale 0.01) |
| 10 | 2 | vDOP | U2 | Vertical DOP (scale 0.01) |
| 12 | 2 | hDOP | U2 | Horizontal DOP (scale 0.01) |
| 14 | 2 | nDOP | U2 | Northing DOP (scale 0.01) |
| 16 | 2 | eDOP | U2 | Easting DOP (scale 0.01) |

---

## UBX-NAV-EELL (0x01 0x3d)
Position Error Ellipse Parameters

**Type:** Periodic/Polled  
**Length:** 16 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with ADR products)

**Description:** This message outputs the error ellipse parameters for the position solutions.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 1 | reserved1 | U1 | Reserved |
| 6 | 2 | errEllipseOrient | U2 | Orientation of semi-major axis of error ellipse (deg, scale 1e-2, from true north) |
| 8 | 4 | errEllipseMajor | U4 | Semi-major axis of error ellipse (mm) |
| 12 | 4 | errEllipseMinor | U4 | Semi-minor axis of error ellipse (mm) |

---

## UBX-NAV-EOE (0x01 0x61)
End of Epoch

**Type:** Periodic  
**Length:** 4 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message is intended to be used as a marker to collect all navigation messages of an epoch. It is output after all enabled NAV class messages (except UBX-NAV-HNR) and after all enabled NMEA messages.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |

---

## UBX-NAV-GEOFENCE (0x01 0x39)
Geofencing Status

**Type:** Periodic/Polled  
**Length:** 8 + 2×numFences bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message outputs the evaluated states of all configured geofences for the current epoch's position. See the Geofencing description for feature details.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 1 | status | U1 | Geofencing status: 0=not available/not reliable, 1=active |
| 6 | 1 | numFences | U1 | Number of geofences |
| 7 | 1 | combState | U1 | Combined (logical OR) state of all geofences: 0=Unknown, 1=Inside, 2=Outside |

**Repeated block (numFences times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 8+2×N | 1 | state | U1 | Geofence state: 0=Unknown, 1=Inside, 2=Outside |
| 9+2×N | 1 | id | U1 | Geofence ID (0 = not available) |

---

## UBX-NAV-HPPOSECEF (0x01 0x13)
High Precision Position Solution in ECEF

**Type:** Periodic/Polled  
**Length:** 28 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 20.01, 20.1, 20.2 and 20.3

**Description:** See important comments concerning validity of position given in section Navigation Output Filters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 8 | 4 | ecefX | I4 | ECEF X coordinate (cm) |
| 12 | 4 | ecefY | I4 | ECEF Y coordinate (cm) |
| 16 | 4 | ecefZ | I4 | ECEF Z coordinate (cm) |
| 20 | 1 | ecefXHp | I1 | High precision component of ECEF X coordinate (mm, scale 0.1). Must be in range -99..+99. Precise coordinate in cm = ecefX + (ecefXHp × 1e-2) |
| 21 | 1 | ecefYHp | I1 | High precision component of ECEF Y coordinate (mm, scale 0.1). Must be in range -99..+99. Precise coordinate in cm = ecefY + (ecefYHp × 1e-2) |
| 22 | 1 | ecefZHp | I1 | High precision component of ECEF Z coordinate (mm, scale 0.1). Must be in range -99..+99. Precise coordinate in cm = ecefZ + (ecefZHp × 1e-2) |
| 23 | 1 | flags | X1 | Additional flags (see bitfield below) |
| 24 | 4 | pAcc | U4 | Position Accuracy Estimate (mm, scale 0.1) |

### flags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | invalidEcef | 1 = Invalid ecefX, ecefY, ecefZ, ecefXHp, ecefYHp and ecefZHp |

---

## UBX-NAV-HPPOSLLH (0x01 0x14)
High Precision Geodetic Position Solution

**Type:** Periodic/Polled  
**Length:** 36 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 20.01, 20.1, 20.2 and 20.3

**Description:** This message outputs the Geodetic position with high precision in the currently selected ellipsoid. The default is the WGS84 Ellipsoid, but can be changed with the message UBX-CFG-DAT. See important comments concerning validity of position given in section Navigation Output Filters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 2 | reserved1 | U1[2] | Reserved |
| 3 | 1 | flags | X1 | Additional flags (see bitfield below) |
| 4 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 8 | 4 | lon | I4 | Longitude (deg, scale 1e-7) |
| 12 | 4 | lat | I4 | Latitude (deg, scale 1e-7) |
| 16 | 4 | height | I4 | Height above ellipsoid (mm) |
| 20 | 4 | hMSL | I4 | Height above mean sea level (mm) |
| 24 | 1 | lonHp | I1 | High precision component of longitude (deg, scale 1e-9). Must be in range -99..+99. Precise longitude in deg×1e-7 = lon + (lonHp × 1e-2) |
| 25 | 1 | latHp | I1 | High precision component of latitude (deg, scale 1e-9). Must be in range -99..+99. Precise latitude in deg×1e-7 = lat + (latHp × 1e-2) |
| 26 | 1 | heightHp | I1 | High precision component of height above ellipsoid (mm, scale 0.1). Must be in range -9..+9. Precise height in mm = height + (heightHp × 0.1) |
| 27 | 1 | hMSLHp | I1 | High precision component of height above mean sea level (mm, scale 0.1). Must be in range -9..+9. Precise height in mm = hMSL + (hMSLHp × 0.1) |
| 28 | 4 | hAcc | U4 | Horizontal accuracy estimate (mm, scale 0.1) |
| 32 | 4 | vAcc | U4 | Vertical accuracy estimate (mm, scale 0.1) |

### flags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | invalidLlh | 1 = Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp |

---

## UBX-NAV-NMI (0x01 0x28)
Navigation Message Cross-Check Information

**Type:** Periodic/Polled  
**Length:** 16 bytes

**Firmware Support:** u-blox 8 / u-blox M8 with protocol version 22.01

**Description:** Information about the validity of received satellite navigation payload.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x01 for this version) |
| 5 | 4 | reserved1 | U1[4] | Reserved |
| 9 | 1 | gpsNmiFlags | X1 | GPS navigation message cross-check information flags (see bitfield below) |
| 10 | 1 | gpsLsFlags | X1 | GPS leap second cross-check information flags (see bitfield below) |
| 11 | 1 | galNmiFlags | X1 | Galileo navigation message cross-check information flags (see bitfield below) |
| 12 | 1 | galLsFlags | X1 | Galileo leap second cross-check information flags (see bitfield below) |
| 13 | 1 | bdsNmiFlags | X1 | BeiDou navigation message cross-check information flags (see bitfield below) |
| 14 | 1 | bdsLsFlags | X1 | BeiDou leap second cross-check information flags (see bitfield below) |
| 15 | 1 | gloNmiFlags | X1 | GLONASS navigation message cross-check information flags (see bitfield below) |

### gpsNmiFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | wnoCheckedGPS | 1 = week number check performed |
| 1 | wnoInvalidGPS | 1 = week number invalid |
| 2 | UTCORefCheckedGPS | 1 = GPS UTCO reference time check performed |
| 3 | UTCORefInvalidGPS | 1 = GPS UTCO reference time invalid |

### gpsLsFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | lsValGPS | 1 = Leap second value out of range |
| 1 | dnRangeGPS | 1 = Day number value out of range |
| 2 | totRangeGPS | 1 = Data reference TOW out of range |
| 3 | lsEventGPS | 1 = Unexpected leap second event |
| 4 | recNowGPS | 1 = Data received this epoch |

### galNmiFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | wnoCheckedGAL | 1 = week number check performed |
| 1 | wnoInvalidGAL | 1 = week number invalid |

### galLsFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | lsValGAL | 1 = Leap second value out of range |
| 1 | dnRangeGAL | 1 = Day number value out of range |
| 2 | totRangeGAL | 1 = Data reference TOW out of range |
| 3 | lsEventGAL | 1 = Unexpected leap second event |
| 4 | recNowGAL | 1 = Data received this epoch |

### bdsNmiFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | wnoCheckedBDS | 1 = week number check performed |
| 1 | wnoInvalidBDS | 1 = week number invalid |

### bdsLsFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | lsValBDS | 1 = Leap second value out of range |
| 1 | dnRangeBDS | 1 = Day number value out of range |
| 2 | totRangeBDS | 1 = Data reference TOW out of range |
| 3 | lsEventBDS | 1 = Unexpected leap second event |
| 4 | recNowBDS | 1 = Data received this epoch |

### gloNmiFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | wnoCheckedGLO | 1 = week number check performed |
| 1 | wnoInvalidGLO | 1 = week number invalid |

---

## UBX-NAV-ODO (0x01 0x09)
Odometer Solution

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message outputs the traveled distance since last reset (see UBX-NAV-RESETODO) together with an associated estimated accuracy and the total cumulated ground distance (can only be reset by a cold start of the receiver).

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 8 | 4 | distance | U4 | Ground distance since last reset (m) |
| 12 | 4 | totalDistance | U4 | Total cumulative ground distance (m) |
| 16 | 4 | distanceStd | U4 | Ground distance accuracy (1-sigma) (m) |

---

## UBX-NAV-ORB (0x01 0x34)
GNSS Orbit Database Info

**Type:** Periodic/Polled  
**Length:** 8 + 6×numSv bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** Status of the GNSS orbit database knowledge.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x01 for this version) |
| 5 | 1 | numSv | U1 | Number of SVs in the database |
| 6 | 2 | reserved1 | U1[2] | Reserved |

**Repeated block (numSv times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 8+6×N | 1 | gnssId | U1 | GNSS ID |
| 9+6×N | 1 | svId | U1 | Satellite ID |
| 10+6×N | 1 | svFlag | X1 | Information Flags (see bitfield below) |
| 11+6×N | 1 | eph | X1 | Ephemeris data (see bitfield below) |
| 12+6×N | 1 | alm | X1 | Almanac data (see bitfield below) |
| 13+6×N | 1 | otherOrb | X1 | Other orbit data available (see bitfield below) |

### svFlag bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-1 | health | SV health: 0=unknown, 1=healthy, 2=not healthy |
| 2-3 | visibility | SV visibility: 0=unknown, 1=below horizon, 2=above horizon, 3=above elevation mask |

### eph bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-4 | ephUsability | How long the receiver will be able to use the stored ephemeris data: 31=unknown, 30=>450 minutes, 30>n>0=(n-1)×15 to n×15 minutes, 0=no longer usable |
| 5-7 | ephSource | 0=not available, 1=GNSS transmission, 2=external aiding, 3-7=other |

### alm bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-4 | almUsability | How long the receiver will be able to use the stored almanac data: 31=unknown, 30=>30 days, 30>n>0=n-1 to n days, 0=no longer usable |
| 5-7 | almSource | 0=not available, 1=GNSS transmission, 2=external aiding, 3-7=other |

### otherOrb bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-4 | anoAopUsability | How long the receiver will be able to use the orbit data: 31=unknown, 30=>30 days, 30>n>0=n-1 to n days, 0=no longer usable |
| 5-7 | type | Type of orbit data: 0=no data, 1=AssistNow Offline, 2=AssistNow Autonomous, 3-7=other |

---

## UBX-NAV-POSECEF (0x01 0x01)
Position Solution in ECEF

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** See important comments concerning validity of position given in section Navigation Output Filters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | ecefX | I4 | ECEF X coordinate (cm) |
| 8 | 4 | ecefY | I4 | ECEF Y coordinate (cm) |
| 12 | 4 | ecefZ | I4 | ECEF Z coordinate (cm) |
| 16 | 4 | pAcc | U4 | Position Accuracy Estimate (cm) |

---

## UBX-NAV-POSLLH (0x01 0x02)
Geodetic Position Solution

**Type:** Periodic/Polled  
**Length:** 28 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message outputs the Geodetic position in the currently selected ellipsoid. The default is the WGS84 Ellipsoid, but can be changed with the message UBX-CFG-DAT. See important comments concerning validity of position given in section Navigation Output Filters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | lon | I4 | Longitude (deg, scale 1e-7) |
| 8 | 4 | lat | I4 | Latitude (deg, scale 1e-7) |
| 12 | 4 | height | I4 | Height above ellipsoid (mm) |
| 16 | 4 | hMSL | I4 | Height above mean sea level (mm) |
| 20 | 4 | hAcc | U4 | Horizontal accuracy estimate (mm) |
| 24 | 4 | vAcc | U4 | Vertical accuracy estimate (mm) |

---

## UBX-NAV-PVT (0x01 0x07)
Navigation Position Velocity Time Solution

**Type:** Periodic/Polled  
**Length:** 92 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message combines position, velocity and time solution, including accuracy figures. Note that during a leap second there may be more or less than 60 seconds in a minute. See the description of leap seconds for details.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 2 | year | U2 | Year (UTC) |
| 6 | 1 | month | U1 | Month, range 1..12 (UTC) |
| 7 | 1 | day | U1 | Day of month, range 1..31 (UTC) |
| 8 | 1 | hour | U1 | Hour of day, range 0..23 (UTC) |
| 9 | 1 | min | U1 | Minute of hour, range 0..59 (UTC) |
| 10 | 1 | sec | U1 | Seconds of minute, range 0..60 (UTC) |
| 11 | 1 | valid | X1 | Validity flags (see bitfield below) |
| 12 | 4 | tAcc | U4 | Time accuracy estimate (UTC) (ns) |
| 16 | 4 | nano | I4 | Fraction of second, range -1e9..1e9 (UTC) (ns) |
| 20 | 1 | fixType | U1 | GNSSfix Type: 0=no fix, 1=dead reckoning only, 2=2D-fix, 3=3D-fix, 4=GNSS+dead reckoning combined, 5=time only fix |
| 21 | 1 | flags | X1 | Fix status flags (see bitfield below) |
| 22 | 1 | flags2 | X1 | Additional flags (see bitfield below) |
| 23 | 1 | numSV | U1 | Number of satellites used in Nav Solution |
| 24 | 4 | lon | I4 | Longitude (deg, scale 1e-7) |
| 28 | 4 | lat | I4 | Latitude (deg, scale 1e-7) |
| 32 | 4 | height | I4 | Height above ellipsoid (mm) |
| 36 | 4 | hMSL | I4 | Height above mean sea level (mm) |
| 40 | 4 | hAcc | U4 | Horizontal accuracy estimate (mm) |
| 44 | 4 | vAcc | U4 | Vertical accuracy estimate (mm) |
| 48 | 4 | velN | I4 | NED north velocity (mm/s) |
| 52 | 4 | velE | I4 | NED east velocity (mm/s) |
| 56 | 4 | velD | I4 | NED down velocity (mm/s) |
| 60 | 4 | gSpeed | I4 | Ground Speed (2-D) (mm/s) |
| 64 | 4 | headMot | I4 | Heading of motion (2-D) (deg, scale 1e-5) |
| 68 | 4 | sAcc | U4 | Speed accuracy estimate (mm/s) |
| 72 | 4 | headAcc | U4 | Heading accuracy estimate (both motion and vehicle) (deg, scale 1e-5) |
| 76 | 2 | pDOP | U2 | Position DOP (scale 0.01) |
| 78 | 2 | flags3 | X2 | Additional flags (see bitfield below) |
| 80 | 4 | reserved1 | U1[4] | Reserved |
| 84 | 4 | headVeh | I4 | Heading of vehicle (2-D) (deg, scale 1e-5). Only valid when headVehValid is set |
| 88 | 2 | magDec | I2 | Magnetic declination (deg, scale 1e-2). Only supported in ADR 4.10 and later |
| 90 | 2 | magAcc | U2 | Magnetic declination accuracy (deg, scale 1e-2). Only supported in ADR 4.10 and later |

### valid bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | validDate | 1 = valid UTC Date |
| 1 | validTime | 1 = valid UTC time of day |
| 2 | fullyResolved | 1 = UTC time of day has been fully resolved (no seconds uncertainty). Cannot be used to check if time is completely solved |
| 3 | validMag | 1 = valid magnetic declination |

### flags bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0 | gnssFixOK | 1 = valid fix (i.e within DOP & accuracy masks) |
| 1 | diffSoln | 1 = differential corrections were applied |
| 5 | headVehValid | 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode |
| 6-7 | carrSoln | Carrier phase range solution status: 0=no carrier phase range solution, 1=floating ambiguities, 2=fixed ambiguities |

### flags2 bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 5 | confirmedAvai | 1 = information about UTC Date and Time of Day validity confirmation is available (only Protocol Versions 19.00, 19.10, 20.10, 20.20, 20.30, 22.00, 23.00, 23.01, 27, 28) |
| 6 | confirmedDate | 1 = UTC Date validity could be confirmed |
| 7 | confirmedTime | 1 = UTC Time of Day could be confirmed |

### flags3 bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | invalidLlh | 1 = Invalid lon, lat, height and hMSL |
| 1-4 | lastCorrectionAge | Age of the most recently received differential correction: 0=Not available, 1=0-1s, 2=1-2s, 3=2-5s, 4=5-10s, 5=10-15s, 6=15-20s, 7=20-30s, 8=30-45s, 9=45-60s, 10=60-90s, 11=90-120s, ≥12=≥120s |

---

## UBX-NAV-RELPOSNED (0x01 0x3C)
Relative Positioning Information in NED Frame

**Type:** Periodic/Polled  
**Length:** 40 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with High Precision GNSS products)

**Description:** The NED frame is defined as the local topological system at the reference station. The relative position vector components in this message, along with their associated accuracies, are given in that local topological system. This message contains the relative position vector from the Reference Station to the Rover, including accuracy figures, in the local topological system defined at the reference station.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | reserved1 | U1 | Reserved |
| 2 | 2 | refStationId | U2 | Reference Station ID. Must be in the range 0..4095 |
| 4 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 8 | 4 | relPosN | I4 | North component of relative position vector (cm) |
| 12 | 4 | relPosE | I4 | East component of relative position vector (cm) |
| 16 | 4 | relPosD | I4 | Down component of relative position vector (cm) |
| 20 | 1 | relPosHPN | I1 | High-precision North component (mm, scale 0.1). Must be in range -99..+99. Full North in cm = relPosN + (relPosHPN × 1e-2) |
| 21 | 1 | relPosHPE | I1 | High-precision East component (mm, scale 0.1). Must be in range -99..+99. Full East in cm = relPosE + (relPosHPE × 1e-2) |
| 22 | 1 | relPosHPD | I1 | High-precision Down component (mm, scale 0.1). Must be in range -99..+99. Full Down in cm = relPosD + (relPosHPD × 1e-2) |
| 23 | 1 | reserved2 | U1 | Reserved |
| 24 | 4 | accN | U4 | Accuracy of relative position North component (mm, scale 0.1) |
| 28 | 4 | accE | U4 | Accuracy of relative position East component (mm, scale 0.1) |
| 32 | 4 | accD | U4 | Accuracy of relative position Down component (mm, scale 0.1) |
| 36 | 4 | flags | X4 | Flags (see bitfield below) |

### flags bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0 | gnssFixOK | A valid fix (i.e within DOP & accuracy masks) |
| 1 | diffSoln | 1 = differential corrections were applied |
| 2 | relPosValid | 1 = relative position components and accuracies are valid |
| 3-4 | carrSoln | Carrier phase range solution status: 0=no carrier phase range solution, 1=floating ambiguities, 2=fixed ambiguities |
| 5 | isMoving | 1 = receiver is operating in moving baseline mode (not supported in protocol versions <20.3) |
| 6 | refPosMiss | 1 = extrapolated reference position was used to compute moving baseline solution this epoch (not supported in protocol versions <20.3) |
| 7 | refObsMiss | 1 = extrapolated reference observations were used to compute moving baseline solution this epoch (not supported in protocol versions <20.3) |

---

## UBX-NAV-RESETODO (0x01 0x10)
Reset Odometer

**Type:** Command  
**Length:** 0 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message resets the traveled distance computed by the odometer (see UBX-NAV-ODO). UBX-ACK-ACK or UBX-ACK-NAK are returned to indicate success or failure.

No payload.

---

## UBX-NAV-SAT (0x01 0x35)
Satellite Information

**Type:** Periodic/Polled  
**Length:** 8 + 12×numSvs bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message displays information about SVs that are either known to be visible or currently tracked by the receiver. All signal related information corresponds to the subset of signals specified in Signal Identifiers.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x01 for this version) |
| 5 | 1 | numSvs | U1 | Number of satellites |
| 6 | 2 | reserved1 | U1[2] | Reserved |

**Repeated block (numSvs times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 8+12×N | 1 | gnssId | U1 | GNSS identifier (see Satellite Numbering) |
| 9+12×N | 1 | svId | U1 | Satellite identifier (see Satellite Numbering) |
| 10+12×N | 1 | cno | U1 | Carrier to noise ratio (signal strength) (dBHz) |
| 11+12×N | 1 | elev | I1 | Elevation (range: +/-90), unknown if out of range (deg) |
| 12+12×N | 2 | azim | I2 | Azimuth (range 0-360), unknown if elevation is out of range (deg) |
| 14+12×N | 2 | prRes | I2 | Pseudorange residual (m, scale 0.1) |
| 16+12×N | 4 | flags | X4 | Bitmask (see bitfield below) |

### flags bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-2 | qualityInd | Signal quality indicator: 0=no signal, 1=searching signal, 2=signal acquired, 3=signal detected but unusable, 4=code locked and time synchronized, 5-7=code and carrier locked and time synchronized. Note: IMES signals not time synchronized, max value 3 |
| 3 | svUsed | 1 = Signal in the subset specified in Signal Identifiers is currently being used for navigation |
| 4-5 | health | Signal health flag: 0=unknown, 1=healthy, 2=unhealthy |
| 6 | diffCorr | 1 = differential correction data is available for this SV |
| 7 | smoothed | 1 = carrier smoothed pseudorange used |
| 8-10 | orbitSource | Orbit source: 0=no orbit information, 1=ephemeris, 2=almanac, 3=AssistNow Offline, 4=AssistNow Autonomous, 5-7=other orbit information |
| 11 | ephAvail | 1 = ephemeris is available for this SV |
| 12 | almAvail | 1 = almanac is available for this SV |
| 13 | anoAvail | 1 = AssistNow Offline data is available for this SV |
| 14 | aopAvail | 1 = AssistNow Autonomous data is available for this SV |
| 16 | sbasCorrUsed | 1 = SBAS corrections have been used for a signal in the subset specified in Signal Identifiers |
| 17 | rtcmCorrUsed | 1 = RTCM corrections have been used for a signal in the subset specified in Signal Identifiers |
| 18 | slasCorrUsed | 1 = QZSS SLAS corrections have been used for a signal in the subset specified in Signal Identifiers |
| 19 | spartnCorrUsed | 1 = SPARTN corrections have been used for a signal in the subset specified in Signal Identifiers |
| 20 | prCorrUsed | 1 = Pseudorange corrections have been used for a signal in the subset specified in Signal Identifiers |
| 21 | crCorrUsed | 1 = Carrier range corrections have been used for a signal in the subset specified in Signal Identifiers |
| 22 | doCorrUsed | 1 = Range rate (Doppler) corrections have been used for a signal in the subset specified in Signal Identifiers |
| 23 | clasCorrUsed | 1 = CLAS corrections have been used for a signal in the subset specified in Signal Identifiers |

---

## UBX-NAV-SBAS (0x01 0x32)
SBAS Status Data

**Type:** Periodic/Polled  
**Length:** 12 + 12×cnt bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message outputs the status of the SBAS sub system.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | geo | U1 | PRN Number of the GEO where correction and integrity data is used from |
| 5 | 1 | mode | U1 | SBAS Mode: 0=Disabled, 1=Enabled integrity, 3=Enabled test mode |
| 6 | 1 | sys | I1 | SBAS System (WAAS/EGNOS/...): -1=Unknown, 0=WAAS, 1=EGNOS, 2=MSAS, 3=GAGAN, 16=GPS |
| 7 | 1 | service | X1 | SBAS Services available (see bitfield below) |
| 8 | 1 | cnt | U1 | Number of SV data following |
| 9 | 1 | statusFlags | X1 | SBAS status flags (see bitfield below) |
| 10 | 2 | reserved1 | U1[2] | Reserved |

**Repeated block (cnt times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 12+12×N | 1 | svid | U1 | SV ID |
| 13+12×N | 1 | reserved2 | U1 | Reserved |
| 14+12×N | 1 | udre | U1 | Monitoring status |
| 15+12×N | 1 | svSys | U1 | System (WAAS/EGNOS/...), same as SYS |
| 16+12×N | 1 | svService | U1 | Services available, same as SERVICE |
| 17+12×N | 1 | reserved3 | U1 | Reserved |
| 18+12×N | 2 | prc | I2 | Pseudo Range correction in cm |
| 20+12×N | 2 | reserved4 | U1[2] | Reserved |
| 22+12×N | 2 | ic | I2 | Ionosphere correction in cm |

### service bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | Ranging | GEO may be used as ranging source |
| 1 | Corrections | GEO is providing correction data |
| 2 | Integrity | GEO is providing integrity |
| 3 | Testmode | GEO is in test mode |
| 4 | Bad | Problem with signal or broadcast data indicated |

### statusFlags bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-1 | integrityUsed | SBAS integrity used: 0=Unknown, 1=Integrity information is not available or SBAS integrity is not enabled, 2=Receiver uses only GPS satellites for which integrity information is available |

---

## UBX-NAV-SLAS (0x01 0x42)
QZSS L1S SLAS Status Data

**Type:** Periodic/Polled  
**Length:** 20 + 8×cnt bytes

**Firmware Support:** u-blox 8 / u-blox M8 with protocol version 19.2

**Description:** This message outputs the status of the QZSS L1S SLAS sub system.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 3 | reserved1 | U1[3] | Reserved |
| 8 | 4 | gmsLon | I4 | Longitude of the used ground monitoring station (deg, scale 1e-3) |
| 12 | 4 | gmsLat | I4 | Latitude of the used ground monitoring station (deg, scale 1e-3) |
| 16 | 1 | gmsCode | U1 | Code of the used ground monitoring station according to the QZSS SLAS Interface Specification |
| 17 | 1 | qzssSvId | U1 | Satellite identifier of the QZS/GEO whose correction data is used (see Satellite Numbering) |
| 18 | 1 | serviceFlags | X1 | Flags regarding SLAS service (see bitfield below) |
| 19 | 1 | cnt | U1 | Number of pseudorange corrections following |

**Repeated block (cnt times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 20+8×N | 1 | gnssId | U1 | GNSS identifier (see Satellite Numbering) |
| 21+8×N | 1 | svId | U1 | Satellite identifier (see Satellite Numbering) |
| 22+8×N | 1 | reserved2 | U1 | Reserved |
| 23+8×N | 3 | reserved3 | U1[3] | Reserved |
| 26+8×N | 2 | prc | I2 | Pseudorange correction (cm) |

### serviceFlags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | gmsAvailable | 1 = Ground monitoring station available |
| 1 | qzssSvAvailable | 1 = Correction providing QZSS SV available |
| 2 | testMode | 1 = Currently used QZSS SV in test mode |

---

## UBX-NAV-SOL (0x01 0x06)
Navigation Solution Information

**Type:** Periodic/Polled  
**Length:** 52 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**DEPRECATED:** This message has only been retained for backwards compatibility; users are recommended to use the UBX-NAV-PVT message in preference.

**Description:** This message combines position, velocity and time solution in ECEF, including accuracy figures.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | fTOW | I4 | Fractional part of iTOW (range: +/-500000) (ns). The precise GPS time of week in seconds is: (iTOW × 1e-3) + (fTOW × 1e-9) |
| 8 | 2 | week | I2 | GPS week number of the navigation epoch (weeks) |
| 10 | 1 | gpsFix | U1 | GPSfix Type: 0=No Fix, 1=Dead Reckoning only, 2=2D-Fix, 3=3D-Fix, 4=GPS+dead reckoning combined, 5=Time only fix, 6..0xff=reserved |
| 11 | 1 | flags | X1 | Fix Status Flags (see bitfield below) |
| 12 | 4 | ecefX | I4 | ECEF X coordinate (cm) |
| 16 | 4 | ecefY | I4 | ECEF Y coordinate (cm) |
| 20 | 4 | ecefZ | I4 | ECEF Z coordinate (cm) |
| 24 | 4 | pAcc | U4 | 3D Position Accuracy Estimate (cm) |
| 28 | 4 | ecefVX | I4 | ECEF X velocity (cm/s) |
| 32 | 4 | ecefVY | I4 | ECEF Y velocity (cm/s) |
| 36 | 4 | ecefVZ | I4 | ECEF Z velocity (cm/s) |
| 40 | 4 | sAcc | U4 | Speed Accuracy Estimate (cm/s) |
| 44 | 2 | pDOP | U2 | Position DOP (scale 0.01) |
| 46 | 1 | reserved1 | U1 | Reserved |
| 47 | 1 | numSV | U1 | Number of SVs used in Nav Solution |
| 48 | 4 | reserved2 | U1[4] | Reserved |

### flags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | GPSfixOK | 1 = Fix within limits (e.g. DOP & accuracy) |
| 1 | DiffSoln | 1 = DGPS used |
| 2 | WKNSET | 1 = Valid GPS week number |
| 3 | TOWSET | 1 = Valid GPS time of week (iTOW & fTOW) |

---

## UBX-NAV-STATUS (0x01 0x03)
Receiver Navigation Status

**Type:** Periodic/Polled  
**Length:** 16 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** See important comments concerning validity of position given in section Navigation Output Filters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | gpsFix | U1 | GPSfix Type (this value does not qualify a fix as valid and within the limits): 0x00=no fix, 0x01=dead reckoning only, 0x02=2D-fix, 0x03=3D-fix, 0x04=GPS+dead reckoning combined, 0x05=Time only fix, 0x06..0xff=reserved |
| 5 | 1 | flags | X1 | Navigation Status Flags (see bitfield below) |
| 6 | 1 | fixStat | X1 | Fix Status Information (see bitfield below) |
| 7 | 1 | flags2 | X1 | Further information about navigation output (see bitfield below) |
| 8 | 4 | ttff | U4 | Time to first fix (millisecond time tag) (ms) |
| 12 | 4 | msss | U4 | Milliseconds since Startup / Reset (ms) |

### flags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | gpsFixOk | 1 = position and velocity valid and within DOP and ACC Masks |
| 1 | diffSoln | 1 = differential corrections were applied |
| 2 | wknSet | 1 = Week Number valid |
| 3 | towSet | 1 = Time of Week valid |

### fixStat bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0 | diffCorr | 1 = differential corrections available |
| 1 | carrSolnValid | 1 = valid carrSoln |
| 6-7 | mapMatching | Map matching status: 00=none, 01=valid but not used, 10=valid and used, 11=valid and used (enables dead reckoning on sensor unavailability) |

### flags2 bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-2 | psmState | Power save mode state: 0=ACQUISITION (or psm disabled), 1=TRACKING, 2=POWER OPTIMIZED TRACKING, 3=INACTIVE |
| 3-4 | spoofDetState | Spoofing detection state (not supported in protocol versions <18): 0=Unknown or deactivated, 1=No spoofing indicated, 2=Spoofing indicated, 3=Multiple spoofing indications. Note: A value of 1 does not mean the receiver is not spoofed, it states that the detector was not triggered in this epoch |
| 6-7 | carrSoln | Carrier phase range solution status: 0=no carrier phase range solution, 1=floating ambiguities, 2=fixed ambiguities |

---

## UBX-NAV-SVINFO (0x01 0x30)
Space Vehicle Information

**Type:** Periodic/Polled  
**Length:** 8 + 12×numCh bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**DEPRECATED:** This message has only been retained for backwards compatibility; users are recommended to use the UBX-NAV-SAT message in preference.

**Description:** Information about satellites used or visible.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | numCh | U1 | Number of channels |
| 5 | 1 | globalFlags | X1 | Bitmask (see bitfield below) |
| 6 | 2 | reserved1 | U1[2] | Reserved |

**Repeated block (numCh times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 8+12×N | 1 | chn | U1 | Channel number, 255 for SVs not assigned to a channel |
| 9+12×N | 1 | svid | U1 | Satellite ID (see Satellite Numbering) |
| 10+12×N | 1 | flags | X1 | Bitmask (see bitfield below) |
| 11+12×N | 1 | quality | X1 | Bitfield (see bitfield below) |
| 12+12×N | 1 | cno | U1 | Carrier to Noise Ratio (Signal Strength) (dBHz) |
| 13+12×N | 1 | elev | I1 | Elevation in integer degrees (deg) |
| 14+12×N | 2 | azim | I2 | Azimuth in integer degrees (deg) |
| 16+12×N | 4 | prRes | I4 | Pseudo range residual in centimeters (cm) |

### globalFlags bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-2 | chipGen | Chip hardware generation: 0=Antaris/Antaris 4, 1=u-blox 5, 2=u-blox 6, 3=u-blox 7, 4=u-blox 8/u-blox M8 |

### flags bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | svUsed | SV is used for navigation |
| 1 | diffCorr | Differential correction data is available for this SV |
| 2 | orbitAvail | Orbit information is available for this SV (Ephemeris or Almanac) |
| 3 | orbitEph | Orbit information is Ephemeris |
| 4 | unhealthy | SV is unhealthy / shall not be used |
| 5 | orbitAlm | Orbit information is Almanac Plus |
| 6 | orbitAop | Orbit information is AssistNow Autonomous |
| 7 | smoothed | Carrier smoothed pseudorange used |

### quality bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0-2 | qualityInd | Signal Quality indicator: 0=no signal, 1=searching signal, 2=signal acquired, 3=signal detected but unusable, 4=code locked and time synchronized, 5-7=code and carrier locked and time synchronized. Note: IMES signals not time synchronized, max value 3 |

---

## UBX-NAV-SVIN (0x01 0x3B)
Survey-in Data

**Type:** Periodic/Polled  
**Length:** 40 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 20, 20.01, 20.1, 20.2 and 20.3 (only with High Precision GNSS products)

**Description:** This message contains information about survey-in parameters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 8 | 4 | dur | U4 | Passed survey-in observation time (s) |
| 12 | 4 | meanX | I4 | Current survey-in mean position ECEF X coordinate (cm) |
| 16 | 4 | meanY | I4 | Current survey-in mean position ECEF Y coordinate (cm) |
| 20 | 4 | meanZ | I4 | Current survey-in mean position ECEF Z coordinate (cm) |
| 24 | 1 | meanXHP | I1 | Current high-precision survey-in mean position ECEF X coordinate (0.1_mm). Must be in range -99..+99. The current survey-in mean position ECEF X coordinate in cm = meanX + (0.01 × meanXHP) |
| 25 | 1 | meanYHP | I1 | Current high-precision survey-in mean position ECEF Y coordinate (0.1_mm). Must be in range -99..+99. The current survey-in mean position ECEF Y coordinate in cm = meanY + (0.01 × meanYHP) |
| 26 | 1 | meanZHP | I1 | Current high-precision survey-in mean position ECEF Z coordinate (0.1_mm). Must be in range -99..+99. The current survey-in mean position ECEF Z coordinate in cm = meanZ + (0.01 × meanZHP) |
| 27 | 1 | reserved2 | U1 | Reserved |
| 28 | 4 | meanAcc | U4 | Current survey-in mean position accuracy (0.1_mm) |
| 32 | 4 | obs | U4 | Number of position observations used during survey-in |
| 36 | 1 | valid | U1 | Survey-in position validity flag: 1=valid, otherwise 0 |
| 37 | 1 | active | U1 | Survey-in in progress flag: 1=in-progress, otherwise 0 |
| 38 | 2 | reserved3 | U1[2] | Reserved |

---

## UBX-NAV-TIMEBDS (0x01 0x24)
BeiDou Time Solution

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message reports the precise BDS time of the most recent navigation solution including validity flags and an accuracy estimate.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | SOW | U4 | BDS time of week (rounded to seconds) (s) |
| 8 | 4 | fSOW | I4 | Fractional part of SOW (range: +/-500000000) (ns). The precise BDS time of week in seconds is: SOW + fSOW × 1e-9 |
| 12 | 2 | week | I2 | BDS week number of the navigation epoch (weeks) |
| 14 | 1 | leapS | I1 | BDS leap seconds (BDS-UTC) (s) |
| 15 | 1 | valid | X1 | Validity Flags (see bitfield below) |
| 16 | 4 | tAcc | U4 | Time Accuracy Estimate (ns) |

### valid bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | sowValid | 1 = Valid SOW and fSOW |
| 1 | weekValid | 1 = Valid week |
| 2 | leapSValid | 1 = Valid leap second |

---

## UBX-NAV-TIMEGAL (0x01 0x25)
Galileo Time Solution

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message reports the precise Galileo time of the most recent navigation solution including validity flags and an accuracy estimate.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | galTow | U4 | Galileo time of week (rounded to seconds) (s) |
| 8 | 4 | fGalTow | I4 | Fractional part of the Galileo time of week (range: +/-500000000) (ns). The precise Galileo time of week in seconds is: galTow + fGalTow × 1e-9 |
| 12 | 2 | galWno | I2 | Galileo week number (weeks) |
| 14 | 1 | leapS | I1 | Galileo leap seconds (Galileo-UTC) (s) |
| 15 | 1 | valid | X1 | Validity Flags (see bitfield below) |
| 16 | 4 | tAcc | U4 | Time Accuracy Estimate (ns) |

### valid bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | galTowValid | 1 = Valid galTow and fGalTow |
| 1 | galWnoValid | 1 = Valid galWno |
| 2 | leapSValid | 1 = Valid leapS |

---

## UBX-NAV-TIMEGLO (0x01 0x23)
GLONASS Time Solution

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message reports the precise GLO time of the most recent navigation solution including validity flags and an accuracy estimate.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | TOD | U4 | GLONASS time of day (rounded to integer seconds) (s) |
| 8 | 4 | fTOD | I4 | Fractional part of TOD (range: +/-500000000) (ns). The precise GLONASS time of day in seconds is: TOD + fTOD × 1e-9 |
| 12 | 2 | Nt | U2 | Current date (range: 1-1461), starting at 1 from the 1st Jan of the year indicated by N4 and ending at 1461 at the 31st Dec of the third year after that indicated by N4 (days) |
| 14 | 1 | N4 | U1 | Four-year interval number starting from 1996 (1=1996, 2=2000, 3=2004...) |
| 15 | 1 | valid | X1 | Validity flags (see bitfield below) |
| 16 | 4 | tAcc | U4 | Time Accuracy Estimate (ns) |

### valid bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | todValid | 1 = Valid TOD and fTOD |
| 1 | dateValid | 1 = Valid N4 and Nt |

---

## UBX-NAV-TIMEGPS (0x01 0x20)
GPS Time Solution

**Type:** Periodic/Polled  
**Length:** 16 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** This message reports the precise GPS time of the most recent navigation solution including validity flags and an accuracy estimate.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | fTOW | I4 | Fractional part of iTOW (range: +/-500000) (ns). The precise GPS time of week in seconds is: (iTOW × 1e-3) + (fTOW × 1e-9) |
| 8 | 2 | week | I2 | GPS week number of the navigation epoch (weeks) |
| 10 | 1 | leapS | I1 | GPS leap seconds (GPS-UTC) (s) |
| 11 | 1 | valid | X1 | Validity Flags (see bitfield below) |
| 12 | 4 | tAcc | U4 | Time Accuracy Estimate (ns) |

### valid bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | towValid | 1 = Valid GPS time of week (iTOW & fTOW) |
| 1 | weekValid | 1 = Valid GPS week number |
| 2 | leapSValid | 1 = Valid GPS leap seconds |

---

## UBX-NAV-TIMELS (0x01 0x26)
Leap Second Event Information

**Type:** Periodic/Polled  
**Length:** 24 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** Information about the upcoming leap second event if one is scheduled.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 3 | reserved1 | U1[3] | Reserved |
| 8 | 1 | srcOfCurrLs | U1 | Information source for the current number of leap seconds: 0=Default (firmware), 1=Derived from time difference GPS/GLONASS, 2=GPS, 3=SBAS, 4=BeiDou, 5=Galileo, 6=Aided data, 7=Configured, 8=NavIC, 255=Unknown |
| 9 | 1 | currLs | I1 | Current number of leap seconds since start of GPS time (Jan 6, 1980). Reflects how much GPS time is ahead of UTC time. Galileo leap seconds = GPS. BeiDou leap seconds = GPS - 14. GLONASS follows UTC (no leap seconds) (s) |
| 10 | 1 | srcOfLsChange | U1 | Information source for the future leap second event: 0=No source, 2=GPS, 3=SBAS, 4=BeiDou, 5=Galileo, 6=GLONASS, 7=NavIC |
| 11 | 1 | lsChange | I1 | Future leap second change if one is scheduled: +1=positive leap second, -1=negative leap second, 0=no future leap second event scheduled or no information available (s) |
| 12 | 4 | timeToLsEvent | I4 | Number of seconds until the next leap second event, or from the last leap second event if no future event scheduled. If >0 event is in the future, =0 event is now, <0 event is in the past. Valid only if validTimeToLsEvent=1 (s) |
| 16 | 2 | dateOfLsGpsWn | U2 | GPS week number (WN) of the next leap second event or the last one if no future event scheduled. Valid only if validTimeToLsEvent=1 |
| 18 | 2 | dateOfLsGpsDn | U2 | GPS day of week number (DN) for the next leap second event or the last one if no future event scheduled. Valid only if validTimeToLsEvent=1. (GPS and Galileo DN: from 1=Sun to 7=Sat. BeiDou DN: from 0=Sun to 6=Sat) |
| 20 | 3 | reserved2 | U1[3] | Reserved |
| 23 | 1 | valid | X1 | Validity flags (see bitfield below) |

### valid bitfield
| Bit | Name | Description |
|-----|------|-------------|
| 0 | validCurrLs | 1 = Valid current number of leap seconds value |
| 1 | validTimeToLsEvent | 1 = Valid time to next leap second event or from the last leap second event if no future event scheduled |

---

## UBX-NAV-TIMEUTC (0x01 0x21)
UTC Time Solution

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** Note that during a leap second there may be more or less than 60 seconds in a minute. See the description of leap seconds for details.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | tAcc | U4 | Time accuracy estimate (UTC) (ns) |
| 8 | 4 | nano | I4 | Fraction of second, range -1e9..1e9 (UTC) (ns) |
| 12 | 2 | year | U2 | Year, range 1999..2099 (UTC) |
| 14 | 1 | month | U1 | Month, range 1..12 (UTC) |
| 15 | 1 | day | U1 | Day of month, range 1..31 (UTC) |
| 16 | 1 | hour | U1 | Hour of day, range 0..23 (UTC) |
| 17 | 1 | min | U1 | Minute of hour, range 0..59 (UTC) |
| 18 | 1 | sec | U1 | Seconds of minute, range 0..60 (UTC) |
| 19 | 1 | valid | X1 | Validity Flags (see bitfield below) |

### valid bitfield
| Bits | Name | Description |
|------|------|-------------|
| 0 | validTOW | 1 = Valid Time of Week |
| 1 | validWKN | 1 = Valid Week Number |
| 2 | validUTC | 1 = Valid UTC Time |
| 6-7 | utcStandard | UTC standard identifier: 0=Information not available, 1=CRL (Japan), 2=NIST, 3=USNO, 4=BIPM, 5=European laboratories, 6=Former Soviet Union (SU), 7=NTSC (China), 8=NPLI (India), 15=Unknown |

---

## UBX-NAV-VELECEF (0x01 0x11)
Velocity Solution in ECEF

**Type:** Periodic/Polled  
**Length:** 20 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** See important comments concerning validity of position given in section Navigation Output Filters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | ecefVX | I4 | ECEF X velocity (cm/s) |
| 8 | 4 | ecefVY | I4 | ECEF Y velocity (cm/s) |
| 12 | 4 | ecefVZ | I4 | ECEF Z velocity (cm/s) |
| 16 | 4 | sAcc | U4 | Speed accuracy estimate (cm/s) |

---

## UBX-NAV-VELNED (0x01 0x12)
Velocity Solution in NED Frame

**Type:** Periodic/Polled  
**Length:** 36 bytes

**Firmware Support:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01

**Description:** See important comments concerning validity of position given in section Navigation Output Filters.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms) |
| 4 | 4 | velN | I4 | North velocity component (cm/s) |
| 8 | 4 | velE | I4 | East velocity component (cm/s) |
| 12 | 4 | velD | I4 | Down velocity component (cm/s) |
| 16 | 4 | speed | U4 | Speed (3-D) (cm/s) |
| 20 | 4 | gSpeed | U4 | Ground speed (2-D) (cm/s) |
| 24 | 4 | heading | I4 | Heading of motion 2-D (deg, scale 1e-5) |
| 28 | 4 | sAcc | U4 | Speed accuracy Estimate (cm/s) |
| 32 | 4 | cAcc | U4 | Course / Heading accuracy estimate (deg, scale 1e-5) |

---

## End of UBX-NAV Messages

Total messages documented: 34
