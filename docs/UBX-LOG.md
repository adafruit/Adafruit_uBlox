# UBX-LOG (0x21) - Logging Messages

Log creation, deletion, info and retrieval

Messages in the LOG class are used to configure and report status information of the logging and batching features.

---

## UBX-LOG-BATCH (0x21 0x11)
Batched data

**Type:** Polled  
**Firmware:** u-blox 8 / u-blox M8 with protocol version 23.01  
**Length:** 100 bytes

This message combines position, velocity and time solution, including accuracy figures. The output of this message can be requested via UBX-LOG-RETRIEVEBATCH. The content of this message is influenced by UBX-CFG-BATCH. Depending on the flags extraPvt and extraOdo some of the fields in this message may not be valid. This validity information is also indicated in this message via flags of the same name. See Data Batching for more information.

**Note:** During a leap second there may be more or less than 60 seconds in a minute. See the description of leap seconds for details.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | contentValid | X1 | Content validity flags (see bitfield below) |
| 2 | 2 | msgCnt | U2 | Message counter; increments for each sent UBX-LOG-BATCH message |
| 4 | 4 | iTOW | U4 | GPS time of week of the navigation epoch (ms). See the description of iTOW for details. Only valid if extraPvt is set. |
| 8 | 2 | year | U2 | Year (UTC) |
| 10 | 1 | month | U1 | Month, range 1..12 (UTC) |
| 11 | 1 | day | U1 | Day of month, range 1..31 (UTC) |
| 12 | 1 | hour | U1 | Hour of day, range 0..23 (UTC) |
| 13 | 1 | min | U1 | Minute of hour, range 0..59 (UTC) |
| 14 | 1 | sec | U1 | Seconds of minute, range 0..60 (UTC) |
| 15 | 1 | valid | X1 | Validity flags (see bitfield below) |
| 16 | 4 | tAcc | U4 | Time accuracy estimate (UTC) (ns). Only valid if extraPvt is set. |
| 20 | 4 | fracSec | I4 | Fraction of second, range -1e9 .. 1e9 (UTC) (ns) |
| 24 | 1 | fixType | U1 | GNSSfix Type: 0=no fix, 2=2D-fix, 3=3D-fix |
| 25 | 1 | flags | X1 | Fix status flags (see bitfield below) |
| 26 | 1 | flags2 | X1 | Additional flags |
| 27 | 1 | numSV | U1 | Number of satellites used in Nav Solution. Only valid if extraPvt is set. |
| 28 | 4 | lon | I4 | Longitude (deg * 1e-7) |
| 32 | 4 | lat | I4 | Latitude (deg * 1e-7) |
| 36 | 4 | height | I4 | Height above ellipsoid (mm) |
| 40 | 4 | hMSL | I4 | Height above mean sea level (mm). Only valid if extraPvt is set. |
| 44 | 4 | hAcc | U4 | Horizontal accuracy estimate (mm) |
| 48 | 4 | vAcc | U4 | Vertical accuracy estimate (mm). Only valid if extraPvt is set. |
| 52 | 4 | velN | I4 | NED north velocity (mm/s). Only valid if extraPvt is set. |
| 56 | 4 | velE | I4 | NED east velocity (mm/s). Only valid if extraPvt is set. |
| 60 | 4 | velD | I4 | NED down velocity (mm/s). Only valid if extraPvt is set. |
| 64 | 4 | gSpeed | I4 | Ground Speed (2-D) (mm/s) |
| 68 | 4 | headMot | I4 | Heading of motion (2-D) (deg * 1e-5) |
| 72 | 4 | sAcc | U4 | Speed accuracy estimate (mm/s). Only valid if extraPvt is set. |
| 76 | 4 | headAcc | U4 | Heading accuracy estimate (deg * 1e-5). Only valid if extraPvt is set. |
| 80 | 2 | pDOP | U2 | Position DOP (* 0.01). Only valid if extraPvt is set. |
| 82 | 2 | reserved1 | U1[2] | Reserved |
| 84 | 4 | distance | U4 | Ground distance since last reset (m). Only valid if extraOdo is set. |
| 88 | 4 | totalDistance | U4 | Total cumulative ground distance (m). Only valid if extraOdo is set. |
| 92 | 4 | distanceStd | U4 | Ground distance accuracy (1-sigma) (m). Only valid if extraOdo is set. |
| 96 | 4 | reserved2 | U1[4] | Reserved |

### Bitfield: contentValid
| Name | Description |
|------|-------------|
| extraPvt | Extra PVT information is valid. The fields iTOW, tAcc, numSV, hMSL, vAcc, velN, velE, velD, sAcc, headAcc and pDOP are only valid if this flag is set. |
| extraOdo | Odometer data is valid. The fields distance, totalDistance and distanceStd are only valid if this flag is set. Note: the odometer feature itself must also be enabled. |

### Bitfield: valid
| Name | Description |
|------|-------------|
| validDate | 1 = valid UTC Date (see Time Validity section for details) |
| validTime | 1 = valid UTC Time of Day (see Time Validity section for details) |

### Bitfield: flags
| Name | Description |
|------|-------------|
| gnssFixOK | 1 = valid fix (i.e within DOP & accuracy masks) |
| diffSoln | 1 = differential corrections were applied |
| psmState | Power save mode state (see Power Management): 0=PSM is not active, 1=Enabled (an intermediate state before Acquisition state), 2=Acquisition, 3=Tracking, 4=Power optimized tracking, 5=Inactive |

---

## UBX-LOG-CREATE (0x21 0x07)
Create log file

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 8 bytes

This message is used to create an initial logging file and activate the logging subsystem. UBX-ACK-ACK or UBX-ACK-NAK are returned to indicate success or failure. This message does not handle activation of recording or filtering of log entries (see UBX-CFG-LOGFILTER).

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | logCfg | X1 | Config flags (see bitfield below) |
| 2 | 1 | reserved1 | U1 | Reserved |
| 3 | 1 | logSize | U1 | Indicates the size of the log: 0=(maximum safe size), 1=(minimum size), 2=(user-defined, see userDefinedSize below) |
| 4 | 4 | userDefinedSize | U4 | Sets the maximum amount of space in the filestore that can be used by the logging task. This field is only applicable if logSize is set to user-defined. (bytes) |

### Bitfield: logCfg
| Name | Description |
|------|-------------|
| circular | Log is circular (new entries overwrite old ones in a full log) if this bit set |

---

## UBX-LOG-ERASE (0x21 0x03)
Erase logged data

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 bytes

This message deactivates the logging system and erases all logged data. UBX-ACK-ACK or UBX-ACK-NAK are returned to indicate success or failure.

No payload

---

## UBX-LOG-FINDTIME (0x21 0x0E)
### Request
Find index of a log entry based on a given time

**Type:** Input  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 10 bytes

This message can be used for a time-based search of a log. It can find the index of the first log entry with time equal to the given time, otherwise the index of the most recent entry with time less than the given time. This index can then be used with the UBX-LOG-RETRIEVE message to provide time-based retrieval of log entries.

**Notes:**
- Searching a log is effective for a given time later than the base date (January 1st, 2004)
- Searching a log for a given time earlier than the base date will result in an 'entry not found' response
- Searching a log for a given time greater than the last recorded entry's time will return the index of the last recorded entry

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | type | U1 | Message type, 0 for request |
| 2 | 2 | year | U2 | Year (1-65635) of UTC time |
| 4 | 1 | month | U1 | Month (1-12) of UTC time |
| 5 | 1 | day | U1 | Day (1-31) of UTC time |
| 6 | 1 | hour | U1 | Hour (0-23) of UTC time |
| 7 | 1 | minute | U1 | Minute (0-59) of UTC time |
| 8 | 1 | second | U1 | Second (0-60) of UTC time |
| 9 | 1 | reserved1 | U1 | Reserved |

### Response
Response to FINDTIME request

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 8 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x01 for this version) |
| 1 | 1 | type | U1 | Message type, 1 for response |
| 2 | 2 | reserved1 | U1[2] | Reserved |
| 4 | 4 | entryNumber | U4 | Index of the first log entry with time = given time, otherwise index of the most recent entry with time < given time. If 0xFFFFFFFF, no log entry found with time <= given time. The indexing of log entries is zero-based. |

---

## UBX-LOG-INFO (0x21 0x08)
### Poll Request
Poll for log information

**Type:** Poll Request  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 bytes

Upon sending of this message, the receiver returns UBX-LOG-INFO as defined below.

No payload

### Response
Log information

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 48 bytes

This message is used to report information about the logging subsystem.

**Notes:**
- The reported maximum log size will be smaller than that originally specified in LOG-CREATE due to logging and filestore implementation overheads
- Log entries are compressed in a variable length fashion, so it may be difficult to predict log space usage with any precision
- There may be times when the receiver does not have an accurate time (e.g. if the week number is not yet known), in which case some entries will not have a timestamp

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x01 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | filestoreCapacity | U4 | The capacity of the filestore (bytes) |
| 8 | 8 | reserved2 | U1[8] | Reserved |
| 16 | 4 | currentMaxLogSize | U4 | The maximum size the current log is allowed to grow to (bytes) |
| 20 | 4 | currentLogSize | U4 | Approximate amount of space in log currently occupied (bytes) |
| 24 | 4 | entryCount | U4 | Number of entries in the log. Note: for circular logs this value will decrease when a group of entries is deleted to make space for new ones. |
| 28 | 2 | oldestYear | U2 | Oldest entry UTC year (1-65635) or zero if there are no entries with known time |
| 30 | 1 | oldestMonth | U1 | Oldest month (1-12) |
| 31 | 1 | oldestDay | U1 | Oldest day (1-31) |
| 32 | 1 | oldestHour | U1 | Oldest hour (0-23) |
| 33 | 1 | oldestMinute | U1 | Oldest minute (0-59) |
| 34 | 1 | oldestSecond | U1 | Oldest second (0-60) |
| 35 | 1 | reserved3 | U1 | Reserved |
| 36 | 2 | newestYear | U2 | Newest year (1-65635) or zero if there are no entries with known time |
| 38 | 1 | newestMonth | U1 | Newest month (1-12) |
| 39 | 1 | newestDay | U1 | Newest day (1-31) |
| 40 | 1 | newestHour | U1 | Newest hour (0-23) |
| 41 | 1 | newestMinute | U1 | Newest minute (0-59) |
| 42 | 1 | newestSecond | U1 | Newest second (0-60) |
| 43 | 1 | reserved4 | U1 | Reserved |
| 44 | 1 | status | X1 | Log status flags (see bitfield below) |
| 45 | 3 | reserved5 | U1[3] | Reserved |

### Bitfield: status
| Name | Description |
|------|-------------|
| recording | Log entry recording is currently turned on |
| inactive | Logging system not active - no log present |
| circular | The current log is circular |

---

## UBX-LOG-RETRIEVEBATCH (0x21 0x10)
Request batch data

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 with protocol version 23.01  
**Length:** 4 bytes

This message is used to request batched data. Batch entries are returned in chronological order, using one UBX-LOG-BATCH per navigation epoch. The speed of transfer can be maximized by using a high data rate. See Data Batching for more information.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | flags | X1 | Flags (see bitfield below) |
| 2 | 2 | reserved1 | U1[2] | Reserved |

### Bitfield: flags
| Name | Description |
|------|-------------|
| sendMonFirst | Send UBX-MON-BATCH message before sending the UBX-LOG-BATCH message(s) |

---

## UBX-LOG-RETRIEVEPOSEXTRA (0x21 0x0F)
Odometer log entry

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 32 bytes

This message is used to report an odometer log entry.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | entryIndex | U4 | The index of this log entry |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 1 | reserved1 | U1 | Reserved |
| 6 | 2 | year | U2 | Year (1-65635) of UTC time. Will be zero if time not known |
| 8 | 1 | month | U1 | Month (1-12) of UTC time |
| 9 | 1 | day | U1 | Day (1-31) of UTC time |
| 10 | 1 | hour | U1 | Hour (0-23) of UTC time |
| 11 | 1 | minute | U1 | Minute (0-59) of UTC time |
| 12 | 1 | second | U1 | Second (0-60) of UTC time |
| 13 | 3 | reserved2 | U1[3] | Reserved |
| 16 | 4 | distance | U4 | Odometer distance traveled since the last time the odometer was reset by a UBX-NAV-RESETODO |
| 20 | 12 | reserved3 | U1[12] | Reserved |

---

## UBX-LOG-RETRIEVEPOS (0x21 0x0B)
Position fix log entry

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 40 bytes

This message is used to report a position fix log entry.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | entryIndex | U4 | The index of this log entry |
| 4 | 4 | lon | I4 | Longitude (deg * 1e-7) |
| 8 | 4 | lat | I4 | Latitude (deg * 1e-7) |
| 12 | 4 | hMSL | I4 | Height above mean sea level (mm) |
| 16 | 4 | hAcc | U4 | Horizontal accuracy estimate (mm) |
| 20 | 4 | gSpeed | U4 | Ground speed (2-D) (mm/s) |
| 24 | 4 | heading | U4 | Heading (deg * 1e-5) |
| 28 | 1 | version | U1 | Message version (0x00 for this version) |
| 29 | 1 | fixType | U1 | Fix type: 0x01=Dead Reckoning only, 0x02=2D-Fix, 0x03=3D-Fix, 0x04=GNSS + Dead Reckoning combined |
| 30 | 2 | year | U2 | Year (1-65635) of UTC time |
| 32 | 1 | month | U1 | Month (1-12) of UTC time |
| 33 | 1 | day | U1 | Day (1-31) of UTC time |
| 34 | 1 | hour | U1 | Hour (0-23) of UTC time |
| 35 | 1 | minute | U1 | Minute (0-59) of UTC time |
| 36 | 1 | second | U1 | Second (0-60) of UTC time |
| 37 | 1 | reserved1 | U1 | Reserved |
| 38 | 1 | numSV | U1 | Number of satellites used in the position fix |
| 39 | 1 | reserved2 | U1 | Reserved |

---

## UBX-LOG-RETRIEVESTRING (0x21 0x0D)
Byte string log entry

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 16 + 1*byteCount bytes

This message is used to report a byte string log entry.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | entryIndex | U4 | The index of this log entry |
| 4 | 1 | version | U1 | Message version (0x00 for this version) |
| 5 | 1 | reserved1 | U1 | Reserved |
| 6 | 2 | year | U2 | Year (1-65635) of UTC time. Will be zero if time not known |
| 8 | 1 | month | U1 | Month (1-12) of UTC time |
| 9 | 1 | day | U1 | Day (1-31) of UTC time |
| 10 | 1 | hour | U1 | Hour (0-23) of UTC time |
| 11 | 1 | minute | U1 | Minute (0-59) of UTC time |
| 12 | 1 | second | U1 | Second (0-60) of UTC time |
| 13 | 1 | reserved2 | U1 | Reserved |
| 14 | 2 | byteCount | U2 | Size of string in bytes |

**Repeated block (byteCount times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 16 + 1*N | 1 | bytes | U1 | The bytes of the string |

---

## UBX-LOG-RETRIEVE (0x21 0x09)
Request log data

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 12 bytes

This message is used to request logged data (log recording must first be disabled, see UBX-CFG-LOGFILTER). Log entries are returned in chronological order, using the messages UBX-LOG-RETRIEVEPOS and UBX-LOG-RETRIEVESTRING. If the odometer was enabled at the time a position was logged, then message UBX-LOG-RETRIEVEPOSEXTRA will also be used. The maximum number of entries that can be returned in response to a single UBX-LOG-RETRIEVE message is 256. If more entries than this are required the message will need to be sent multiple times with different startNumbers. The retrieve will be stopped if any UBX-LOG message is received. The speed of transfer can be maximized by using a high data rate and temporarily stopping the GPS processing (see UBX-CFG-RST).

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | startNumber | U4 | Index of first log entry to be transferred. If it is larger than the index of the last available log entry, then the first log entry to be transferred is the last available log entry. The indexing of log entries is zero-based. |
| 4 | 4 | entryCount | U4 | Number of log entries to transfer in total including the first entry to be transferred. If it is larger than the log entries available starting from the first entry to be transferred, then only the available log entries are transferred followed by a UBX-ACK-NAK. The maximum is 256. |
| 8 | 1 | version | U1 | Message version (0x00 for this version) |
| 9 | 3 | reserved1 | U1[3] | Reserved |

---

## UBX-LOG-STRING (0x21 0x04)
Store arbitrary string in on-board flash

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 + 1*N bytes

This message can be used to store an arbitrary byte string in the on-board flash memory. The maximum length that can be stored is 256 bytes.

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N*1 | 1 | bytes | U1 | The string of bytes to be logged (maximum 256) |
