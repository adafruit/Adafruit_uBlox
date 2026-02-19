# UBX-MON (0x0A) - Monitoring Messages

Communication Status, Stack Usage, Task Status

Messages in the MON class are used to report the receiver status, such as stack usage, I/O subsystem statistics etc.

---

## UBX-MON-BATCH (0x0A 0x32)
Data batching buffer status

**Type:** Polled  
**Firmware:** u-blox 8 / u-blox M8 with protocol version 23.01  
**Length:** 12 bytes

This message contains status information about the batching buffer. It can be polled and it can also be sent by the receiver as a response to a UBX-LOG-RETRIEVEBATCH message before the UBX-LOG-BATCH messages. See Data Batching for more information.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 2 | fillLevel | U2 | Current buffer fill level, i.e. number of epochs currently stored |
| 6 | 2 | dropsAll | U2 | Number of dropped epochs since startup. Note: changing the batching configuration will reset this counter. |
| 8 | 2 | dropsSinceMon | U2 | Number of dropped epochs since last MON-BATCH message |
| 10 | 2 | nextMsgCnt | U2 | The next retrieved UBX-LOG-BATCH will have this msgCnt value. |

---

## UBX-MON-GNSS (0x0A 0x28)
Information message major GNSS selection

**Type:** Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 8 bytes

This message reports major GNSS selection. It does this by means of bit masks in U1 fields. Each bit in a bit mask corresponds to one major GNSS. Augmentation systems are not reported.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | supported | X1 | A bit mask showing the major GNSS that can be supported by this receiver (see bitfield below) |
| 2 | 1 | defaultGnss | X1 | A bit mask showing the default major GNSS selection. If the default major GNSS selection is currently configured in the efuse for this receiver, it takes precedence over the default major GNSS selection configured in the executing firmware of this receiver. (see bitfield below) |
| 3 | 1 | enabled | X1 | A bit mask showing the current major GNSS selection enabled for this receiver (see bitfield below) |
| 4 | 1 | simultaneous | U1 | Maximum number of concurrent major GNSS that can be supported by this receiver |
| 5 | 3 | reserved1 | U1[3] | Reserved |

### Bitfield: supported
| Name | Description |
|------|-------------|
| GPSSup | GPS is supported |
| GlonassSup | GLONASS is supported |
| BeidouSup | BeiDou is supported |
| GalileoSup | Galileo is supported |

### Bitfield: defaultGnss
| Name | Description |
|------|-------------|
| GPSDef | GPS is default-enabled |
| GlonassDef | GLONASS is default-enabled |
| BeidouDef | BeiDou is default-enabled |
| GalileoDef | Galileo is default-enabled |

### Bitfield: enabled
| Name | Description |
|------|-------------|
| GPSEna | GPS is enabled |
| GlonassEna | GLONASS is enabled |
| BeidouEna | BeiDou is enabled |
| GalileoEna | Galileo is enabled |

---

## UBX-MON-HW2 (0x0A 0x0B)
Extended hardware status

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 28 bytes

Status of different aspects of the hardware such as Imbalance, Low-Level Configuration and POST Results.

The first four parameters of this message represent the complex signal from the RF front end. The following rules of thumb apply:
- The smaller the absolute value of the variable ofsI and ofsQ, the better.
- Ideally, the magnitude of the I-part (magI) and the Q-part (magQ) of the complex signal should be the same.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | ofsI | I1 | Imbalance of I-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance) |
| 1 | 1 | magI | U1 | Magnitude of I-part of complex signal, scaled (0 = no signal, 255 = max. magnitude) |
| 2 | 1 | ofsQ | I1 | Imbalance of Q-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance) |
| 3 | 1 | magQ | U1 | Magnitude of Q-part of complex signal, scaled (0 = no signal, 255 = max. magnitude) |
| 4 | 1 | cfgSource | U1 | Source of low-level configuration (114 = ROM, 111 = OTP, 112 = config pins, 102 = flash image) |
| 5 | 3 | reserved1 | U1[3] | Reserved |
| 8 | 4 | lowLevCfg | U4 | Low-level configuration (obsolete in protocol versions greater than 15) |
| 12 | 8 | reserved2 | U1[8] | Reserved |
| 20 | 4 | postStatus | U4 | POST status word |
| 24 | 4 | reserved3 | U1[4] | Reserved |

---

## UBX-MON-HW (0x0A 0x09)
Hardware status

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 60 bytes

Status of different aspects of the hardware, such as antenna, PIO/peripheral pins, noise level, automatic gain control (AGC)

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | pinSel | X4 | Mask of pins set as peripheral/PIO |
| 4 | 4 | pinBank | X4 | Mask of pins set as bank A/B |
| 8 | 4 | pinDir | X4 | Mask of pins set as input/output |
| 12 | 4 | pinVal | X4 | Mask of pins value low/high |
| 16 | 2 | noisePerMS | U2 | Noise level as measured by the GPS core |
| 18 | 2 | agcCnt | U2 | AGC Monitor, as percentage of maximum gain, range 0 to 8191 (100%) |
| 20 | 1 | aStatus | U1 | Status of the antenna supervisor state machine (0=INIT, 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN) |
| 21 | 1 | aPower | U1 | Current power status of antenna (0=OFF, 1=ON, 2=DONTKNOW) |
| 22 | 1 | flags | X1 | Flags (see bitfield below) |
| 23 | 1 | reserved1 | U1 | Reserved |
| 24 | 4 | usedMask | X4 | Mask of pins that are used by the virtual pin manager |
| 28 | 17 | VP | U1[17] | Array of pin mappings for each of the 17 physical pins |
| 45 | 1 | cwSuppression | U1 | CW interference suppression level, scaled (0 = no CW jamming, 255 = strong CW jamming) |
| 46 | 2 | reserved2 | U1[2] | Reserved |
| 48 | 4 | pinIrq | X4 | Mask of pins value using the PIO Irq |
| 52 | 4 | pullH | X4 | Mask of pins value using the PIO pull high resistor |
| 56 | 4 | pullL | X4 | Mask of pins value using the PIO pull low resistor |

### Bitfield: flags
| Name | Description |
|------|-------------|
| rtcCalib | RTC is calibrated |
| safeBoot | Safeboot mode (0 = inactive, 1 = active) |
| jammingState | Output from jamming/interference monitor (0 = unknown or feature disabled or flag unavailable, 1 = ok - no significant jamming, 2 = warning - interference visible but fix OK, 3 = critical - interference visible and no fix). This flag is deprecated in protocol versions that support UBX-SEC-SIG (version 0x02) and always reported as 0; instead jammingState in UBX-SEC-SIG should be monitored. |
| xtalAbsent | RTC xtal has been determined to be absent (not supported in protocol versions less than 18) |

---

## UBX-MON-IO (0x0A 0x02)
I/O system status

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 + 20*N bytes

The size of the message is determined by the number of ports 'N' the receiver supports, i.e. on u-blox 5 the number of ports is 6.

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N*20 | 4 | rxBytes | U4 | Number of bytes ever received |
| 4 + 20*N | 4 | txBytes | U4 | Number of bytes ever sent |
| 8 + 20*N | 2 | parityErrs | U2 | Number of 100 ms timeslots with parity errors |
| 10 + 20*N | 2 | framingErrs | U2 | Number of 100 ms timeslots with framing errors |
| 12 + 20*N | 2 | overrunErrs | U2 | Number of 100 ms timeslots with overrun errors |
| 14 + 20*N | 2 | breakCond | U2 | Number of 100 ms timeslots with break conditions |
| 16 + 20*N | 4 | reserved1 | U1[4] | Reserved |

---

## UBX-MON-MSGPP (0x0A 0x06)
Message parse and process status

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 120 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 16 | msg1 | U2[8] | Number of successfully parsed messages for each protocol on port0 |
| 16 | 16 | msg2 | U2[8] | Number of successfully parsed messages for each protocol on port1 |
| 32 | 16 | msg3 | U2[8] | Number of successfully parsed messages for each protocol on port2 |
| 48 | 16 | msg4 | U2[8] | Number of successfully parsed messages for each protocol on port3 |
| 64 | 16 | msg5 | U2[8] | Number of successfully parsed messages for each protocol on port4 |
| 80 | 16 | msg6 | U2[8] | Number of successfully parsed messages for each protocol on port5 |
| 96 | 24 | skipped | U4[6] | Number skipped bytes for each port |

---

## UBX-MON-PATCH (0x0A 0x27)
### Poll Request
Poll request for installed patches

**Type:** Poll Request  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 bytes

No payload

### Response
Installed patches

**Type:** Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 4 + 16*nEntries bytes

This message reports information about patches installed and currently enabled on the receiver. It does not report on patches installed and then disabled. An enabled patch is considered active when the receiver executes from the code space where the patch resides on. For example, a ROM patch is reported active only when the system runs from ROM.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 2 | version | U2 | Message version (0x0001 for this version) |
| 2 | 2 | nEntries | U2 | Total number of reported patches |

**Repeated block (nEntries times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 4 + 16*N | 4 | patchInfo | X4 | Status information about the reported patch (see bitfield below) |
| 8 + 16*N | 4 | comparatorNumber | U4 | The number of the comparator |
| 12 + 16*N | 4 | patchAddress | U4 | The address that is targeted by the patch |
| 16 + 16*N | 4 | patchData | U4 | The data that is inserted at the patchAddress |

#### Bitfield: patchInfo
| Name | Description |
|------|-------------|
| activated | 1: the patch is active, 0: otherwise |
| location | Indicates where the patch is stored. 0: eFuse, 1: ROM, 2: BBR, 3: file system |

---

## UBX-MON-RXBUF (0x0A 0x07)
Receiver buffer status

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 24 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 12 | pending | U2[6] | Number of bytes pending in receiver buffer for each target |
| 12 | 6 | usage | U1[6] | Maximum usage receiver buffer during the last sysmon period for each target (%) |
| 18 | 6 | peakUsage | U1[6] | Maximum usage receiver buffer for each target (%) |

---

## UBX-MON-RXR (0x0A 0x21)
Receiver status information

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 1 byte

The receiver ready message is sent when the receiver changes from or to backup mode.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | flags | X1 | Receiver status flags (see bitfield below) |

### Bitfield: flags
| Name | Description |
|------|-------------|
| awake | not in backup mode |

---

## UBX-MON-SMGR (0x0A 0x2E)
Synchronization manager status

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 16 bytes

This message reports the status of internal and external oscillators and sources as well as whether GNSS is used for disciplining.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | iTOW | U4 | Time of the week (ms) |
| 8 | 2 | intOsc | X2 | A bit mask, indicating the status of the local oscillator (see bitfield below) |
| 10 | 2 | extOsc | X2 | A bit mask, indicating the status of the external oscillator (see bitfield below) |
| 12 | 1 | discSrc | U1 | Disciplining source identifier: 0=internal oscillator, 1=GNSS, 2=EXTINT0, 3=EXTINT1, 4=internal oscillator measured by the host, 5=external oscillator measured by the host |
| 13 | 1 | gnss | X1 | A bit mask, indicating the status of the GNSS (see bitfield below) |
| 14 | 1 | extInt0 | X1 | A bit mask, indicating the status of the external input 0 (see bitfield below) |
| 15 | 1 | extInt1 | X1 | A bit mask, indicating the status of the external input 1 (see bitfield below) |

### Bitfield: intOsc
| Name | Description |
|------|-------------|
| intOscState | State of the oscillator: 0=autonomous operation, 1=calibration ongoing, 2=oscillator is steered by the host, 3=idle state |
| intOscCalib | 1 = oscillator gain is calibrated |
| intOscDisc | 1 = signal is disciplined |

### Bitfield: extOsc
| Name | Description |
|------|-------------|
| extOscState | State of the oscillator: 0=autonomous operation, 1=calibration ongoing, 2=oscillator is steered by the host, 3=idle state |
| extOscCalib | 1 = oscillator gain is calibrated |
| extOscDisc | 1 = signal is disciplined |

### Bitfield: gnss
| Name | Description |
|------|-------------|
| gnssAvail | 1 = GNSS is present |

### Bitfield: extInt0
| Name | Description |
|------|-------------|
| extInt0Avail | 1 = signal present at this input |
| extInt0Type | Source type: 0=frequency, 1=time |
| extInt0FeedBack | This source is used as feedback of the external oscillator |

### Bitfield: extInt1
| Name | Description |
|------|-------------|
| extInt1Avail | 1 = signal present at this input |
| extInt1Type | Source type: 0=frequency, 1=time |
| extInt1FeedBack | This source is used as feedback of the external oscillator |

---

## UBX-MON-SPT (0x0A 0x2F)
Sensor production test

**Type:** Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with ADR or UDR products)  
**Length:** 4 + 12*numRes + 4*numSensor bytes

This message reports the state of, and measurements made during, sensor self-tests. This message can also be used to retrieve information about detected sensor(s) and driver(s) used. This message is only supported if a sensor is directly connected to the u-blox chip. This includes modules that contain IMUs.

**Note:** This message shows the status of the last self-test since sensor startup. The self-test results are not stored in non-volatile memory.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x01 for this version) |
| 1 | 1 | numSensor | U1 | Number of sensors reported in this message |
| 2 | 1 | numRes | U1 | Number of result items reported in this message |
| 3 | 1 | reserved1 | U1 | Reserved |

**Repeated block (numSensor times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 4 + 4*N | 1 | sensorId | U1 | Sensor ID. See list of IDs in description below |
| 5 + 4*N | 1 | drvVer | X1 | Version information (see bitfield below) |
| 6 + 4*N | 1 | testState | U1 | State of one sensor's test: 0=test not yet started, 1=test started but not yet finished, 2=test did not finish due to error during execution, 3=test finished normally, test data is available |
| 7 + 4*N | 1 | drvFileName | U1 | 0 if the active driver is loaded from image, last character of the file name if it is loaded from separate file. |

**Sensor IDs:**
- 1: ST LSM6DS0 6-axis IMU with temperature sensor
- 2: Invensense MPU6500 6-axis IMU with temperature sensor
- 3: Bosch BMI160 6-axis IMU with temperature sensor
- 7: ST LSM6DS3 6-axis IMU with temperature sensor
- 9: Bosch SMI130 6-axis IMU with temperature sensor
- 12: MPU6515, 6-axis inertial sensor from Invensense
- 13: ST LSM6DSL 6-axis IMU with temperature sensor
- 14: SMG130, 3-axis gyroscope with temperature sensor from Bosch
- 15: SMI230, 6-axis IMU with temperature sensor from Bosch
- 16: BMI260, 6-axis IMU with temperature sensor from Bosch
- 17: ICM330DLC, 6-axis IMU with temperature sensor from ST
- 18: LSM6DSR, 6-axis IMU with 85 deg temperature sensor from ST
- 19: ICM42605, 6-axis IMU with 85 deg temperature sensor from InvenSense TDK
- 20: IIM42652, 6-axis IMU with 105 deg temperature sensor from InvenSense TDK
- 21: BMI320, 6-axis IMU with 85 deg temperature sensor from Bosch
- 22: IAM20680HT, 6-axis IMU with 105 deg temperature sensor from InvenSense TDK
- 23: LSM6DSOW, 6-axis IMU with 85 deg temperature sensor from ST

**Repeated block (numRes times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 4 + 12*N + 4*numSensor | 2 | sensorIdRes | U2 | Sensor ID; eligible values are the same as in sensorIdState field |
| 6 + 12*N + 4*numSensor | 2 | sensorType | U2 | Sensor type and axis (see sensor type values below) |
| 8 + 12*N + 4*numSensor | 2 | resType | U2 | The type of result stored in the value field (see result type values below) |
| 10 + 12*N + 4*numSensor | 2 | reserved2 | U1[2] | Reserved |
| 12 + 12*N + 4*numSensor | 4 | value | I4 | Value of the specific test result |

**Sensor Type Values:**
- 5: Gyroscope z axis
- 12: Gyroscope temperature
- 13: Gyroscope y axis
- 14: Gyroscope x axis
- 16: Accelerometer x axis
- 17: Accelerometer y axis
- 18: Accelerometer z axis
- 19: Barometer
- 22: Magnetometer x axis
- 23: Magnetometer y axis
- 24: Magnetometer z axis
- 25: Barometer temperature

**Result Type Values:**
- 1: Measurement without self-test offset (raw and unscaled digital value)
- 2: Measurement with positive self-test offset (raw and unscaled digital value)
- 3: Measurement with negative self-test offset (raw and unscaled digital value)
- 4: Minimum off-to-positive to pass self-test, as deduced from on-chip trimming information
- 5: Maximum off-to-positive to pass self-test, as deduced from on-chip trimming information
- 6: Minimum negative-to-positive to pass self-test, as deduced from on-chip trimming information
- 7: Maximum negative-to-positive to pass self-test, as deduced from on-chip trimming information
- 8: Self-test passed; test passed if value = 1 and failed if 0. Used if the decision is read out from the sensor itself.

### Bitfield: drvVer
| Name | Description |
|------|-------------|
| drvVerMaj | Driver major version |
| drvVerMin | Driver minor version |

---

## UBX-MON-TXBUF (0x0A 0x08)
Transmitter buffer status

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 28 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 12 | pending | U2[6] | Number of bytes pending in transmitter buffer for each target |
| 12 | 6 | usage | U1[6] | Maximum usage transmitter buffer during the last sysmon period for each target (%) |
| 18 | 6 | peakUsage | U1[6] | Maximum usage transmitter buffer for each target (%) |
| 24 | 1 | tUsage | U1 | Maximum usage of transmitter buffer during the last sysmon period for all targets (%) |
| 25 | 1 | tPeakusage | U1 | Maximum usage of transmitter buffer for all targets (%) |
| 26 | 1 | errors | X1 | Error bitmask (see bitfield below) |
| 27 | 1 | reserved1 | U1 | Reserved |

### Bitfield: errors
| Name | Description |
|------|-------------|
| limit | Buffer limit of corresponding target reached |
| mem | Memory Allocation error |
| alloc | Allocation error (TX buffer full) |

---

## UBX-MON-VER (0x0A 0x04)
### Poll Request
Poll receiver and software version

**Type:** Poll Request  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 bytes

No payload

### Response
Receiver and software version

**Type:** Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 40 + 30*N bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 30 | swVersion | CH[30] | Nul-terminated software version string |
| 30 | 10 | hwVersion | CH[10] | Nul-terminated hardware version string |

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 40 + 30*N | 30 | extension | CH[30] | Extended software information strings. A series of nul-terminated strings. Each extension field is 30 characters long and contains varying software information. Not all extension fields may appear. Examples: software version string of the underlying ROM (when the receiver's firmware is running from flash), the firmware version, the supported protocol version, the module identifier, the flash information structure (FIS) file information, the supported major GNSS, the supported augmentation systems. See Firmware and protocol versions for details. |
