# UBX-CFG Protocol Messages (0x06)

**Configuration Input Messages** — Configure the receiver.

Messages in the CFG class can be used to configure the receiver and poll current configuration values. Any messages in the CFG class sent to the receiver are either acknowledged (with message UBX-ACK-ACK) if processed successfully or rejected (with message UBX-ACK-NAK) if processing unsuccessfully.

---

## UBX-CFG-ANT (0x06 0x13)
Antenna control settings

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+

This message allows the user to configure the antenna supervisor. The antenna supervisor can be used to detect the status of an active antenna and control it. It can be used to turn off the supply to the antenna in the event of a short circuit (for example) or to manage power consumption in power save mode.

Refer to UBX-MON-HW for a description of the fields in the message used to obtain the status of the antenna.

Note that not all pins can be used for antenna supervisor operation, the default pins are recommended.

**Length:** 4 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 2 | flags | X2 | Antenna flag mask (see bitfield below) |
| 2 | 2 | pins | X2 | Antenna pin configuration (see bitfield below) |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | svcs | Enable antenna supply voltage control signal |
| 1 | scd | Enable short circuit detection |
| 2 | ocd | Enable open circuit detection |
| 3 | pdwnOnSCD | Power down antenna supply if short circuit is detected. (only in combination with bit 1) |
| 4 | recovery | Enable automatic recovery from short state |

**Bitfield: pins**

| Bits | Name | Description |
|------|------|-------------|
| 0-4 | pinSwitch | PIO-pin used for switching antenna supply |
| 5-9 | pinSCD | PIO-pin used for detecting a short in the antenna supply |
| 10-14 | pinOCD | PIO-pin used for detecting open/not connected antenna |
| 15 | reconfig | If set to one, and this command is sent to the receiver, the receiver will reconfigure the pins as specified |

---

## UBX-CFG-BATCH (0x06 0x93)
Get/set data batching configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 with protocol version 23.01  
**Length:** 8 bytes

Gets or sets the configuration for data batching. See Data Batching for more information.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | flags | X1 | Flags (see bitfield below) |
| 2 | 2 | bufSize | U2 | Size of buffer in number of epochs to store |
| 4 | 2 | notifThrs | U2 | Buffer fill level that triggers PIO notification, in number of epochs stored |
| 6 | 1 | pioId | U1 | PIO ID to use for buffer level notification |
| 7 | 1 | reserved1 | U1 | Reserved |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | enable | Enable data batching |
| 1 | extraPvt | Store extra PVT information. The fields iTOW, tAcc, numSV, hMSL, vAcc, velN, velE, velD, sAcc, headAcc and pDOP in UBX-LOG-BATCH are only valid if this flag is set |
| 2 | extraOdo | Store odometer data. The fields distance, totalDistance and distanceStd in UBX-LOG-BATCH are only valid if this flag is set. Note: the odometer feature itself must also be enabled |
| 3 | pioEnable | Enable PIO notification |
| 4 | pioActiveLow | PIO is active low |

---

## UBX-CFG-CFG (0x06 0x09)
Clear, save and load configurations

**Type:** Command  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 12 or 13 bytes

See Receiver configuration for a detailed description on how receiver configuration should be used. The three masks are made up of individual bits, each bit indicating the sub-section of all configurations on which the corresponding action shall be carried out. The reserved bits in the masks must be set to '0'.

Note that commands can be combined. The sequence of execution is clear, save, load.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | clearMask | X4 | Mask with configuration sub-sections to clear (load defaults to permanent) (see bitfield below) |
| 4 | 4 | saveMask | X4 | Mask with configuration sub-sections to save to non-volatile memory |
| 8 | 4 | loadMask | X4 | Mask with configuration sub-sections to load from non-volatile memory |
| 12 | 1 | deviceMask | X1 | **Optional.** Mask which selects the memory devices for this command (see bitfield below) |

**Bitfield: clearMask / saveMask / loadMask**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | ioPort | Communications port settings. Modifying this sub-section results in an IO system reset. Because of this undefined data may be output for a short period of time after receiving the message |
| 1 | msgConf | Message configuration |
| 2 | infMsg | INF message configuration |
| 3 | navConf | Navigation configuration |
| 4 | rxmConf | Receiver Manager configuration |
| 8 | senConf | Sensor interface configuration (not supported in protocol versions less than 19) |
| 9 | rinvConf | Remote inventory configuration |
| 10 | antConf | Antenna configuration |
| 11 | logConf | Logging configuration |
| 12 | ftsConf | FTS configuration. Only applicable to the FTS product variant |

**Bitfield: deviceMask**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | devBBR | Battery backed RAM |
| 1 | devFlash | Flash |
| 2 | devEEPROM | EEPROM |
| 4 | devSpiFlash | SPI Flash |

---

## UBX-CFG-DAT (0x06 0x06)

### Set user-defined datum
**Type:** Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 44 bytes

For more information see the description of Geodetic Systems and Frames.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 8 | majA | R8 | - | m | Semi-major axis (accepted range = 6,300,000.0 to 6,500,000.0 meters) |
| 8 | 8 | flat | R8 | - | - | 1.0 / flattening (accepted range is 0.0 to 500.0) |
| 16 | 4 | dX | R4 | - | m | X axis shift at the origin (accepted range is +/- 5000.0 meters) |
| 20 | 4 | dY | R4 | - | m | Y axis shift at the origin (accepted range is +/- 5000.0 meters) |
| 24 | 4 | dZ | R4 | - | m | Z axis shift at the origin (accepted range is +/- 5000.0 meters) |
| 28 | 4 | rotX | R4 | - | s | Rotation about the X axis (accepted range is +/- 20.0 milli-arc seconds) |
| 32 | 4 | rotY | R4 | - | s | Rotation about the Y axis (accepted range is +/- 20.0 milli-arc seconds) |
| 36 | 4 | rotZ | R4 | - | s | Rotation about the Z axis (accepted range is +/- 20.0 milli-arc seconds) |
| 40 | 4 | scale | R4 | - | ppm | Scale change (accepted range is 0.0 to 50.0 parts per million) |

### Get currently defined datum
**Type:** Get  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 52 bytes

Returns the parameters of the currently defined datum. If no user-defined datum has been set, this will default to WGS84.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 2 | datumNum | U2 | - | - | Datum number: 0 = WGS84, 0xFFFF = user-defined |
| 2 | 6 | datumName | CH[6] | - | - | ASCII string: WGS84 or USER |
| 8 | 8 | majA | R8 | - | m | Semi-major axis (accepted range = 6,300,000.0 to 6,500,000.0 meters) |
| 16 | 8 | flat | R8 | - | - | 1.0 / flattening (accepted range is 0.0 to 500.0) |
| 24 | 4 | dX | R4 | - | m | X axis shift at the origin (accepted range is +/- 5000.0 meters) |
| 28 | 4 | dY | R4 | - | m | Y axis shift at the origin (accepted range is +/- 5000.0 meters) |
| 32 | 4 | dZ | R4 | - | m | Z axis shift at the origin (accepted range is +/- 5000.0 meters) |
| 36 | 4 | rotX | R4 | - | s | Rotation about the X axis (accepted range is +/- 20.0 milli-arc seconds) |
| 40 | 4 | rotY | R4 | - | s | Rotation about the Y axis (accepted range is +/- 20.0 milli-arc seconds) |
| 44 | 4 | rotZ | R4 | - | s | Rotation about the Z axis (accepted range is +/- 20.0 milli-arc seconds) |
| 48 | 4 | scale | R4 | - | ppm | Scale change (accepted range is 0.0 to 50.0 parts per million) |

---

## UBX-CFG-DGNSS (0x06 0x70)
DGNSS configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+ (only with High Precision GNSS products)  
**Length:** 4 bytes

This message allows the user to configure the DGNSS configuration of the receiver.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | dgnssMode | U1 | Specifies differential mode: 2 = RTK float (No attempts are made to fix ambiguities), 3 = RTK fixed (Ambiguities are fixed whenever possible) |
| 1 | 3 | reserved1 | U1[3] | Reserved |

---

## UBX-CFG-DOSC (0x06 0x61)
Disciplined oscillator configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 16+ (only with Time & Frequency Sync products)  
**Length:** 4 + 32×numOsc bytes

**⚠️ Note:** This message is unlikely to be useful for a general GNSS library. Time & Frequency Sync product specific.

This message allows the characteristics of the internal or external oscillator to be described to the receiver. The gainVco and gainUncertainty parameters are normally set using the calibration process initiated using UBX-TIM-VCOCAL. The behavior of the system can be badly affected by setting the wrong values, so customers are advised to only change these parameters with care.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 1 | numOsc | U1 | - | - | Number of oscillators to configure (affects length of this message) |
| 2 | 2 | reserved1 | U1[2] | - | - | Reserved |

**Repeated block (numOsc times):**

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 4+32×N | 1 | oscId | U1 | - | - | Id of oscillator: 0 = internal oscillator, 1 = external oscillator |
| 5+32×N | 1 | reserved2 | U1 | - | - | Reserved |
| 6+32×N | 2 | flags | X2 | - | - | Flags (see bitfield below) |
| 8+32×N | 4 | freq | U4 | - | Hz | Nominal frequency of source |
| 12+32×N | 4 | phaseOffset | I4 | - | ps | Intended phase offset of the oscillator relative to the leading edge of the time pulse |
| 16+32×N | 4 | withTemp | U4 | 2^-8 | ppb | Oscillator stability limit over operating temperature range (must be > 0) |
| 20+32×N | 4 | withAge | U4 | 2^-8 | ppb/year | Oscillator stability with age (must be > 0) |
| 24+32×N | 2 | timeToTemp | U2 | - | s | The minimum time that it could take for a temperature variation to move the oscillator frequency by 'withTemp' (must be > 0) |
| 26+32×N | 2 | reserved3 | U1[2] | - | - | Reserved |
| 28+32×N | 4 | gainVco | I4 | 2^-16 | ppb/raw LSB change | Oscillator control gain/slope; change of frequency per unit change in raw control |
| 32+32×N | 1 | gainUncertainty | U1 | 2^-8 | - | Relative uncertainty (1 standard deviation) of oscillator control gain/slope |
| 33+32×N | 3 | reserved4 | U1[3] | - | - | Reserved |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | isCalibrated | 1 if the oscillator gain is calibrated, 0 if not |
| 1-5 | controlIf | Communication interface for oscillator control: 0 = Custom DAC attached to receiver's I2C, 1 = Microchip MCP4726 (12 bit DAC) attached to receiver's I2C, 2 = TI DAC8571 (16 bit DAC) attached to receiver's I2C, 13 = 12 bit DAC attached to host, 14 = 14 bit DAC attached to host, 15 = 16 bit DAC attached to host. Note that for DACs attached to the host, the host must monitor UBX-TIM-DOSC messages and pass the supplied raw values on to the DAC |

---

## UBX-CFG-ESFALG (0x06 0x56)
Get/set IMU-mount misalignment configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15.01+ (only with ADR or UDR products)  
**Length:** 12 bytes

**⚠️ Note:** ADR/UDR product specific. Not useful for general GNSS applications.

Get/set the IMU-mount misalignment configuration (rotation from installation-frame to the IMU-frame). A detailed description on how to compose this configuration is given in the ADR Installation section for ADR products and UDR Installation section for UDR products.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 4 | bitfield | U4 | - | - | Bitfield (see below) |
| 4 | 4 | yaw | U4 | 1e-2 | deg | User-defined IMU-mount yaw angle [0, 36000], e.g. for 60.00 degree yaw angle the configured value would be 6000 |
| 8 | 2 | pitch | I2 | 1e-2 | deg | User-defined IMU-mount pitch angle [-9000, 9000], e.g. for 60.00 degree pitch angle the configured value would be 6000 |
| 10 | 2 | roll | I2 | 1e-2 | deg | User-defined IMU-mount roll angle [-18000, 18000], e.g. for 60.00 degree roll angle the configured value would be 6000 |

**Bitfield: bitfield**

| Bits | Name | Description |
|------|------|-------------|
| 0-7 | version | Message version (0x00 for this version) |
| 8 | doAutoMntAlg | Only supported on certain products. Enable/disable automatic IMU-mount alignment (0: Disabled, 1: Enabled). This flag can only be used with modules containing an internal IMU |

---

## UBX-CFG-ESFA (0x06 0x4C)
Get/set the Accelerometer (A) sensor configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 19+ (only with UDR products)  
**Length:** 20 bytes

**⚠️ Note:** UDR product specific. Not useful for general GNSS applications.

Get/set the configuration for the accelerometer sensor required for External Sensor Fusion (ESF) based navigation. More details can be found in the Accelerometer Configuration section.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 9 | reserved1 | U1[9] | - | - | Reserved |
| 10 | 1 | accelRmsThdl | U1 | 2^-6 | m/s² | Accelerometer RMS threshold below which automatically estimated accelerometer noise-level (accuracy) is updated |
| 11 | 1 | frequency | U1 | - | Hz | Nominal accelerometer sensor data sampling frequency |
| 12 | 2 | latency | U2 | - | ms | Accelerometer sensor data latency due to e.g. CAN bus |
| 14 | 2 | accuracy | U2 | 1e-4 | m/s² | Accelerometer sensor data accuracy |
| 16 | 4 | reserved2 | U1[4] | - | - | Reserved |

---

## UBX-CFG-ESFG (0x06 0x4D)
Get/set the Gyroscope (G) sensor configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 19+ (only with UDR products)  
**Length:** 20 bytes

**⚠️ Note:** UDR product specific. Not useful for general GNSS applications.

Get/set the configuration for the gyroscope sensor required for External Sensor Fusion (ESF) based navigation. More details can be found in the Gyroscope Configuration section.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 7 | reserved1 | U1[7] | - | - | Reserved |
| 8 | 2 | tcTableSaveRate | U2 | - | s | Temperature-dependent gyroscope bias table saving update rate |
| 10 | 1 | gyroRmsThdl | U1 | 2^-8 | deg/s | Gyroscope sensor RMS threshold below which automatically estimated gyroscope noise-level (accuracy) is updated |
| 11 | 1 | frequency | U1 | - | Hz | Nominal gyroscope sensor data sampling frequency |
| 12 | 2 | latency | U2 | - | ms | Gyroscope sensor data latency due to e.g. CAN bus |
| 14 | 2 | accuracy | U2 | 1e-3 | deg/s | Gyroscope sensor data accuracy |
| 16 | 4 | reserved2 | U1[4] | - | - | Reserved |

---

## UBX-CFG-ESFWT (0x06 0x82)
Get/set wheel-tick configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15.01+ (only with ADR products)  
**Length:** 32 bytes

**⚠️ Note:** ADR product specific. Not useful for general GNSS applications.

Get/set the wheel-tick configuration for GWT or GAWT solution. Further information on the configuration parameters is given in the Automotive Dead Reckoning (ADR) chapter. This field can only be used with modules supporting analog wheel-tick signals and containing an internal IMU.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 1 | flags1 | X1 | - | - | Flags (see bitfield below) |
| 2 | 1 | flags2 | X1 | - | - | Flags (see bitfield below) |
| 3 | 1 | reserved1 | U1[1] | - | - | Reserved |
| 4 | 4 | wtFactor | U4 | 1e-6 | - | Wheel-tick scale factor to obtain distance [m] from wheel-ticks (0 = not set) |
| 8 | 4 | wtQuantError | U4 | 1e-6 | m (or m/s) | Wheel-tick quantization. If useWtSpeed is set then this is interpreted as the speed measurement error RMS |
| 12 | 4 | wtCountMax | U4 | - | - | Wheel-tick counter maximum value (rollover - 1). See notes in spec for details |
| 16 | 2 | wtLatency | U2 | - | ms | Wheel-tick data latency due to e.g. CAN bus |
| 18 | 1 | wtFrequency | U1 | - | Hz | Nominal wheel-tick data frequency (0 = not set) |
| 19 | 1 | flags3 | X1 | - | - | Flags (see bitfield below) |
| 20 | 2 | speedDeadBand | U2 | - | cm/s | Speed sensor dead band (0 = not set) |
| 22 | 10 | reserved2 | U1[10] | - | - | Reserved |

**Bitfield: flags1**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | combineTicks | Use combined rear wheel-ticks instead of the single tick |
| 1 | useWtSpeed | Use speed measurements (data type 11 in ESF-MEAS) instead of single ticks (data type 10) |
| 2 | dirPinPol | Only supported on certain products. Direction pin polarity: 0 = High signal level means forward direction, 1 = High signal level means backward direction |
| 3 | useWtPin | Use wheel-tick pin for speed measurement |

**Bitfield: flags2**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | autoWtCountMaxOff | Disable automatic estimation of maximum absolute wheel-tick counter value (0: enabled, 1: disabled). (Not supported in protocol versions less than 19) |
| 1 | autoDirPinPolOff | Only supported on certain products. Disable automatic wheel-tick direction pin polarity detection (0: enabled, 1: disabled). (Not supported in protocol versions less than 19) |
| 2 | autoSoftwareWtOff | Only supported on certain products. Disable automatic use of wheel-tick or speed data received over the software interface if available (0: enabled, 1: disabled). (Not supported in protocol versions less than 19) |
| 3 | autoUseWtSpeedOff | Disable automatic receiver reconfiguration for processing speed data instead of wheel-tick data if no wheel-tick data are available but speed data were detected (0: enabled, 1: disabled). (Not supported in protocol versions less than 19) |

**Bitfield: flags3**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | cntBothEdges | Only supported on certain products. Count both rising and falling edges on wheel-tick signal (only relevant if wheel-tick is measured by the u-blox receiver). Only turn on this feature if the wheel-tick signal has 50% duty cycle |

---

## UBX-CFG-ESRC (0x06 0x60)
External synchronization source configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 16+ (only with Time & Frequency Sync products)  
**Length:** 4 + 36×numSources bytes

**⚠️ Note:** Time & Frequency Sync product specific. Not useful for general GNSS applications.

External time or frequency source configuration. The stability of time and frequency sources is described using different fields, see sourceType field documentation.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 1 | numSources | U1 | - | - | Number of sources (affects length of this message) |
| 2 | 2 | reserved1 | U1[2] | - | - | Reserved |

**Repeated block (numSources times):**

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 4+36×N | 1 | extInt | U1 | - | - | EXTINT index of this source (0 for EXTINT0 and 1 for EXTINT1) |
| 5+36×N | 1 | sourceType | U1 | - | - | Source type: 0 = none, 1 = frequency source, 2 = time source, 3 = feedback from external oscillator |
| 6+36×N | 2 | flags | X2 | - | - | Flags (see bitfield below) |
| 8+36×N | 4 | freq | U4 | 2^-2 | Hz | Nominal frequency of source |
| 12+36×N | 4 | reserved2 | U1[4] | - | - | Reserved |
| 16+36×N | 4 | withTemp | U4 | 2^-8 | ppb | Oscillator stability limit over operating temperature range (must be > 0). Only used if sourceType is 1 |
| 20+36×N | 4 | withAge | U4 | 2^-8 | ppb/year | Oscillator stability with age (must be > 0). Only used if sourceType is 1 |
| 24+36×N | 2 | timeToTemp | U2 | - | s | The minimum time that it could take for a temperature variation to move the oscillator frequency by 'withTemp' (must be > 0). Only used if sourceType is 1 |
| 26+36×N | 2 | maxDevLifeTime | U2 | - | ppb | Maximum frequency deviation during lifetime (must be > 0). Only used if sourceType is 1 |
| 28+36×N | 4 | offset | I4 | - | ns | Phase offset of signal. Only used if sourceType is 2 |
| 32+36×N | 4 | offsetUncertainty | U4 | - | ns | Uncertainty of phase offset (one standard deviation). Only used if sourceType is 2 |
| 36+36×N | 4 | jitter | U4 | - | ns/s | Phase jitter (must be > 0). Only used if sourceType is 2 |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | polarity | Polarity of signal: 0 = leading edge is rising edge, 1 = leading edge is falling edge |
| 1 | gnssUtc | Time base of timing signal: 0 = GNSS (as specified in CFG-TP5 or GPS if CFG-TP5 indicates UTC), 1 = UTC. Only used if sourceType is 2 |

---

## UBX-CFG-GEOFENCE (0x06 0x69)
Geofencing configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 18+  
**Length:** 8 + 12×numFences bytes

Gets or sets the geofencing configuration. See the Geofencing description for feature details.

If the receiver is sent a valid new configuration, it will respond with a UBX-ACK-ACK message and immediately change to the new configuration. Otherwise the receiver will reject the request, by issuing a UBX-ACK-NAK and continuing operation with the previous configuration.

Note that the acknowledge message does not indicate whether the PIO configuration has been successfully applied (pin assigned), it only indicates the successful configuration of the feature. The configured PIO must be previously unoccupied for successful assignment.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 1 | numFences | U1 | - | - | Number of geofences contained in this message. Note that the receiver can only store a limited number of geofences (currently 4) |
| 2 | 1 | confLvl | U1 | - | - | Required confidence level for state evaluation: 0 = no confidence required, 1 = 68%, 2 = 95%, 3 = 99.7%, 4 = 99.99% |
| 3 | 1 | reserved1 | U1[1] | - | - | Reserved |
| 4 | 1 | pioEnabled | U1 | - | - | 1 = Enable PIO combined fence state output, 0 = disable |
| 5 | 1 | pinPolarity | U1 | - | - | PIO pin polarity. 0 = Low means inside, 1 = Low means outside. Unknown state is always high |
| 6 | 1 | pin | U1 | - | - | PIO pin number |
| 7 | 1 | reserved2 | U1[1] | - | - | Reserved |

**Repeated block (numFences times):**

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 8+12×N | 4 | lat | I4 | 1e-7 | deg | Latitude of the geofence circle center |
| 12+12×N | 4 | lon | I4 | 1e-7 | deg | Longitude of the geofence circle center |
| 16+12×N | 4 | radius | U4 | 1e-2 | m | Radius of the geofence circle |

---

## UBX-CFG-GNSS (0x06 0x3E)
GNSS system configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 4 + 8×numConfigBlocks bytes

Gets or sets the GNSS system channel sharing configuration.

**Configuration requirements:**
- At least one major GNSS must be enabled
- At least 4 tracking channels must be available to each enabled major GNSS (maxTrkCh ≥ 4)
- The number of tracking channels in use must not exceed the number available in hardware
- The sum of all reserved tracking channels must be ≤ the number of tracking channels in use

**Notes:**
- To avoid cross-correlation issues, it is recommended that GPS and QZSS are always both enabled or both disabled
- Polling this message returns the configuration of all supported GNSS, whether enabled or not
- See section GNSS Configuration for a discussion of the use of this message
- See section Satellite Numbering for a description of the GNSS IDs available
- Applying the GNSS system configuration takes some time. After issuing UBX-CFG-GNSS, wait first for the acknowledgement from the receiver and then 0.5 seconds before sending the next command
- If Galileo is enabled, UBX-CFG-GNSS must be followed by UBX-CFG-CFG to save current configuration to BBR and then by UBX-CFG-RST with resetMode set to Hardware reset
- Configuration specific to the GNSS system can be done via other messages (e.g. UBX-CFG-SBAS)

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | msgVer | U1 | Message version (0x00 for this version) |
| 1 | 1 | numTrkChHw | U1 | Number of tracking channels available in hardware (read only) |
| 2 | 1 | numTrkChUse | U1 | (Read only in protocol versions > 23) Number of tracking channels to use. Must be > 0, ≤ numTrkChHw. If 0xFF, then number of tracking channels to use will be set to numTrkChHw |
| 3 | 1 | numConfigBlocks | U1 | Number of configuration blocks following |

**Repeated block (numConfigBlocks times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 4+8×N | 1 | gnssId | U1 | System identifier (see Satellite Numbering) |
| 5+8×N | 1 | resTrkCh | U1 | (Read only in protocol versions > 23) Number of reserved (minimum) tracking channels for this system |
| 6+8×N | 1 | maxTrkCh | U1 | (Read only in protocol versions > 23) Maximum number of tracking channels used for this system. Must be > 0, ≥ resTrkCh, ≤ numTrkChUse and ≤ maximum number of tracking channels supported for this system |
| 7+8×N | 1 | reserved1 | U1 | Reserved |
| 8+8×N | 4 | flags | X4 | Bitfield of flags. At least one signal must be configured in every enabled system (see bitfield below) |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | enable | Enable this system |
| 16-23 | sigCfgMask | Signal configuration mask (system-dependent, see below) |

**Signal configuration masks by GNSS ID:**

**GPS (gnssId = 0):**
- 0x01: GPS L1C/A
- 0x10: GPS L2C
- 0x20: GPS L5

**SBAS (gnssId = 1):**
- 0x01: SBAS L1C/A

**Galileo (gnssId = 2):**
- 0x01: Galileo E1 (not supported in protocol versions < 18)
- 0x10: Galileo E5a
- 0x20: Galileo E5b

**BeiDou (gnssId = 3):**
- 0x01: BeiDou B1I
- 0x10: BeiDou B2I
- 0x80: BeiDou B2A

**QZSS (gnssId = 5):**
- 0x01: QZSS L1C/A
- 0x04: QZSS L1S
- 0x10: QZSS L2C
- 0x20: QZSS L5

**GLONASS (gnssId = 6):**
- 0x01: GLONASS L1
- 0x10: GLONASS L2

---

## UBX-CFG-HNR (0x06 0x5C)
High navigation rate settings

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15.01+ (only with ADR or UDR products)  
**Length:** 4 bytes

**⚠️ Note:** ADR/UDR product specific. Not useful for general GNSS applications.

The u-blox receivers support high rates of navigation update up to 30 Hz. The navigation solution output UBX-NAV-HNR will not be aligned to the top of a second.

- The update rate has a direct influence on the power consumption
- For most applications a 1 Hz update rate would be sufficient

| Offset | Size | Name | Type | Unit | Description |
|--------|------|------|------|------|-------------|
| 0 | 1 | highNavRate | U1 | Hz | Rate of navigation solution output |
| 1 | 3 | reserved1 | U1[3] | - | Reserved |

---

## UBX-CFG-INF (0x06 0x02)

### Poll configuration for one protocol
**Type:** Poll Request  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 1 byte

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | protocolID | U1 | Protocol identifier: 0 = UBX protocol, 1 = NMEA protocol, 2-255 = Reserved |

### Information message configuration
**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 0 + 10×N bytes

The value of infMsgMask[x] below is formed so that each bit represents one of the INF class messages (bit 0 for ERROR, bit 1 for WARNING and so on). Several configurations can be concatenated to one input message. In this case the payload length can be a multiple of the normal length. Output messages from the module contain only one configuration unit.

**Note:** I/O port mapping:
- Port 0 = I2C (DDC)
- Port 1 = Serial port 1
- Port 2 = Serial port 2
- Port 3 = USB
- Port 4 = SPI
- Port 5 = Reserved for future use

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N×10 | 1 | protocolID | U1 | Protocol identifier: 0 = UBX protocol, 1 = NMEA protocol, 2-255 = Reserved |
| 1+10×N | 3 | reserved1 | U1[3] | Reserved |
| 4+10×N | 6 | infMsgMask | X1[6] | A bit mask, saying which information messages are enabled on each I/O port (see bitfield below) |

**Bitfield: infMsgMask (each of 6 bytes, one per I/O port)**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | ERROR | Enable ERROR |
| 1 | WARNING | Enable WARNING |
| 2 | NOTICE | Enable NOTICE |
| 3 | TEST | Enable TEST |
| 4 | DEBUG | Enable DEBUG |

---

## UBX-CFG-ITFM (0x06 0x39)
Jamming/interference monitor configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 8 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | config | X4 | Interference config word (see bitfield below) |
| 4 | 4 | config2 | X4 | Extra settings for jamming/interference monitor (see bitfield below) |

**Bitfield: config**

| Bits | Name | Description |
|------|------|-------------|
| 0-3 | bbThreshold | Broadband jamming detection threshold (unit = dB) |
| 4-7 | cwThreshold | CW jamming detection threshold (unit = dB) |
| 8-30 | algorithmBits | Reserved algorithm settings - should be set to 0x16B156 in hex for correct settings |
| 31 | enable | Enable interference detection |

**Bitfield: config2**

| Bits | Name | Description |
|------|------|-------------|
| 0-12 | generalBits | General settings - should be set to 0x31E in hex for correct setting |
| 13-14 | antSetting | Antenna setting: 0 = unknown, 1 = passive, 2 = active |
| 16 | enable2 | Set to 1 to scan auxiliary bands (u-blox 8 / u-blox M8 only, otherwise ignored) |

---

## UBX-CFG-LOGFILTER (0x06 0x47)
Data logger configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 12 bytes

This message can be used to configure the data logger, i.e. to enable/disable the log recording and to get/set the position entry filter settings.

Position entries can be filtered based on time difference, position difference or current speed thresholds. Position and speed filtering also have a minimum time interval. A position is logged if any of the thresholds are exceeded. If a threshold is set to zero it is ignored. The maximum rate of position logging is 1 Hz.

The filter settings will be configured to the provided values only if the 'applyAllFilterSettings' flag is set. This allows the recording to be enabled/disabled independently of configuring the filter settings.

Configuring the data logger in the absence of a logging file is supported. By doing so, once the logging file is created, the data logger configuration will take effect immediately and logging recording and filtering will activate according to the configuration.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x01 for this version) |
| 1 | 1 | flags | X1 | - | - | Flags (see bitfield below) |
| 2 | 2 | minInterval | U2 | - | s | Minimum time interval between logged positions (0 = not set). This is only applied in combination with the speed and/or position thresholds. If both minInterval and timeThreshold are set, minInterval must be ≤ timeThreshold |
| 4 | 2 | timeThreshold | U2 | - | s | If the time difference is greater than the threshold, then the position is logged (0 = not set) |
| 6 | 2 | speedThreshold | U2 | - | m/s | If the current speed is greater than the threshold, then the position is logged (0 = not set). minInterval also applies |
| 8 | 4 | positionThreshold | U4 | - | m | If the 3D position difference is greater than the threshold, then the position is logged (0 = not set). minInterval also applies |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | recordEnabled | 1 = enable recording, 0 = disable recording |
| 1 | psmOncePerWakeupEnabled | 1 = enable recording only one single position per PSM on/off mode wake-up period, 0 = disable once per wake-up |
| 2 | applyAllFilterSettings | 1 = apply all filter settings, 0 = only apply recordEnabled |

---

## UBX-CFG-MSG (0x06 0x01)

### Poll a message configuration
**Type:** Poll Request  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 2 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | msgClass | U1 | Message class |
| 1 | 1 | msgID | U1 | Message identifier |

### Set message rate(s)
**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 8 bytes

Get/set message rate configuration(s) to/from the receiver. See also section How to change between protocols.

Send rate is relative to the event a message is registered on. For example, if the rate of a navigation message is set to 2, the message is sent every second navigation solution. For configuring NMEA messages, the section NMEA Messages Overview describes class and identifier numbers used.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | msgClass | U1 | Message class |
| 1 | 1 | msgID | U1 | Message identifier |
| 2 | 6 | rate | U1[6] | Send rate on I/O port (6 ports: DDC, UART1, UART2, USB, SPI, reserved) |

### Set message rate (current port)
**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 3 bytes

Set message rate configuration for the current port. See also section How to change between protocols.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | msgClass | U1 | Message class |
| 1 | 1 | msgID | U1 | Message identifier |
| 2 | 1 | rate | U1 | Send rate on current port |

---

## UBX-CFG-NAV5 (0x06 0x24)
Navigation engine settings

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 36 bytes

See the Navigation Configuration Settings Description for a detailed description of how these settings affect receiver operation.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 2 | mask | X2 | - | - | Parameters bitmask. Only the masked parameters will be applied (see bitfield below) |
| 2 | 1 | dynModel | U1 | - | - | Dynamic platform model: 0 = portable, 2 = stationary, 3 = pedestrian, 4 = automotive, 5 = sea, 6 = airborne <1g, 7 = airborne <2g, 8 = airborne <4g, 9 = wrist-worn watch (not supported in protocol versions < 18), 10 = motorbike (supported in protocol versions 19.2, 35.10, 35.15, 35.20), 11 = robotic lawn mower, 12 = electric kick scooter |
| 3 | 1 | fixMode | U1 | - | - | Position fixing mode: 1 = 2D only, 2 = 3D only, 3 = auto 2D/3D |
| 4 | 4 | fixedAlt | I4 | 0.01 | m | Fixed altitude (mean sea level) for 2D fix mode |
| 8 | 4 | fixedAltVar | U4 | 0.0001 | m² | Fixed altitude variance for 2D mode |
| 12 | 1 | minElev | I1 | - | deg | Minimum elevation for a GNSS satellite to be used in NAV |
| 13 | 1 | reserved | U1 | - | - | Reserved |
| 14 | 2 | pDop | U2 | 0.1 | - | Position DOP mask to use |
| 16 | 2 | tDop | U2 | 0.1 | - | Time DOP mask to use |
| 18 | 2 | pAcc | U2 | - | m | Position accuracy mask |
| 20 | 2 | tAcc | U2 | - | m | Time accuracy mask |
| 22 | 1 | staticHoldThresh | U1 | - | cm/s | Static hold threshold |
| 23 | 1 | dgnssTimeout | U1 | - | s | DGNSS timeout |
| 24 | 1 | cnoThreshNumSVs | U1 | - | - | Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted |
| 25 | 1 | cnoThresh | U1 | - | dBHz | C/N0 threshold for deciding whether to attempt a fix |
| 26 | 2 | reserved1 | U1[2] | - | - | Reserved |
| 28 | 2 | staticHoldMaxDist | U2 | - | m | Static hold distance threshold (before quitting static hold) |
| 30 | 1 | utcStandard | U1 | - | - | UTC standard to be used: 0 = Automatic, 3 = USNO (GPS), 5 = Europe (Galileo), 6 = SU (GLONASS), 7 = NTSC China (BeiDou), 8 = NPLI India (NavIC). (Not supported in protocol versions < 16) |
| 31 | 5 | reserved2 | U1[5] | - | - | Reserved |

**Bitfield: mask**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | dyn | Apply dynamic model settings |
| 1 | minEl | Apply minimum elevation settings |
| 2 | posFixMode | Apply fix mode settings |
| 3 | drLim | Reserved |
| 4 | posMask | Apply position mask settings |
| 5 | timeMask | Apply time mask settings |
| 6 | staticHoldMask | Apply static hold settings |
| 7 | dgpsMask | Apply DGPS settings |
| 8 | cnoThreshold | Apply CNO threshold settings (cnoThresh, cnoThreshNumSVs) |
| 9 | utc | Apply UTC settings (not supported in protocol versions < 16) |

---

## UBX-CFG-NAVX5 (0x06 0x23)
Navigation engine expert settings

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  

**Note:** There are multiple versions of this message with different payload lengths (40 or 44 bytes) depending on protocol version. The main differences are in the supported fields and bitfield definitions.

**Length:** 40 bytes (protocol versions 15-17, 18-23.01) or 44 bytes (protocol versions 19.1-19.2)

*[Due to complexity and version-specific variations, consult the full specification for detailed field definitions]*

Key fields include:
- version (U2)
- mask1, mask2 (X2, X4) - parameter bitmasks
- minSVs, maxSVs (U1) - min/max satellites for navigation
- minCNO (U1) - minimum satellite signal level
- iniFix3D (U1) - initial fix must be 3D
- ackAiding (U1) - issue acknowledgements for assistance messages
- wknRollover (U2) - GPS week rollover number
- usePPP (U1) - use Precise Point Positioning
- aopCfg (U1) - AssistNow Autonomous configuration
- aopOrbMaxErr (U2) - Maximum acceptable AssistNow Autonomous orbit error
- useAdr (U1) - Enable/disable ADR/UDR sensor fusion (only on certain products)
- sigAttenCompMode (U1) - Attenuated signal compensation (protocol versions 18+)

**⚠️ Note:** This message contains many advanced expert settings that can adversely affect receiver performance if set incorrectly. Use with caution.

---

## UBX-CFG-NMEA (0x06 0x17)
NMEA protocol configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+

**Note:** Multiple message versions exist (4 bytes, 12 bytes, 20 bytes). Use the latest version (20 bytes, V1) for full functionality.

### Extended NMEA protocol configuration V1
**Length:** 20 bytes

Get/set the NMEA protocol configuration. See section NMEA Protocol Configuration for a detailed description of the configuration effects on NMEA output.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | filter | X1 | Filter flags (see bitfield below) |
| 1 | 1 | nmeaVersion | U1 | NMEA version: 0x21 = 2.1, 0x23 = 2.3, 0x40 = 4.0, 0x41 = 4.10, 0x4B = 4.11 (product-dependent availability) |
| 2 | 1 | numSV | U1 | Maximum number of SVs to report per TalkerId: 0 = unlimited, 8 = 8 SVs, 12 = 12 SVs, 16 = 16 SVs |
| 3 | 1 | flags | X1 | Flags (see bitfield below) |
| 4 | 4 | gnssToFilter | X4 | Filters out satellites based on their GNSS. If a bitfield is enabled, the corresponding satellites will be not output (see bitfield below) |
| 8 | 1 | svNumbering | U1 | Satellite numbering: 0 = Strict (satellites not output if no NMEA-defined value), 1 = Extended (use proprietary numbering) |
| 9 | 1 | mainTalkerId | U1 | Main Talker ID override: 0 = not overridden, 1 = GP, 2 = GL, 3 = GN, 4 = GA, 5 = GB, 6 = GQ (NMEA 4.11+) |
| 10 | 1 | gsvTalkerId | U1 | GSV Talker ID: 0 = Use GNSS-specific Talker ID, 1 = Use the main Talker ID |
| 11 | 1 | version | U1 | Message version (0x01 for this version) |
| 12 | 2 | bdsTalkerId | CH[2] | BeiDou Talker ID (2 characters). If set to zero, then the default BeiDou Talker ID will be used |
| 14 | 6 | reserved1 | U1[6] | Reserved |

**Bitfield: filter**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | posFilt | Enable position output for failed or invalid fixes |
| 1 | mskPosFilt | Enable position output for invalid fixes |
| 2 | timeFilt | Enable time output for invalid times |
| 3 | dateFilt | Enable date output for invalid dates |
| 4 | gpsOnlyFilter | Restrict output to GPS satellites only |
| 5 | trackFilt | Enable COG output even if COG is frozen |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | compat | Enable compatibility mode. This might be needed for certain applications when customer's NMEA parser expects a fixed number of digits in position coordinates |
| 1 | consider | Enable considering mode |
| 2 | limit82 | Enable strict limit to 82 characters maximum |
| 3 | highPrec | Enable high precision mode. This flag cannot be set in conjunction with either compatibility mode or Limit82 mode (not supported in protocol versions < 20.01) |

**Bitfield: gnssToFilter**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | gps | Disable reporting of GPS satellites |
| 1 | sbas | Disable reporting of SBAS satellites |
| 3 | galileo | Disable reporting of Galileo satellites |
| 4 | qzss | Disable reporting of QZSS satellites |
| 5 | glonass | Disable reporting of GLONASS satellites |
| 6 | beidou | Disable reporting of BeiDou satellites |

---

## UBX-CFG-ODO (0x06 0x1E)
Odometer, low-speed COG engine settings

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 20 bytes

**Note:** This feature is not supported for the FTS product variant.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | - | - | Reserved |
| 4 | 1 | flags | U1 | - | - | Odometer/Low-speed COG filter flags (see bitfield below) |
| 5 | 1 | odoCfg | X1 | - | - | Odometer filter settings (see bitfield below) |
| 6 | 6 | reserved2 | U1[6] | - | - | Reserved |
| 12 | 1 | cogMaxSpeed | U1 | 1e-1 | m/s | Speed below which course-over-ground (COG) is computed with the low-speed COG filter |
| 13 | 1 | cogMaxPosAcc | U1 | - | m | Maximum acceptable position accuracy for computing COG with the low-speed COG filter |
| 14 | 2 | reserved3 | U1[2] | - | - | Reserved |
| 16 | 1 | velLpGain | U1 | - | - | Velocity low-pass filter level, range 0..255 |
| 17 | 1 | cogLpGain | U1 | - | - | COG low-pass filter level (at speed < 8 m/s), range 0..255 |
| 18 | 2 | reserved4 | U1[2] | - | - | Reserved |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | useODO | Odometer-enabled flag |
| 1 | useCOG | Low-speed COG filter enabled flag |
| 2 | outLPVel | Output low-pass filtered velocity flag |
| 3 | outLPCog | Output low-pass filtered heading (COG) flag |

**Bitfield: odoCfg**

| Bits | Name | Description |
|------|------|-------------|
| 0-2 | profile | Profile type: 0 = running, 1 = cycling, 2 = swimming, 3 = car, 4 = custom |

---

## UBX-CFG-PM2 (0x06 0x3B)
Extended power management configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 44 or 48 bytes (version-dependent)

**Note:** This feature is not supported for ADR, FTS or HPG products.

*[Multiple versions exist with different field layouts. The general structure includes:]*

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version |
| 1 | 1 | reserved1 | U1 | - | - | Reserved |
| 2 | 1 | maxStartupStateDur | U1 | - | s | Maximum time to spend in Acquisition state (0 = bound disabled) |
| 3 | 1 | reserved2 | U1 | - | - | Reserved |
| 4 | 4 | flags | X4 | - | - | PSM configuration flags (see bitfield below) |
| 8 | 4 | updatePeriod | U4 | - | ms | Position update period (0 = receiver will never retry a fix) |
| 12 | 4 | searchPeriod | U4 | - | ms | Acquisition retry period if previously failed (0 = never retry) |
| 16 | 4 | gridOffset | U4 | - | ms | Grid offset relative to GPS start of week |
| 20 | 2 | onTime | U2 | - | s | Time to stay in Tracking state |
| 22 | 2 | minAcqTime | U2 | - | s | Minimal search time |
| 24 | 20 | reserved3 | U1[20] | - | - | Reserved |
| 44 | 4 | extintInactivityMs | U4 | - | ms | Inactivity time out on EXTINT pin if enabled (protocol versions 18+ in 48-byte version) |

**Bitfield: flags** (varies by version, common fields):

| Bits | Name | Description |
|------|------|-------------|
| 0-2 | optTarget | Optimization target (protocol version 23.01): 000 = performance (default), 001 = power save |
| 4 | extintSel | EXTINT pin select: 0 = EXTINT0, 1 = EXTINT1 |
| 5 | extintWake | EXTINT pin control: enable keep receiver awake |
| 6 | extintBackup | EXTINT pin control: enable force receiver into BACKUP mode |
| 7 | extintInactive | EXTINT pin control: enable force backup on inactivity timeout |
| 10-11 | limitPeakCurr | Limit peak current: 00 = disabled, 01 = enabled |
| 16 | waitTimeFix | Wait for Timefix: 0 = wait for normal fix, 1 = wait for time fix |
| 17 | updateRTC | Update Real Time Clock |
| 18 | updateEPH | Update Ephemeris |
| 21 | doNotEnterOff | Behavior of receiver in case of no fix: 0 = enter Inactive state, 1 = keep trying |
| 22-23 | mode | Mode of operation: 00 = ON/OFF operation (PSMOO), 01 = cyclic tracking operation (PSMCT) |

**⚠️ Note:** This message has significant version-specific variations. Consult the full specification for your protocol version.

---

## UBX-CFG-PMS (0x06 0x86)
Power mode setup

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 22, 22.01, 23, 23.01  
**Length:** 8 bytes

Using UBX-CFG-PMS to set Super-E mode to 1, 2 or 4 Hz navigation rates sets minAcqTime to 180 s instead of the default 300 s in protocol version 23.01.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 1 | powerSetupValue | U1 | - | - | Power setup value: 0x00 = Full power, 0x01 = Balanced, 0x02 = Interval, 0x03 = Aggressive with 1 Hz, 0x04 = Aggressive with 2 Hz, 0x05 = Aggressive with 4 Hz, 0xFF = Invalid (only when polling) |
| 2 | 2 | period | U2 | - | s | Position update period and search period. Recommended minimum period is 10 s, although the receiver accepts any value bigger than 5 s. Only valid when powerSetupValue set to Interval, otherwise must be set to '0' |
| 4 | 2 | onTime | U2 | - | s | Duration of the ON phase, must be smaller than the period. Only valid when powerSetupValue set to Interval, otherwise must be set to '0' |
| 6 | 2 | reserved1 | U1[2] | - | - | Reserved |

---

## UBX-CFG-PRT (0x06 0x00)

### Poll configuration for one I/O port
**Type:** Poll Request  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 1 byte

Sending this message with a port ID as payload results in having the receiver return the configuration for the specified port.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | PortID | U1 | Port identifier number (see the other versions of CFG-PRT for valid values) |

### Port configuration for UART ports
**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 20 bytes

Several configurations can be concatenated to one input message. Note that this message can affect baud rate and other transmission parameters. Because there may be messages queued for transmission there may be uncertainty about which protocol applies to such messages.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | portID | U1 | - | - | Port identifier number (see the integration manual for valid UART port IDs) |
| 1 | 1 | reserved1 | U1 | - | - | Reserved |
| 2 | 2 | txReady | X2 | - | - | TX ready PIN configuration (see bitfield below) |
| 4 | 4 | mode | X4 | - | - | UART mode (see bitfield below) |
| 8 | 4 | baudRate | U4 | - | Bits/s | Baud rate in bits/second |
| 12 | 2 | inProtoMask | X2 | - | - | Input protocols active (see bitfield below) |
| 14 | 2 | outProtoMask | X2 | - | - | Output protocols active (see bitfield below) |
| 16 | 2 | flags | X2 | - | - | Flags bit mask (see bitfield below) |
| 18 | 2 | reserved2 | U1[2] | - | - | Reserved |

**Bitfield: txReady**

| Bits | Name | Description |
|------|------|-------------|
| 0 | en | Enable TX ready feature for this port |
| 1 | pol | Polarity: 0 = High-active, 1 = Low-active |
| 2-6 | pin | PIO to be used (must not be in use by another function) |
| 7-15 | thres | Threshold: The given threshold is multiplied by 8 bytes. Range 0x000-0x1FF (0-4088 bytes) |

**Bitfield: mode**

| Bits | Name | Description |
|------|------|-------------|
| 6-7 | charLen | Character length: 00 = 5bit (not supported), 01 = 6bit (not supported), 10 = 7bit (supported only with parity), 11 = 8bit |
| 9-11 | parity | Parity: 000 = Even, 001 = Odd, 10X = No parity, X1X = Reserved |
| 12-13 | nStopBits | Number of Stop bits: 00 = 1 Stop bit, 01 = 1.5 Stop bit, 10 = 2 Stop bit, 11 = 0.5 Stop bit |

**Bitfield: inProtoMask**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | inUbx | UBX protocol |
| 1 | inNmea | NMEA protocol |
| 2 | inRtcm | RTCM2 protocol |
| 5 | inRtcm3 | RTCM3 protocol (not supported in protocol versions < 20) |

**Bitfield: outProtoMask**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | outUbx | UBX protocol |
| 1 | outNmea | NMEA protocol |
| 5 | outRtcm3 | RTCM3 protocol (not supported in protocol versions < 20) |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 1 | extendedTxTimeout | Extended TX timeout: if set, the port will time out if allocated TX memory ≥4 kB and no activity for 1.5 s |

### Port configuration for USB port
**Type:** Get/Set  
**Length:** 20 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | portID | U1 | Port identifier number (= 3 for USB port) |
| 1 | 1 | reserved1 | U1 | Reserved |
| 2 | 2 | txReady | X2 | TX ready PIN configuration (see bitfield above) |
| 4 | 8 | reserved2 | U1[8] | Reserved |
| 12 | 2 | inProtoMask | X2 | Input protocols active (see bitfield above) |
| 14 | 2 | outProtoMask | X2 | Output protocols active (see bitfield above) |
| 16 | 2 | reserved3 | U1[2] | Reserved |
| 18 | 2 | reserved4 | U1[2] | Reserved |

### Port configuration for SPI port
**Type:** Get/Set  
**Length:** 20 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | portID | U1 | Port identifier number (= 4 for SPI port) |
| 1 | 1 | reserved1 | U1 | Reserved |
| 2 | 2 | txReady | X2 | TX ready PIN configuration (see bitfield above) |
| 4 | 4 | mode | X4 | SPI Mode Flags (see bitfield below) |
| 8 | 4 | reserved2 | U1[4] | Reserved |
| 12 | 2 | inProtoMask | X2 | Input protocols active (see bitfield above) |
| 14 | 2 | outProtoMask | X2 | Output protocols active (see bitfield above) |
| 16 | 2 | flags | X2 | Flags bit mask (see bitfield above) |
| 18 | 2 | reserved3 | U1[2] | Reserved |

**Bitfield: mode (SPI)**

| Bits | Name | Description |
|------|------|-------------|
| 1-2 | spiMode | SPI Mode: 00 = Mode 0 (CPOL=0, CPHA=0), 01 = Mode 1 (CPOL=0, CPHA=1), 10 = Mode 2 (CPOL=1, CPHA=0), 11 = Mode 3 (CPOL=1, CPHA=1) |
| 8-13 | ffCnt | Number of bytes containing 0xFF to receive before switching off reception. Range: 0 (mechanism off) - 63 |

### Port configuration for I2C (DDC) port
**Type:** Get/Set  
**Length:** 20 bytes

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | portID | U1 | Port identifier number (= 0 for I2C (DDC) port) |
| 1 | 1 | reserved1 | U1 | Reserved |
| 2 | 2 | txReady | X2 | TX ready PIN configuration (see bitfield above) |
| 4 | 4 | mode | X4 | I2C (DDC) Mode Flags (see bitfield below) |
| 8 | 4 | reserved2 | U1[4] | Reserved |
| 12 | 2 | inProtoMask | X2 | Input protocols active (see bitfield above) |
| 14 | 2 | outProtoMask | X2 | Output protocols active (see bitfield above) |
| 16 | 2 | flags | X2 | Flags bit mask (see bitfield above) |
| 18 | 2 | reserved3 | U1[2] | Reserved |

**Bitfield: mode (I2C)**

| Bits | Name | Description |
|------|------|-------------|
| 1-7 | slaveAddr | Slave address. Range: 0x07 < slaveAddr < 0x78. Bit 0 must be 0 |

---

## UBX-CFG-PWR (0x06 0x57)
Put receiver in a defined power state

**Type:** Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 8 bytes

**⚠️ DEPRECATED:** This message is deprecated in protocol versions > 17. Use UBX-CFG-RST for GNSS start/stop and UBX-RXM-PMREQ for software backup.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x01 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | state | U4 | Enter system state: 0x52554E20 = GNSS running, 0x53544F50 = GNSS stopped, 0x42434B50 = Software backup (USB interface will be disabled, other wakeup source is needed) |

---

## UBX-CFG-RATE (0x06 0x08)
Navigation/measurement rate settings

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 6 bytes

This message allows the user to alter the rate at which navigation solutions (and the measurements that they depend on) are generated by the receiver. The calculation of the navigation solution will always be aligned to the top of a second zero (first second of the week) of the configured reference time system.

**Notes:**
- Navigation period is an integer multiple of the measurement period (protocol versions > 17)
- Each measurement triggers the measurements generation and, if available, raw data output
- The navRate value defines that every nth measurement triggers a navigation epoch
- The update rate has a direct influence on power consumption
- For most applications a 1 Hz update rate would be sufficient
- When using power save mode, measurement and navigation rate can differ from the values configured here

| Offset | Size | Name | Type | Unit | Description |
|--------|------|------|------|------|-------------|
| 0 | 2 | measRate | U2 | ms | The elapsed time between GNSS measurements, which defines the rate, e.g. 100 ms ⇒ 10 Hz, 1000 ms ⇒ 1 Hz, 10000 ms ⇒ 0.1 Hz. Measurement rate should be ≥ 25 ms (≥ 50 ms in protocol versions < 24) |
| 2 | 2 | navRate | U2 | cycles | The ratio between the number of measurements and the number of navigation solutions, e.g. 5 means five measurements for every navigation solution. Maximum value is 127. (This parameter is ignored and the navRate is fixed to 1 in protocol versions < 18) |
| 4 | 2 | timeRef | U2 | - | The time system to which measurements are aligned: 0 = UTC time, 1 = GPS time, 2 = GLONASS time (not supported in protocol versions < 18), 3 = BeiDou time (not supported in protocol versions < 18), 4 = Galileo time (not supported in protocol versions < 18), 5 = NavIC time (not supported in protocol versions < 29) |

---

## UBX-CFG-RINV (0x06 0x34)
Contents of remote inventory

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 1 + 1×N bytes

If N is greater than 30, the excess bytes are discarded.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | flags | X1 | Flags (see bitfield below) |
| 1+1×N | 1 | data | U1 | Data to store/stored in remote inventory (repeated N times) |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | dump | Dump data at startup. Does not work if flag binary is set |
| 1 | binary | Data is binary |

---

## UBX-CFG-RST (0x06 0x04)
Reset receiver / Clear backup data structures

**Type:** Command  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 4 bytes

Do not expect this message to be acknowledged by the receiver.
- Newer FW version will not acknowledge this message at all
- Older FW version will acknowledge this message but the acknowledge may not be sent completely before the receiver is reset

**Notes:**
- If Galileo is enabled, UBX-CFG-RST Controlled GNSS start must be followed by UBX-CFG-CFG to save current configuration to BBR and then by UBX-CFG-RST with resetMode set to Hardware reset
- If Galileo is enabled, use resetMode Hardware reset instead of Controlled software reset or Controlled software reset (GNSS only)

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 2 | navBbrMask | X2 | BBR sections to clear. The following special sets apply: 0x0000 = Hot start, 0x0001 = Warm start, 0xFFFF = Cold start (see bitfield below) |
| 2 | 1 | resetMode | U1 | Reset Type: 0x00 = Hardware reset (watchdog) immediately, 0x01 = Controlled software reset, 0x02 = Controlled software reset (GNSS only), 0x04 = Hardware reset (watchdog) after shutdown, 0x08 = Controlled GNSS stop, 0x09 = Controlled GNSS start |
| 3 | 1 | reserved1 | U1 | Reserved |

**Bitfield: navBbrMask**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | eph | Ephemeris |
| 1 | alm | Almanac |
| 2 | health | Health |
| 3 | klob | Klobuchar parameters |
| 4 | pos | Position |
| 5 | clkd | Clock drift |
| 6 | osc | Oscillator parameter |
| 7 | utc | UTC correction + GPS leap seconds parameters |
| 8 | rtc | RTC |
| 9 | sfdr | SFDR Parameters (only available on the ADR/UDR/HPS product variant) and weak signal compensation estimates |
| 10 | vmon | SFDR Vehicle Monitoring Parameter (only available on the ADR/UDR/HPS product variant) |
| 11 | tct | TCT Parameters (only available on the ADR/UDR/HPS product variant) |
| 15 | aop | Autonomous orbit parameters |

---

## UBX-CFG-RXM (0x06 0x11)
RXM configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 2 bytes

For a detailed description see section Power Management.

**Note:** Power save mode cannot be selected when the receiver is configured to process GLONASS signals (using UBX-CFG-GNSS) in protocol versions 15-17.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | reserved1 | U1 | Reserved |
| 1 | 1 | lpMode | U1 | Low power mode: 0 = Continuous mode, 1 = Power save mode, 4 = Continuous mode (Note: for protocol versions ≥ 14, both 0 and 4 configure Continuous mode) |

---

## UBX-CFG-SBAS (0x06 0x16)
SBAS configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 8 bytes

This message configures the SBAS receiver subsystem (i.e. WAAS, EGNOS, MSAS). See SBAS configuration settings description for a detailed description of how these settings affect receiver operation.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | mode | X1 | SBAS mode (see bitfield below) |
| 1 | 1 | usage | X1 | SBAS usage (see bitfield below) |
| 2 | 1 | maxSBAS | U1 | Maximum number of SBAS prioritized tracking channels (valid range: 0 - 3) to use (obsolete and superseded by UBX-CFG-GNSS in protocol versions 14+) |
| 3 | 1 | scanmode2 | X1 | Continuation of scanmode bitmask below (see bitfield below) |
| 4 | 4 | scanmode1 | X4 | Which SBAS PRN numbers to search for (bitmask). If all bits are set to zero, auto-scan (i.e. all valid PRNs) are searched. Every bit corresponds to a PRN number (see bitfield below) |

**Bitfield: mode**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | enabled | SBAS enabled (1) / disabled (0) - **This field is deprecated; use UBX-CFG-GNSS to enable/disable SBAS operation** |
| 1 | test | SBAS testbed: Use data anyhow (1) / Ignore data when in test mode (SBAS msg 0) |

**Bitfield: usage**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | range | Use SBAS GEOs as a ranging source (for navigation) |
| 1 | diffCorr | Use SBAS differential corrections |
| 2 | integrity | Use SBAS integrity information. If enabled, the receiver will only use GPS satellites for which integrity information is available |

**Bitfield: scanmode1 / scanmode2**
Bitmask where each bit corresponds to a SBAS PRN number. Refer to the full specification for the complete PRN mapping.

---

## UBX-CFG-SENIF (0x06 0x88)
I2C sensor interface configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 19+ (only with ADR or UDR products)  
**Length:** 6 bytes

**⚠️ Note:** ADR/UDR product specific. Not useful for general GNSS applications.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | type | U1 | Type of interface, 0 for I2C |
| 1 | 1 | version | U1 | Message version, 0 for this message |
| 2 | 2 | flags | X2 | Feature configuration flags (see bitfield below) |
| 4 | 2 | pioConf | X2 | PIO configuration flags (see bitfield below) |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | senConn | Sensor is connected to I2C interface |

**Bitfield: pioConf**

| Bits | Name | Description |
|------|------|-------------|
| 0-4 | i2cSdaPio | PIO of the I2C SDA line (Supported options - see spec) |
| 5-9 | i2cSclPio | PIO of the I2C SCL line (Supported options - see spec) |

---

## UBX-CFG-SLAS (0x06 0x8D)
SLAS configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 with protocol version 19.2 (only with ADR or UDR products)  
**Length:** 4 bytes

**⚠️ Note:** ADR/UDR product specific. Not useful for general GNSS applications.

This message configures the QZSS SLAS (Sub-meter Level Augmentation System). See the SLAS Configuration Settings Description for a detailed description of how these settings affect receiver operation.

To apply SLAS corrections, QZSS operation and L1S signal tracking must be enabled see UBX-CFG-GNSS.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | mode | X1 | SLAS Mode (see bitfield below) |
| 1 | 3 | reserved1 | U1[3] | Reserved |

**Bitfield: mode**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | enabled | Apply QZSS SLAS DGNSS corrections: Enabled (1) / Disabled (0) |
| 1 | test | Use QZSS SLAS data when in test mode (SLAS msg 0): Use data anyhow (1) / Ignore data when in Test Mode (0) |
| 2 | raim | Raim out measurements that are not corrected by QZSS SLAS, if at least 5 measurements are corrected: Enabled (1) / Disabled (0) |

---

## UBX-CFG-SMGR (0x06 0x62)
Synchronization manager configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 16+ (only with Time & Frequency Sync products)  
**Length:** 20 bytes

**⚠️ Note:** Time & Frequency Sync product specific. Not useful for general GNSS applications.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 1 | minGNSSFix | U1 | - | - | Minimum number of GNSS fixes before we commit to use it as a source |
| 2 | 2 | maxFreqChangeRate | U2 | - | ppb/s | Maximum frequency change rate during disciplining. Must not exceed 30ppb/s |
| 4 | 2 | maxPhaseCorrRate | U2 | - | ns/s | Maximum phase correction rate in coherent time pulse mode. Must not exceed 100ns/s |
| 6 | 2 | reserved1 | U1[2] | - | - | Reserved |
| 8 | 2 | freqTolerance | U2 | - | ppb | Limit of possible deviation from nominal before UBX-TIM-TOS indicates that frequency is out of tolerance |
| 10 | 2 | timeTolerance | U2 | - | ns | Limit of possible deviation from nominal before UBX-TIM-TOS indicates that time pulse is out of tolerance |
| 12 | 2 | messageCfg | X2 | - | - | Sync manager message configuration (see bitfield below) |
| 14 | 2 | maxSlewRate | U2 | - | us/s | Maximum slew rate, the maximum time correction that shall be applied between locked pulses in corrective time pulse mode. To have no limit on the slew rate, set the flag disableMaxSlewRate to 1 |
| 16 | 4 | flags | X4 | - | - | Flags (see bitfield below) |

**Bitfield: messageCfg**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | measInternal | 1 = report the estimated offset of the internal oscillator based on the oscillator model |
| 1 | measGNSS | 1 = report the internal oscillator's offset relative to GNSS |
| 2 | measEXTINT0 | 1 = report the internal oscillator's offset relative to the source on EXTINT0 |
| 3 | measEXTINT1 | 1 = report the internal oscillator's offset relative to the source on EXTINT1 |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | disableInternal | 1 = disable disciplining of the internal oscillator |
| 1 | disableExternal | 1 = disable disciplining of the external oscillator |
| 2 | preferenceMode | Reference selection preference: 0 = best frequency accuracy, 1 = best phase accuracy |
| 3 | enableGNSS | 1 = enable use of GNSS as synchronization source |
| 4 | enableEXTINT0 | 1 = enable use of EXTINT0 as synchronization source |
| 5 | enableEXTINT1 | 1 = enable use of EXTINT1 as synchronization source |
| 6 | enableHostMeasInt | 1 = enable use of host measurements on the internal oscillator as synchronization source |
| 7 | enableHostMeasExt | 1 = enable use of host measurements on the external oscillator as synchronization source |
| 8 | useAnyFix | 0 = use over-determined navigation solutions only, 1 = use any fix |
| 9 | disableMaxSlewRate | 0 = use the value in the field maxSlewRate, 1 = don't use the value in the field maxSlewRate |
| 10 | issueFreqWarning | 1 = issue a warning (via UBX-TIM-TOS flag) when frequency uncertainty exceeds freqTolerance |
| 11 | issueTimeWarning | 1 = issue a warning (via UBX-TIM-TOS flag) when time uncertainty exceeds timeTolerance |
| 12-13 | TPCoherent | Control time pulse coherency: 0 = Coherent pulses, 1 = Non-coherent pulses, 2 = Post-initialization coherent pulses |
| 14 | disableOffset | 1 = disable automatic storage of oscillator offset |

---

## UBX-CFG-SPT (0x06 0x64)
Configure and start a sensor production test

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15.01+ (only with ADR or UDR products)  
**Length:** 12 bytes

**⚠️ Note:** ADR/UDR product specific. Not useful for general GNSS applications.

The production test uses the built-in self-test capabilities of an attached sensor. This message is only supported if a sensor is directly connected to the u-blox receiver.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | reserved1 | U1 | Reserved |
| 2 | 2 | sensorId | U2 | ID of the sensor to be tested; see UBX-MON-SPT for defined IDs |
| 4 | 8 | reserved2 | U1[8] | Reserved |

---

## UBX-CFG-TMODE2 (0x06 0x3D)
Time mode settings 2

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+ (only with Time & Frequency Sync or Time Sync products)  
**Length:** 28 bytes

**⚠️ Note:** This message is available only for timing receivers. This message replaces the deprecated UBX-CFG-TMODE message.

See the Time Mode Description for details.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | timeMode | U1 | - | - | Time Transfer Mode: 0 = Disabled, 1 = Survey In, 2 = Fixed Mode (true position information required), 3-255 = Reserved |
| 1 | 1 | reserved1 | U1 | - | - | Reserved |
| 2 | 2 | flags | X2 | - | - | Time mode flags (see bitfield below) |
| 4 | 4 | ecefXOrLat | I4 | cm or deg×1e-7 | - | WGS84 ECEF X coordinate or latitude, depending on flags above |
| 8 | 4 | ecefYOrLon | I4 | cm or deg×1e-7 | - | WGS84 ECEF Y coordinate or longitude, depending on flags above |
| 12 | 4 | ecefZOrAlt | I4 | cm | - | WGS84 ECEF Z coordinate or altitude, depending on flags above |
| 16 | 4 | fixedPosAcc | U4 | - | mm | Fixed position 3D accuracy |
| 20 | 4 | svinMinDur | U4 | - | s | Survey-in minimum duration |
| 24 | 4 | svinAccLimit | U4 | - | mm | Survey-in position accuracy limit |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | lla | Position is given in LAT/LON/ALT (default is ECEF) |
| 1 | altInv | Altitude is not valid, in case lla was set |

---

## UBX-CFG-TMODE3 (0x06 0x71)
Time mode settings 3

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 20+ (only with High Precision GNSS products)  
**Length:** 40 bytes

Configures the receiver to be in Time Mode. The position referred to in this message is that of the Antenna Reference Point (ARP). See the Time Mode Description for details.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | version | U1 | - | - | Message version (0x00 for this version) |
| 1 | 1 | reserved1 | U1 | - | - | Reserved |
| 2 | 2 | flags | X2 | - | - | Receiver mode flags (see bitfield below) |
| 4 | 4 | ecefXOrLat | I4 | cm or deg×1e-7 | - | WGS84 ECEF X coordinate (or latitude) of the ARP position, depending on flags |
| 8 | 4 | ecefYOrLon | I4 | cm or deg×1e-7 | - | WGS84 ECEF Y coordinate (or longitude) of the ARP position, depending on flags |
| 12 | 4 | ecefZOrAlt | I4 | cm | - | WGS84 ECEF Z coordinate (or altitude) of the ARP position, depending on flags |
| 16 | 1 | ecefXOrLatHP | I1 | 0.1 mm or deg×1e-9 | - | High-precision WGS84 ECEF X coordinate (or latitude) of the ARP position. Must be in the range -99..+99. The precise coordinate in units of cm (or 1e-7 degrees) is given by: ecefXOrLat + (ecefXOrLatHP × 1e-2) |
| 17 | 1 | ecefYOrLonHP | I1 | 0.1 mm or deg×1e-9 | - | High-precision WGS84 ECEF Y coordinate (or longitude) of the ARP position. Must be in the range -99..+99 |
| 18 | 1 | ecefZOrAltHP | I1 | 0.1 mm | - | High-precision WGS84 ECEF Z coordinate (or altitude) of the ARP position. Must be in the range -99..+99 |
| 19 | 1 | reserved2 | U1 | - | - | Reserved |
| 20 | 4 | fixedPosAcc | U4 | 0.1 mm | - | Fixed position 3D accuracy |
| 24 | 4 | svinMinDur | U4 | - | s | Survey-in minimum duration |
| 28 | 4 | svinAccLimit | U4 | 0.1 mm | - | Survey-in position accuracy limit |
| 32 | 8 | reserved3 | U1[8] | - | - | Reserved |

**Bitfield: flags**

| Bits | Name | Description |
|------|------|-------------|
| 0-7 | mode | Receiver Mode: 0 = Disabled, 1 = Survey In, 2 = Fixed Mode (true ARP position information required), 3-255 = Reserved |
| 8 | lla | Position is given in LAT/LON/ALT (default is ECEF) |

---

## UBX-CFG-TP5 (0x06 0x31)

### Poll time pulse parameters for time pulse 0
**Type:** Poll Request  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15-22  
**Length:** 0 bytes (no payload)

Sending this (empty / no-payload) message to the receiver results in the receiver returning a message of type UBX-CFG-TP5 with a payload as defined below for timepulse 0.

### Poll time pulse parameters
**Type:** Poll Request  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15-22  
**Length:** 1 byte

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | tpIdx | U1 | Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2) |

### Time pulse parameters
**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 32 bytes

This message is used to get/set time pulse parameters. For more information see section Time pulse.

**Note:** Two message versions exist (version 0x00 for protocol version 15, version 0x01 for protocol versions 16+). The main difference is terminology (GPS vs GNSS) and additional flags.

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 1 | tpIdx | U1 | - | - | Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2) |
| 1 | 1 | version | U1 | - | - | Message version (0x00 or 0x01 depending on protocol version) |
| 2 | 2 | reserved1 | U1[2] | - | - | Reserved |
| 4 | 2 | antCableDelay | I2 | - | ns | Antenna cable delay |
| 6 | 2 | rfGroupDelay | I2 | - | ns | RF group delay |
| 8 | 4 | freqPeriod | U4 | - | Hz or us | Frequency or period time, depending on setting of bit 'isFreq' |
| 12 | 4 | freqPeriodLock | U4 | - | Hz or us | Frequency or period time when locked to GNSS time, only used if 'lockedOtherSet' is set |
| 16 | 4 | pulseLenRatio | U4 | - | us or 2^-32 | Pulse length or duty cycle, depending on 'isLength' |
| 20 | 4 | pulseLenRatioLock | U4 | - | us or 2^-32 | Pulse length or duty cycle when locked to GNSS time, only used if 'lockedOtherSet' is set |
| 24 | 4 | userConfigDelay | I4 | - | ns | User-configurable time pulse delay |
| 28 | 4 | flags | X4 | - | - | Configuration flags (see bitfield below) |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | active | If set enable time pulse; if pin assigned to another function, other function takes precedence. Must be set for FTS variant |
| 1 | lockGnssFreq | If set, synchronize time pulse to GNSS as soon as GNSS time is valid. This flag is ignored by the FTS product variant. This flag can be unset only in Timing product variants |
| 2 | lockedOtherSet | If set the receiver switches between the timepulse settings given by 'freqPeriodLocked' & 'pulseLenLocked' and those given by 'freqPeriod' & 'pulseLen' |
| 3 | isFreq | If set 'freqPeriodLock' and 'freqPeriod' are interpreted as frequency, otherwise interpreted as period |
| 4 | isLength | If set 'pulseLenRatioLock' and 'pulseLenRatio' interpreted as pulse length, otherwise interpreted as duty cycle |
| 5 | alignToTow | Align pulse to top of second (period time must be integer fraction of 1s). Also set 'lockGnssFreq' to use this feature |
| 6 | polarity | Pulse polarity: 0 = falling edge at top of second, 1 = rising edge at top of second |
| 7-9 | gridUtcGnss | Timegrid to use: 0 = UTC, 1 = GPS, 2 = GLONASS, 3 = BeiDou, 4 = Galileo (not supported in protocol versions < 18) |
| 10-11 | syncMode | Sync Manager lock mode (FTS variant only): 0 = switch to locked as soon as accurate time available and never switch back, 1 = switch back to unlocked when time gets inaccurate |

---

## UBX-CFG-TXSLOT (0x06 0x53)
TX buffer time slots configuration

**Type:** Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 16+ (only with Time & Frequency Sync products)  
**Length:** 16 bytes

**⚠️ Note:** Time & Frequency Sync product specific. Not useful for general GNSS applications.

This message configures how transmit time slots are defined for the receiver interfaces. These time slots are relative to the chosen time pulse. A receiver that supports this message offers 3 time slots: nr. 0, 1 and 2. These time pulses follow each other and their associated priorities decrease in this order.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | enable | X1 | Bitfield of ports for which the slots are enabled (see bitfield below) |
| 2 | 1 | refTp | U1 | Reference timepulse source: 0 = Timepulse, 1 = Timepulse 2 |
| 3 | 1 | reserved1 | U1 | Reserved |
| 4+4×N | 4 | end | U4 | End of timeslot in milliseconds after time pulse (repeated 3 times for slots 0, 1, 2) |

**Bitfield: enable**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | DDC | DDC/I2C |
| 1 | UART1 | UART 1 |
| 2 | UART2 | UART 2 |
| 3 | USB | USB |
| 4 | SPI | SPI |

---

## UBX-CFG-USB (0x06 0x1B)
USB configuration

**Type:** Get/Set  
**Supported:** u-blox 8 / u-blox M8 protocol versions 15+  
**Length:** 108 bytes

| Offset | Size | Name | Type | Scale | Unit | Description |
|--------|------|------|------|-------|------|-------------|
| 0 | 2 | vendorID | U2 | - | - | Vendor ID. This field shall only be set to registered Vendor IDs. Changing this field requires special Host drivers |
| 2 | 2 | productID | U2 | - | - | Product ID. Changing this field requires special Host drivers |
| 4 | 2 | reserved1 | U1[2] | - | - | Reserved |
| 6 | 2 | reserved2 | U1[2] | - | - | Reserved |
| 8 | 2 | powerConsumption | U2 | - | mA | Power consumed by the device |
| 10 | 2 | flags | X2 | - | - | Various configuration flags (see bitfield below) |
| 12 | 32 | vendorString | CH[32] | - | - | String containing the vendor name. 32 ASCII bytes including 0-termination |
| 44 | 32 | productString | CH[32] | - | - | String containing the product name. 32 ASCII bytes including 0-termination |
| 76 | 32 | serialNumber | CH[32] | - | - | String containing the serial number. 32 ASCII bytes including 0-termination. Changing the String fields requires special Host drivers |

**Bitfield: flags**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | reEnum | Force re-enumeration |
| 1 | powerMode | Self-powered (1), bus-powered (0) |

---

## Summary

This document describes all 40 UBX-CFG protocol messages for the u-blox M8 GNSS receiver. 

**Key categories:**

- **Core configuration:** CFG-CFG, CFG-RST, CFG-RATE, CFG-PRT
- **Navigation:** CFG-NAV5, CFG-NAVX5, CFG-GNSS
- **Protocols:** CFG-NMEA, CFG-MSG, CFG-INF
- **Power management:** CFG-PM2, CFG-PMS, CFG-RXM
- **Advanced features:** CFG-GEOFENCE, CFG-ODO, CFG-LOGFILTER
- **Product-specific:**
  - **ADR/UDR:** CFG-ESFALG, CFG-ESFA, CFG-ESFG, CFG-ESFWT, CFG-HNR, CFG-SENIF, CFG-SLAS, CFG-SPT
  - **Time/Frequency Sync:** CFG-DOSC, CFG-ESRC, CFG-SMGR, CFG-TMODE2, CFG-TMODE3, CFG-TP5, CFG-TXSLOT
  - **High Precision:** CFG-DGNSS, CFG-TMODE3

**Notes:**
- Many messages have version-specific variations
- Some messages are deprecated (CFG-PWR)
- Product-specific messages (ADR, UDR, FTS, HPG, Time/Frequency Sync) are marked accordingly
- Always consult the receiver acknowledgement (UBX-ACK-ACK/NAK) to confirm configuration success

**For library implementation priorities:**
1. **Essential:** CFG-CFG, CFG-RST, CFG-RATE, CFG-PRT, CFG-MSG, CFG-NAV5, CFG-GNSS
2. **Common:** CFG-NMEA, CFG-RXM, CFG-SBAS, CFG-INF
3. **Optional:** CFG-ANT, CFG-ODO, CFG-GEOFENCE, CFG-ITFM, CFG-LOGFILTER, CFG-PM2, CFG-PMS, CFG-USB
4. **Product-specific:** All ESF*, HNR, DOSC, ESRC, SMGR, TMODE*, TP5, TXSLOT, SENIF, SLAS, SPT, DGNSS messages
