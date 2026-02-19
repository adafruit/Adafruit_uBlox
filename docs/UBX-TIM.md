# UBX-TIM (0x0D) - Timing Messages

Time Pulse Output, Time Mark Results

Messages in the TIM class are used to output timing information from the receiver, like Time Pulse and Time Mark measurements.

---

## UBX-TIM-DOSC (0x0D 0x11)
Disciplined oscillator control

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 8 bytes

The receiver sends this message when it is disciplining an external oscillator and the external oscillator is set up to be controlled via the host.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | value | U4 | The raw value to be applied to the DAC controlling the external oscillator. The least significant bits should be written to the DAC, with the higher bits being ignored. |

---

## UBX-TIM-FCHG (0x0D 0x16)
Oscillator frequency changed notification

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 32 bytes

This message reports frequency changes commanded by the sync manager for the internal and external oscillator. It is output at the configured rate even if the sync manager decides not to command a frequency change.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 4 | iTOW | U4 | GPS time of week of the navigation epoch from which the sync manager obtains the GNSS specific data (ms). Like for the NAV message, the iTOW can be used to group messages of a single sync manager run together (See the description of iTOW for details) |
| 8 | 4 | intDeltaFreq | I4 | Frequency increment of the internal oscillator (ppb * 2^-8) |
| 12 | 4 | intDeltaFreqUnc | U4 | Uncertainty of the internal oscillator frequency increment (ppb * 2^-8) |
| 16 | 4 | intRaw | U4 | Current raw DAC setting commanded to the internal oscillator |
| 20 | 4 | extDeltaFreq | I4 | Frequency increment of the external oscillator (ppb * 2^-8) |
| 24 | 4 | extDeltaFreqUnc | U4 | Uncertainty of the external oscillator frequency increment (ppb * 2^-8) |
| 28 | 4 | extRaw | U4 | Current raw DAC setting commanded to the external oscillator |

---

## UBX-TIM-HOC (0x0D 0x17)
Host oscillator control

**Type:** Input  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 8 bytes

This message can be sent by the host to force the receiver to bypass the disciplining algorithms in the SMGR and carry out the instructed changes to internal or external oscillator frequency. No checks are carried out on the size of the frequency change requested, so normal limits imposed by the SMGR are ignored.

**Note:** It is recommended that the disciplining of that oscillator is disabled before this message is sent (i.e. by clearing the enableInternal or enableExternal flag in the UBX-CFG-SMGR message), otherwise the autonomous disciplining processes may cancel the effect of the direct command. The GNSS subsystem may temporarily lose track of some/all satellite signals if a large change of the internal oscillator is made.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | oscId | U1 | Id of oscillator: 0=internal oscillator, 1=external oscillator |
| 2 | 1 | flags | U1 | Flags (see bitfield below) |
| 3 | 1 | reserved1 | U1 | Reserved |
| 4 | 4 | value | I4 | Required frequency offset (ppb * 2^-8) or raw output, depending on the flags |

### Bitfield: flags
| Name | Description |
|------|-------------|
| raw | Type of value: 0=frequency offset, 1=raw digital output |
| difference | Nature of value: 0=absolute (i.e. relative to 0), 1=relative to current setting |

---

## UBX-TIM-SMEAS (0x0D 0x13)
Source measurement

**Type:** Input/Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 12 + 24*numMeas bytes

Frequency and/or phase measurement of synchronization sources. The measurements are relative to the nominal frequency and nominal phase. The receiver reports the measurements on its sync sources using this message. Which measurements are reported can be configured using UBX-CFG-SMGR. The host may report offset of the receiver's outputs with this message as well. The receiver has to be configured using UBX-CFG-SMGR to enable the use of the external measurement messages. Otherwise the receiver will ignore them.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | numMeas | U1 | Number of measurements in repeated block |
| 2 | 2 | reserved1 | U1[2] | Reserved |
| 4 | 4 | iTOW | U4 | Time of the week (ms) |
| 8 | 4 | reserved2 | U1[4] | Reserved |

**Repeated block (numMeas times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 12 + 24*N | 1 | sourceId | U1 | Index of source. SMEAS can provide six measurement sources. The first four sourceId values represent measurements made by the receiver and sent to the host. sourceId 0=internal oscillator measurement, 1=GNSS, 2=EXTINT0, 3=EXTINT1. The remaining two (sourceId 4 and 5) are made by the host and sent to the receiver: 4=host measurement of internal oscillator, 5=host measurement of external oscillator. |
| 13 + 24*N | 1 | flags | X1 | Flags (see bitfield below) |
| 14 + 24*N | 1 | phaseOffsetFrac | I1 | Sub-nanosecond phase offset (ns * 2^-8); the total offset is the sum of phaseOffset and phaseOffsetFrac |
| 15 + 24*N | 1 | phaseUncFrac | U1 | Sub-nanosecond phase uncertainty (ns * 2^-8) |
| 16 + 24*N | 4 | phaseOffset | I4 | Phase offset (ns), positive if the source lags accurate phase and negative if the source is early |
| 20 + 24*N | 4 | phaseUnc | U4 | Phase uncertainty (one standard deviation) (ns) |
| 24 + 24*N | 4 | reserved3 | U1[4] | Reserved |
| 28 + 24*N | 4 | freqOffset | I4 | Frequency offset (ppb * 2^-8), positive if the source frequency is too high, negative if the frequency is too low |
| 32 + 24*N | 4 | freqUnc | U4 | Frequency uncertainty (one standard deviation) (ppb * 2^-8) |

### Bitfield: flags
| Name | Description |
|------|-------------|
| freqValid | 1 = frequency measurement is valid |
| phaseValid | 1 = phase measurement is valid |

---

## UBX-TIM-SVIN (0x0D 0x04)
Survey-in data

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync or Time Sync products)  
**Length:** 28 bytes

This message contains information about survey-in parameters. For details about the Time mode see section Time mode configuration.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | dur | U4 | Passed survey-in observation time (s) |
| 4 | 4 | meanX | I4 | Current survey-in mean position ECEF X coordinate (cm) |
| 8 | 4 | meanY | I4 | Current survey-in mean position ECEF Y coordinate (cm) |
| 12 | 4 | meanZ | I4 | Current survey-in mean position ECEF Z coordinate (cm) |
| 16 | 4 | meanV | U4 | Current survey-in mean position 3D variance (mm^2) |
| 20 | 4 | obs | U4 | Number of position observations used during survey-in |
| 24 | 1 | valid | U1 | Survey-in position validity flag, 1 = valid, otherwise 0 |
| 25 | 1 | active | U1 | Survey-in in progress flag, 1 = in-progress, otherwise 0 |
| 26 | 2 | reserved1 | U1[2] | Reserved |

---

## UBX-TIM-TM2 (0x0D 0x03)
Time mark data

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 28 bytes

This message contains information for high precision time stamping / pulse counting. The delay figures and timebase given in UBX-CFG-TP5 are also applied to the time results output in this message.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | ch | U1 | Channel (i.e. EXTINT) upon which the pulse was measured |
| 1 | 1 | flags | X1 | Bitmask (see bitfield below) |
| 2 | 2 | count | U2 | Rising edge counter |
| 4 | 2 | wnR | U2 | Week number of last rising edge |
| 6 | 2 | wnF | U2 | Week number of last falling edge |
| 8 | 4 | towMsR | U4 | Tow of rising edge (ms) |
| 12 | 4 | towSubMsR | U4 | Millisecond fraction of tow of rising edge in nanoseconds (ns) |
| 16 | 4 | towMsF | U4 | Tow of falling edge (ms) |
| 20 | 4 | towSubMsF | U4 | Millisecond fraction of tow of falling edge in nanoseconds (ns) |
| 24 | 4 | accEst | U4 | Accuracy estimate (ns) |

### Bitfield: flags
| Name | Description |
|------|-------------|
| mode | 0=single, 1=running |
| run | 0=armed, 1=stopped |
| newFallingEdge | New falling edge detected |
| timeBase | 0=Time base is Receiver time, 1=Time base is GNSS time (the system according to the configuration in UBX-CFG-TP5 for tpIdx=0), 2=Time base is UTC (the variant according to the configuration in UBX-CFG-NAV5) |
| utc | 0=UTC not available, 1=UTC available |
| time | 0=Time is not valid, 1=Time is valid (Valid GNSS fix) |
| newRisingEdge | New rising edge detected |

---

## UBX-TIM-TOS (0x0D 0x12)
Time pulse time and frequency data

**Type:** Periodic  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 56 bytes

This message contains information about the time pulse that has just happened and the state of the disciplined oscillators(s) at the time of the pulse. It gives the UTC and GNSS times and time uncertainty of the pulse together with frequency and frequency uncertainty of the disciplined oscillators. It also supplies leap second information.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x00 for this version) |
| 1 | 1 | gnssId | U1 | GNSS system used for reporting GNSS time (see Satellite Numbering) |
| 2 | 2 | reserved1 | U1[2] | Reserved |
| 4 | 4 | flags | X4 | Flags (see bitfield below) |
| 8 | 2 | year | U2 | Year of UTC time |
| 10 | 1 | month | U1 | Month of UTC time |
| 11 | 1 | day | U1 | Day of UTC time |
| 12 | 1 | hour | U1 | Hour of UTC time |
| 13 | 1 | minute | U1 | Minute of UTC time |
| 14 | 1 | second | U1 | Second of UTC time |
| 15 | 1 | utcStandard | U1 | UTC standard identifier: 0=unknown, 3=USNO, 6=Soviet Union, 7=NTSC China |
| 16 | 4 | utcOffset | I4 | Time offset between the preceding pulse and UTC top of second (ns) |
| 20 | 4 | utcUncertainty | U4 | Uncertainty of utcOffset (ns) |
| 24 | 4 | week | U4 | GNSS week number |
| 28 | 4 | TOW | U4 | GNSS time of week (s) |
| 32 | 4 | gnssOffset | I4 | Time offset between the preceding pulse and GNSS top of second (ns) |
| 36 | 4 | gnssUncertainty | U4 | Uncertainty of gnssOffset (ns) |
| 40 | 4 | intOscOffset | I4 | Internal oscillator frequency offset (ppb * 2^-8) |
| 44 | 4 | intOscUncertainty | U4 | Internal oscillator frequency uncertainty (ppb * 2^-8) |
| 48 | 4 | extOscOffset | I4 | External oscillator frequency offset (ppb * 2^-8) |
| 52 | 4 | extOscUncertainty | U4 | External oscillator frequency uncertainty (ppb * 2^-8) |

### Bitfield: flags
| Name | Description |
|------|-------------|
| leapNow | 1 = currently in a leap second |
| leapSoon | 1 = leap second scheduled in current minute |
| leapPositive | 1 = positive leap second |
| timeInLimit | 1 = time pulse is within tolerance limit (UBX-CFG-SMGR timeTolerance field) |
| intOscInLimit | 1 = internal oscillator is within tolerance limit (UBX-CFG-SMGR freqTolerance field) |
| extOscInLimit | 1 = external oscillator is within tolerance limit (UBX-CFG-SMGR freqTolerance field) |
| gnssTimeValid | 1 = GNSS time is valid |
| UTCTimeValid | 1 = UTC time is valid |
| DiscSrc | Disciplining source identifier: 0=internal oscillator, 1=GNSS, 2=EXTINT0, 3=EXTINT1, 4=internal oscillator measured by the host, 5=external oscillator measured by the host |
| raim | 1 = (T)RAIM system is currently active. Note this flag only reports the current state of the GNSS solution; it is not affected by whether or not the GNSS solution is being used to discipline the oscillator. |
| cohPulse | 1 = coherent pulse generation is currently in operation |
| lockedPulse | 1 = time pulse is locked |

---

## UBX-TIM-TP (0x0D 0x01)
Time pulse time data

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3 and 22  
**Length:** 16 bytes

This message contains information on the timing of the next pulse at the TIMEPULSE0 output. The recommended configuration when using this message is to set both the measurement rate (UBX-CFG-RATE) and the timepulse frequency (UBX-CFG-TP5) to 1 Hz. For more information see section Time pulse.

**Note:** TIMEPULSE0 and this message are not available from DR products using the dedicated I2C sensor interface, including NEO-M8L and NEO-M8U modules.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | towMS | U4 | Time pulse time of week according to time base (ms) |
| 4 | 4 | towSubMS | U4 | Submillisecond part of towMS (ms * 2^-32) |
| 8 | 4 | qErr | I4 | Quantization error of time pulse (ps) |
| 12 | 2 | week | U2 | Time pulse week number according to time base (weeks) |
| 14 | 1 | flags | X1 | Flags (see bitfield below) |
| 15 | 1 | refInfo | X1 | Time reference information (see bitfield below) |

### Bitfield: flags
| Name | Description |
|------|-------------|
| timeBase | 0 = Time base is GNSS, 1 = Time base is UTC |
| utc | 0 = UTC not available, 1 = UTC available |
| raim | (T)RAIM information: 0=Information not available, 1=Not active, 2=Active |
| qErrInvalid | 0 = Quantization error valid, 1 = Quantization error invalid |
| TpNotLocked | 0 = Next TP is locked to GNSS, 1 = Next TP is based on local time and not locked to GNSS - week/tow may be invalid |

### Bitfield: refInfo
| Name | Description |
|------|-------------|
| timeRefGnss | GNSS reference information. Only valid if time base is GNSS (timeBase=0). 0=GPS, 1=GLONASS, 2=BeiDou, 3=Galileo, 4=NavIC, 15=Unknown |
| utcStandard | UTC standard identifier. Only valid if time base is UTC (timeBase=1). 0=Information not available, 1=CRL Tokyo, 2=NIST, 3=USNO, 4=BIPM, 5=European laboratories, 6=Soviet Union, 7=NTSC China, 8=NPLI India, 15=Unknown |

---

## UBX-TIM-VCOCAL (0x0D 0x15)
### Stop calibration command
Stop calibration

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 1 byte

Stop all ongoing calibration (both oscillators are affected)

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | type | U1 | Message type (0 for this message) |

### VCO calibration extended command
VCO calibration extended command

**Type:** Command  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 12 bytes

Calibrate (measure) gain of the voltage controlled oscillator. The calibration is performed by varying the raw oscillator control values between the limits specified in raw0 and raw1. maxStepSize is the largest step change that can be used during the calibration process. The "raw values" are either PWM duty cycle values or DAC values depending on how the VCTCXO is connected to the system. The measured gain is the transfer function dRelativeFrequencyChange/dRaw (not dFrequency/dVoltage).

**Important notes:**
- Care must be taken when calibrating the internal oscillator against the GNSS source as severe frequency changes could lose satellite signal tracking
- Only the chosen frequency source should be enabled during calibration and remain stable throughout
- See full description in raw text for complete calibration process details

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | type | U1 | Message type (2 for this message) |
| 1 | 1 | version | U1 | Message version (0x00 for this version) |
| 2 | 1 | oscId | U1 | Oscillator to be calibrated: 0=internal oscillator, 1=external oscillator |
| 3 | 1 | srcId | U1 | Reference source: 0=internal oscillator, 1=GNSS, 2=EXTINT0, 3=EXTINT1. Option 0 should be used when calibrating the external oscillator. Options 1-3 should be used when calibrating the internal oscillator. |
| 4 | 2 | reserved1 | U1[2] | Reserved |
| 6 | 2 | raw0 | U2 | First value used for calibration (raw values) |
| 8 | 2 | raw1 | U2 | Second value used for calibration (raw values) |
| 10 | 2 | maxStepSize | U2 | Maximum step size to be used (raw values) |

### Results of the calibration
Results of the calibration

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01 (only with Time & Frequency Sync products)  
**Length:** 12 bytes

This message is sent when the oscillator gain calibration process is finished (successful or unsuccessful). It notifies the user of the calibrated oscillator gain. If the oscillator gain calibration process was successful, this message will contain the measured gain (field gainVco) and its uncertainty (field gainUncertainty). The calibration process can however fail. In that case the two fields gainVco and gainUncertainty are set to zero.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | type | U1 | Message type (3 for this message) |
| 1 | 1 | version | U1 | Message version (0x00 for this version) |
| 2 | 1 | oscId | U1 | Id of oscillator: 0=internal oscillator, 1=external oscillator |
| 3 | 3 | reserved1 | U1[3] | Reserved |
| 6 | 2 | gainUncertainty | U2 | Relative gain uncertainty after calibration (1/1 * 2^-16), 0 if calibration failed |
| 8 | 4 | gainVco | I4 | Calibrated gain (ppb/raw LSB * 2^-16) or 0 if calibration failed |

---

## UBX-TIM-VRFY (0x0D 0x06)
Sourced time verification

**Type:** Periodic/Polled  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 20 bytes

This message contains verification information about previous time received via assistance data or from RTC.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 4 | itow | I4 | integer millisecond tow received by source (ms) |
| 4 | 4 | frac | I4 | sub-millisecond part of tow (ns) |
| 8 | 4 | deltaMs | I4 | integer milliseconds of delta time (current time minus sourced time) (ms) |
| 12 | 4 | deltaNs | I4 | Sub-millisecond part of delta time (ns) |
| 16 | 2 | wno | U2 | Week number (week) |
| 18 | 1 | flags | X1 | Flags (see bitfield below) |
| 19 | 1 | reserved1 | U1 | Reserved |

### Bitfield: flags
| Name | Description |
|------|-------------|
| src | Aiding time source: 0=no time aiding done, 2=source was RTC, 3=source was assistance data |
