# SAM-M8Q Hardware Integration Guide

Reference: SAM-M8Q Data Sheet (UBX-16012619) and Hardware Integration Manual (UBX-16018358).

## Pin Reference

| Pin | Name       | I/O | Description                          |
|-----|------------|-----|--------------------------------------|
| 1   | GND        |     | Ground                               |
| 2   | VCC_IO     | I   | IO supply voltage (2.7–3.6V)         |
| 3   | V_BCKP     | I   | Backup battery supply (1.4–3.6V)     |
| 7   | TIMEPULSE  | O   | 1 PPS output (default 1Hz)           |
| 8   | SAFEBOOT_N | I   | Reserved — leave unconnected or HIGH |
| 9   | SDA        | I/O | DDC (I2C) data                       |
| 12  | SCL        | I   | DDC (I2C) clock (max 400 kHz)        |
| 13  | TxD        | O   | UART transmit                        |
| 14  | RxD        | I   | UART receive                         |
| 17  | VCC        | I   | Main supply (2.7–3.6V, typ 3.0V)    |
| 18  | RESET_N    | I   | Active low reset (11kΩ internal pull-up) |
| 19  | EXTINT0    | I   | External interrupt / wakeup          |
| 4,5,6,10,11,15,16,20 | GND | | Ground pads |

## Power Supply

- **VCC:** 2.7–3.6V (typ 3.0V). Max current 67mA peak, ~29mA continuous tracking (GPS+GLONASS).
- **VCC_IO:** 2.7V to min(VCC+0.3, 3.6V). Can tie to VCC if running at 3.3V.
- **V_BCKP:** 1.4–3.6V. Connect a coin cell (CR1220 etc.) for hot/warm start. Draws ~15µA at 1.8V.
  If unused, leave floating or tie to VCC.
- **Decoupling:** 100nF ceramic close to VCC and VCC_IO. 10µF bulk on VCC recommended.
- No overvoltage protection built in — add TVS diodes if supply may exceed 3.6V.

### Power Modes (typical at 3.0V)

| Mode                   | GPS+GLONASS | GPS only |
|------------------------|-------------|----------|
| Acquisition            | 32 mA       | 25 mA    |
| Continuous tracking    | 29 mA       | 23 mA    |
| Power Save (1Hz cycle) | 9.5 mA      | 9.5 mA   |

## I2C / DDC Interface

- Default address: **0x42** (7-bit), not configurable.
- I2C Fast Mode compatible (up to 400 kHz SCL).
- Slave mode only — the module never initiates transactions.
- **Pull-ups:** 4.7kΩ on SDA and SCL to VCC_IO. Most breakout boards include these.
- The DDC interface shares protocol selection with UART — use UBX-CFG-PRT to configure
  which protocols (NMEA, UBX, RTCM) are active on each port.
- **I2C write chunking:** Messages longer than 32 bytes must be split into chunks
  (Wire library limitation). The Adafruit_UBloxDDC driver handles this automatically.

## UART Interface

- Default baud: **9600** (configurable via UBX-CFG-PRT).
- 8N1 format.
- TxD is output (connect to host RxD), RxD is input (connect to host TxD).
- Supports NMEA, UBX, and RTCM protocols simultaneously.
- Common baud rates: 9600, 19200, 38400, 57600, 115200, 230400.

## Reset (RESET_N)

- Active low with **11kΩ internal pull-up** — no external pull-up needed.
- Drive LOW for ≥100ms to reset the module, then release (set HIGH or tri-state).
- After reset release, allow **≥1.5 seconds** for the module to boot before communicating.
- **RP2040 note:** GPIO pins float at power-on, which can hold the module in reset.
  Drive RESET_N HIGH immediately in setup(), before any delays or Serial init.

## TIMEPULSE / PPS

- Outputs a 1Hz pulse synchronized to GPS/UTC time (configurable via UBX-CFG-TP5).
- **Requires a valid fix** to produce pulses. No fix = no PPS.
- Open-drain output — use a pull-up resistor (10kΩ to VCC_IO) if connecting to a
  high-impedance input. Most MCU internal pull-ups are sufficient for reading.
- Pulse width and frequency configurable from sub-Hz to 10 MHz.

## EXTINT (External Interrupt)

- Fixed voltage thresholds relative to VCC_IO.
- Primary uses:
  - **Power Save Mode control:** Force receiver ON/OFF.
  - **Time aiding:** Connect an accurate time pulse for time synchronization.
  - **Frequency aiding:** Rectangular signal up to 500 kHz (min 50ns high/low phase).
- Can force receiver OFF even when Power Save Mode is not active.
- Leave floating or drive LOW if not used.

## SAFEBOOT_N

- Reserved pin for firmware recovery.
- **Leave unconnected or tie HIGH** for normal operation.
- Designs should allow access to this pin and UART for future service/firmware updates.

## Embedded Antenna

- Integrated GNSS patch antenna (RHCP, 3 dBiC peak gain).
- Internal SAW filter + LNA — no external active antenna components needed.
- **Ground plane:** Optimal performance with **50×50mm** ground plane beneath the module.
  Smaller ground planes reduce gain and radiation efficiency.
- **Keep-out zone:** No copper, traces, or components above the module's antenna area.
  See Hardware Integration Manual for exact keep-out dimensions.
- **Orientation:** Antenna faces skyward. Mount on top layer of PCB with clear sky view.
- **Tuning marks:** Small scratches on the antenna element are from factory RF tuning — not defects.
- The module supports GPS, GLONASS, and Galileo simultaneously. Galileo is disabled by
  default — enable via UBX-CFG-GNSS.

## Wiring Diagram (QT Py RP2040 Test Setup)

This is the wiring used by the `hw_tests/full_pin_test` sketch:

```
QT Py RP2040          SAM-M8Q Breakout
─────────────         ────────────────
3V3         ───────── VCC, VCC_IO
GND         ───────── GND
SDA (Wire)  ───────── SDA (pin 9)
SCL (Wire)  ───────── SCL (pin 12)
TX (Serial1)───────── RxD (pin 14)
RX (Serial1)───────── TxD (pin 13)
A0          ───────── RESET_N (pin 18)
A1          ───────── TIMEPULSE (pin 7)
A2          ───────── EXTINT0 (pin 19)
```

Optional: coin cell on V_BCKP for hot start capability.

## Hardware Test Coverage

The `full_pin_test` sketch verifies:

1. **I2C / DDC** — Connection at 0x42, UBX NAV-PVT poll/response
2. **Hardware Reset** — Toggle RESET_N low, verify module recovers
3. **UART Auto-Baud** — Probe at 9600/57600/115200
4. **UART Communication** — Raw UBX NAV-PVT poll and parse over Serial1
5. **PPS Signal** — Count rising edges over 3.5s (requires GPS fix)
6. **EXTINT Pin** — Read idle state (informational)

## Design Checklist

- [ ] VCC and VCC_IO decoupled with 100nF + 10µF
- [ ] I2C pull-ups present (4.7kΩ or breakout-provided)
- [ ] RESET_N driven HIGH early in firmware (especially on RP2040/floating-pin MCUs)
- [ ] SAFEBOOT_N accessible for firmware recovery
- [ ] UART accessible for firmware updates
- [ ] Ground plane ≥50×50mm under module
- [ ] No components/traces in antenna keep-out zone
- [ ] Module mounted antenna-up with clear sky view
- [ ] Backup battery on V_BCKP if hot/warm start needed
- [ ] ESD protection if antenna area is user-touchable

## ESD Precautions

The SAM-M8Q is an electrostatic sensitive device. Standard ESD precautions apply:
- Use ESD-safe soldering iron
- Ground yourself when handling
- Exposed antenna areas are particularly vulnerable — add ESD protection if the module
  is in a user-accessible location

## References

1. SAM-M8Q Hardware Integration Manual (UBX-16018358)
2. u-blox 8 / M8 Receiver Description Including Protocol Specification (UBX-13003221)
3. Power Management Application Note (UBX-13005162)
