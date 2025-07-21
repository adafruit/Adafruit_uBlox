# Adafruit uBlox Library [![Build Status](https://github.com/adafruit/Adafruit_uBlox/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_uBlox/actions)

This is a driver library to abstract away the details of communicating with u-blox GPS and RTK modules. It provides a simple interface for sending and receiving data over a generic Stream interface.

This library provides two main classes:
- `Adafruit_UBX`: Interfaces with u-blox GPS/RTK modules using any stream object as an input (UART, DDC, or other) and parses UBX protocol messages.
- `Adafruit_UBloxDDC`: Interface for communicating with u-blox modules over DDC (I2C).

For parsing NMEA sentences, we provide an example for using this library with the [Adafruit_GPS library](https://github.com/adafruit/Adafruit_GPS).

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

MIT license, all text above must be included in any redistribution