# UBX-SEC (0x27) - Security Feature Messages

Security Features

Messages in the SEC class are used for security features of the receiver.

---

## UBX-SEC-UNIQID (0x27 0x03)
Unique chip ID

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 9 bytes

This message is used to retrieve a unique chip identifier (40 bits, 5 bytes).

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | version | U1 | Message version (0x01 for this version) |
| 1 | 3 | reserved1 | U1[3] | Reserved |
| 4 | 5 | uniqueId | U1[5] | Unique chip ID |
