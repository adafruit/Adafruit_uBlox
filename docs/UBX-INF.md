# UBX-INF (0x04) - Information Messages

Printf-Style Messages, with IDs such as Error, Warning, Notice

Messages in the INF class are used to output strings in a printf style from the firmware or application code. All INF messages have an associated type to indicate the kind of message.

---

## UBX-INF-DEBUG (0x04 0x04)
ASCII output with debug contents

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 + 1*N bytes

This message has a variable length payload, representing an ASCII string.

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N*1 | 1 | str | CH | ASCII Character |

---

## UBX-INF-ERROR (0x04 0x00)
ASCII output with error contents

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 + 1*N bytes

This message has a variable length payload, representing an ASCII string.

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N*1 | 1 | str | CH | ASCII Character |

---

## UBX-INF-NOTICE (0x04 0x02)
ASCII output with informational contents

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 + 1*N bytes

This message has a variable length payload, representing an ASCII string.

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N*1 | 1 | str | CH | ASCII Character |

---

## UBX-INF-TEST (0x04 0x03)
ASCII output with test contents

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 + 1*N bytes

This message has a variable length payload, representing an ASCII string.

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N*1 | 1 | str | CH | ASCII Character |

---

## UBX-INF-WARNING (0x04 0x01)
ASCII output with warning contents

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 0 + 1*N bytes

This message has a variable length payload, representing an ASCII string.

**Repeated block (N times):**

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| N*1 | 1 | str | CH | ASCII Character |
