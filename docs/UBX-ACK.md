# UBX-ACK (0x05) - Ack/Nak Messages

Acknowledge or Reject messages to UBX-CFG input messages

Messages in the UBX-ACK class output the processing results to UBX-CFG and some other messages.

---

## UBX-ACK-ACK (0x05 0x01)
Message acknowledged

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 2 bytes

Output upon processing of an input message. A UBX-ACK-ACK is sent as soon as possible but at least within one second.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | clsID | U1 | Class ID of the Acknowledged Message |
| 1 | 1 | msgID | U1 | Message ID of the Acknowledged Message |

---

## UBX-ACK-NAK (0x05 0x00)
Message not acknowledged

**Type:** Output  
**Firmware:** u-blox 8 / u-blox M8 protocol versions 15, 15.01, 16, 17, 18, 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 22.01, 23 and 23.01  
**Length:** 2 bytes

Output upon processing of an input message. A UBX-ACK-NAK is sent as soon as possible but at least within one second.

| Offset | Size | Name | Type | Description |
|--------|------|------|------|-------------|
| 0 | 1 | clsID | U1 | Class ID of the Not-Acknowledged Message |
| 1 | 1 | msgID | U1 | Message ID of the Not-Acknowledged Message |
