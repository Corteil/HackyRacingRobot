# Serial Protocol: Host ↔ Yukon

Communication between the Raspberry Pi (host) and the Pimoroni Yukon (RP2040) uses a compact 5-byte packet format. All bytes are printable ASCII so they do not interfere with the MicroPython REPL.

---

## Packet format (Host → Yukon)

```
[SYNC, CMD, V_HIGH, V_LOW, CHK]
```

| Field  | Encoding                     | Range             |
|--------|------------------------------|-------------------|
| SYNC   | `0x7E` (`~`) — fixed marker  | always `0x7E`     |
| CMD    | `cmd_code + 0x20`            | `0x21–0x25`       |
| V_HIGH | `(value >> 4) + 0x40`        | `0x40–0x4F`       |
| V_LOW  | `(value & 0xF) + 0x50`       | `0x50–0x5F`       |
| CHK    | `CMD ^ V_HIGH ^ V_LOW`       | `49–62`, never equals SYNC |

Response: `ACK` (`0x06`) on success, `NAK` (`0x15`) on any framing or checksum error.

---

## Commands

| Command      | Code | Value meaning |
|--------------|------|---------------|
| `CMD_LED`    | 1    | 0 = LED_A off, 1 = LED_A on, 2 = LED_B off, 3 = LED_B on |
| `CMD_LEFT`   | 2    | Motor speed byte (see below) |
| `CMD_RIGHT`  | 3    | Motor speed byte |
| `CMD_KILL`   | 4    | Ignored — zeros both motors immediately |
| `CMD_SENSOR` | 5    | Ignored — device replies with 7 sensor data packets then ACK |

---

## Motor speed encoding

Speed is encoded as a single byte in the range 0–200:

| Speed          | Byte value |
|----------------|------------|
| 0% (stop)      | 0          |
| +50% (forward) | 50         |
| +100% (forward)| 100        |
| −50% (reverse) | 150        |
| −100% (reverse)| 200        |

Decode logic on the Yukon:
- `byte <= 100` → `speed = byte / 100` (forward)
- `byte > 100`  → `speed = -(byte - 100) / 100` (reverse)

---

## Sensor response (Device → Host)

`CMD_SENSOR` triggers 7 data packets followed by ACK. Each data packet uses the same 5-byte wire format with `RESP_TYPE` replacing `CMD`:

| ID | Name             | Scale factor   | Unit |
|----|------------------|----------------|------|
| 0  | Voltage          | raw ÷ 10       | V    |
| 1  | Current          | raw ÷ 100      | A    |
| 2  | Board temp       | raw ÷ 3        | °C   |
| 3  | Left module temp | raw ÷ 3        | °C   |
| 4  | Right module temp| raw ÷ 3        | °C   |
| 5  | Left fault       | 1.0 (raw = 0/1)| —    |
| 6  | Right fault      | 1.0 (raw = 0/1)| —    |

`RESP_TYPE` encoding: `resp_id + 0x30` (range `0x30–0x36`).
