# Wire formats

Everything that goes on the air in this system, byte by byte, with the numbers
you need to decode a hex dump in a stairwell.

There are two links and they do not carry the same bytes:

```
  CAR (battery)                 BRIDGE, floor 5              DISPLAYS, floors 1-10
  XIAO + SX1262                 XIAO + SX1262                CYD x10
        |                             |                            |
        |  LoRa 915 MHz               |  ESP-NOW, 802.11 LR        |
        |  SF10/BW125/CR4-8           |  channel 1, flooded        |
        |  0xE0 STATE  8 B            |  0x5E1E envelope           |
        |  0xE1 STATS 24 B            |  15 B / 31 B on the wire   |
        +---------------------------->+--------------------------->+
                                                                   |
                                                            displays relay
                                                            to each other too
```

The bridge does not re-broadcast the LoRa bytes verbatim. It strips three bytes
(`tag`, `txId`, `seq`), puts the remaining body inside an ESP-NOW envelope with
its own sequence number and CRC, and floods that. So a display never sees `txId`
or the LoRa `seq` - those two exist only on the LoRa hop. Sections 5 and 6
explain why that is deliberate.

**Source of truth.** Every offset, mask and constant below is taken from
`src/elev_packet.h`, `src/mesh_packet.h` and `src/crc16.h`. Where the code and
the design spec's section 4 disagree, **the code is right** - the spec is the
older document. The disagreements found are small and are listed in section 8;
none of them touch a field offset.

---

## 1. The LoRa radio parameters

| parameter | value | build flag (`[lora_base]`) |
|---|---|---|
| frequency | 915.0 MHz | `LORA_FREQUENCY` |
| bandwidth | 125 kHz | `LORA_BANDWIDTH` |
| spreading factor | 10 | `LORA_SPREADING_FACTOR` |
| coding rate | 4/8 | `LORA_CODING_RATE` |
| sync word | 0x34 | `LORA_SYNC_WORD` |
| TX power | 22 dBm | `LORA_TX_POWER` |
| preamble | 8 symbols | `LORA_PREAMBLE_LEN` |
| header | explicit | RadioLib default |
| hardware CRC | on | RadioLib default |
| LDRO | off | `Ts` = 8.192 ms < 16 ms, so it is not required |

Both ends are the same module (Wio-SX1262 on a XIAO ESP32S3) and both are
configured from the same `[lora_base]` section, so there is no way for the car
and the bridge to end up on different settings without editing one file.

### Where BW125 came from

`ALGORITHM.md` §9 records one field run in this shaft: **SF10 / BW125 / 14 dBm,
5.21% batch loss over 3 h**, 49 dropouts (one per 3.7 min), 90% of them a single
10 s batch, worst case 40 s. (That worst hole is 40 missing samples spanning
41 s between the two real samples either side of it, which is why the replay
harness and `docs/ALGORITHM_PORT.md` §6 call it 41 s.)

`elevatormons/platformio.ini` sets its `baro_base` environment to BW62.5. Nothing
was ever measured at BW62.5 in this shaft. This project uses **BW125 because that
is the configuration behind the only loss number anyone has**, and runs it at
22 dBm - 8 dB more than the run that produced 5.21%.

| config | 8-byte airtime | sensitivity | in-shaft evidence |
|---|---|---|---|
| SF10/BW62.5/CR4-8 | 725 ms | -135 dBm | none |
| **SF10/BW125/CR4-8** | **297 ms** | -132 dBm | 5.21% loss at 14 dBm |

The temptation is to spend airtime on a higher spreading factor. §9 argues
against it directly: loss barely tracks SNR (r = -0.13), there is still 0.3% loss
above +6 dB of margin, and the worst SNR bin only loses 5.6%. That is a shadowing
and interference pattern, not a decode floor, so SF buys margin the link was
never short of while costing airtime linearly. The one genuine weak spot - floor
9, bottoming at -2.0 dB SNR against a +14.2 dB median - is an antenna placement
problem, which is why the bridge sits on floor 5 with its antenna favouring the
upper shaft.

CR 4/8 is kept because the shaft is a multipath environment and forward error
correction is the cheapest thing that recovers a packet chewed up by a fade.

### 1.1 Airtime, and how to re-cost a packet

Airtime is the power budget. The car averages ~19 mA and every millisecond on
the air costs ~140 mA. Before adding a field to either format, run this:

```
Ts        = 2^SF / BW              = 2^10 / 125000        = 8.192 ms
t_preamble = (n_pre + 4.25) * Ts   = (8 + 4.25) * 8.192   = 100.352 ms

n_payload = 8 + ceil( (8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE)) )
                * (CR + 4)

  PL   payload bytes            SF  = 10
  CRC  1 (hardware CRC on)      IH  = 0 (explicit header)
  DE   0 (LDRO off)             CR  = 4 (for coding rate 4/8)

n_payload = 8 + ceil( (8*PL + 4) / 40 ) * 8
t_air     = t_preamble + n_payload * Ts
```

Worked for the two formats in use:

```
PL =  8 -> 8 + ceil( 68/40)*8 = 24 sym -> 196.608 ms -> total 296.96 ms ~ 297 ms
PL = 24 -> 8 + ceil(196/40)*8 = 48 sym -> 393.216 ms -> total 493.57 ms ~ 494 ms
```

The `ceil(...)*8` is the part that bites: **airtime quantises in 8-symbol steps
of 65.536 ms.** Payload length does not cost airtime smoothly - it costs nothing
for several bytes and then costs 65.5 ms all at once.

| PL (bytes) | symbols | airtime | note |
|---|---|---|---|
| 5 | 24 | 297 ms | |
| 8 | 24 | 297 ms | **STATE today** |
| 9 | 24 | 297 ms | one free byte |
| 10 | 32 | 362 ms | +65.5 ms |
| 21 | 48 | 494 ms | |
| 24 | 48 | 494 ms | **STATS today, at the top of its step** |
| 25 | 56 | 559 ms | +65.5 ms |
| 32 | 64 | 625 ms | |

What a step costs in current, at the spec §2.2 duty cycles and 140 mA:

| change | added current |
|---|---|
| STATE 8 -> 10 bytes, every 2 s at 71.2% moving | **+3.3 mA** on a ~19 mA budget |
| STATS 24 -> 25 bytes, every 60 s | +0.15 mA |

Sanity check on the same arithmetic against the spec's own line items:
`297 ms / 2 s x 140 mA x 0.712 = 14.8 mA` and `494 ms / 60 s x 140 mA = 1.15 mA`,
which is the 15 mA and 1.2 mA in spec §2.2.

### 1.2 Why neither LoRa format carries an application CRC

The SX1262 computes and checks a hardware CRC-16 in the modem, and it is enabled
on both ends. RadioLib reports a CRC failure as a receive error - the frame never
reaches application code. `loraPoll()` returns `true` with `*outState` set to the
failure so the bridge can count it, and the payload in that case is explicitly
not to be trusted (`lora_link.h`).

A second, application-level CRC would spend 2 bytes re-checking bytes that have
already been checked. Given the 8-symbol quantisation above, 2 bytes is enough to
push a packet into the next step: STATS at 24 is already at the ceiling of its
step, so a CRC there would be a straight 65.5 ms. The `tag` byte stays as the
guard against a foreign frame that happens to be the right length and to pass
the same sync word.

Note that the hardware CRC is already in the airtime numbers - it is the
`16*CRC` term in the formula above.

The ESP-NOW leg is different and **does** carry a CRC. Section 5 explains why.

---

## 2. `0xE0` STATE - 8 bytes, 297 ms

Sent every 2 s while the car is moving and for 10 s after it stops
(`STATE_INTERVAL_MS`, `STATE_HOLD_AFTER_STOP_MS`). The first packet fires
immediately on motion onset, which the transmitter catches within 250 ms by
polling the stillness test faster than the algorithm runs.

All multi-byte fields are little-endian, written out byte by byte rather than
memcpy'd from a struct - the format does not depend on three chips happening to
agree about padding.

| off | size | type | field | units / range | meaning |
|---|---|---|---|---|---|
| 0 | 1 | u8 | `tag` | 0xE0 | format identifier, `ELEV_TAG_STATE` |
| 1 | 1 | u8 | `txId` | 0-255 | transmitter identity. Only one car node exists today; the bridge drops this before the mesh |
| 2 | 1 | u8 | `seq` | 0-255, wraps | loss statistics only - see section 6 |
| 3 | 1 | u8 | `floor` | 0-255 | confirmed 1-based floor index. **0 means the model is not ready** and the floor is being withheld. Index 1 is the lowest landing ever seen, not a name - the display maps index to label through `FLOOR_LABELS` |
| 4 | 2 | i16 | `posQ8` | 1/256 floor, +/-128 floors | live fractional position relative to floor 1. **Animation only** - see section 4 |
| 6 | 1 | u8 | `state` | bit field | see below |
| 7 | 1 | u8 | `confidence` | 0-255 | how cleanly the last jump landed on the floor lattice |

`confidence` is derived from `FloorBroadcast.confidence`, which `floor_monitor.h`
computes as `1.0 - 2.0 * off`, clamped at 0, where `off` is the distance from the
observed jump to the nearest integer multiple of the pitch. 1.0 (255 on the wire)
is a perfect lattice hit; 0 is half a floor off. An arrival with `off` above
`FLOOR_OFF_LATTICE` (0.30) is rejected as the car coasting through rather than
landing, so an ordinary confirmed arrival will not report below roughly 0.40 of
full scale - a low value means a forced resync, and that is the interesting case.

### The `state` byte at offset 6

```
  bit   7     6     5     4        3          2     1      0
      +-----+-----+-----+--------+----------+-----+-----+--------+
      |  -  |  -  |  -  |sensor  | model    |   direction  |moving|
      |     |     |     |  Err   |  Ready   |   (2 bits)   |      |
      +-----+-----+-----+--------+----------+-----+-----+--------+
        reserved, sent as 0
```

| mask | name | meaning |
|---|---|---|
| 0x01 | `ELEV_STATE_MOVING` | the car is moving by the algorithm's own stillness test |
| 0x06 | `ELEV_STATE_DIR_MASK` | direction, shifted left by `ELEV_STATE_DIR_SHIFT` = 1 |
| 0x08 | `ELEV_STATE_MODEL_READY` | pitch established; if clear, `floor` is 0 |
| 0x10 | `ELEV_STATE_SENSOR_ERR` | the BMP390 is not answering |

Direction values (`enum ElevDirection`): `0` idle, `1` up, `2` down. `3` is
unused. `elevStateByte()` masks the direction into two bits, so a bad direction
value becomes idle rather than corrupting `modelReady` next door.

Do not open-code these masks. `elev_packet.h` provides `elevStateMoving()`,
`elevStateDirection()`, `elevStateModelReady()`, `elevStateSensorErr()` and the
single assembler `elevStateByte()`, which is the only place the bit layout is
written down. The two-bit field at a shift is exactly the kind of thing that gets
shifted the wrong way in one place out of four.

### A real STATE packet, decoded

Car ascending between floor 3 and floor 4, model good, clean lattice fit:

```
  offset:   0    1    2    3    4    5    6    7
          +----+----+----+----+---------+----+----+
          | E0 | 01 | 2A | 03 |  9A 02  | 0B | F2 |
          +----+----+----+----+---------+----+----+
            |    |    |    |       |      |    |
            |    |    |    |       |      |    +-- confidence = 242
            |    |    |    |       |      |         242/255 = 0.95, so off = 0.025
            |    |    |    |       |      |         of a floor. A good landing.
            |    |    |    |       |      |
            |    |    |    |       |      +-- state = 0x0B = 0000 1011
            |    |    |    |       |           bit0 moving       = 1
            |    |    |    |       |           bits1-2 direction = 01 = UP
            |    |    |    |       |           bit3 modelReady   = 1
            |    |    |    |       |           bit4 sensorErr    = 0
            |    |    |    |       |
            |    |    |    |       +-- posQ8, little-endian: 0x029A = 666
            |    |    |    |            666/256 = 2.60 floors above floor 1
            |    |    |    |            -> display shows the digits sweeping
            |    |    |    |               through 3.60 on the 1-based scale
            |    |    |    |
            |    |    |    +-- floor = 3, the last CONFIRMED landing.
            |    |    |         Not 4. The car has left 3 and has not
            |    |    |         arrived anywhere yet.
            |    |    |
            |    |    +-- seq = 0x2A = 42
            |    +-- txId = 1
            +-- tag = 0xE0, STATE
```

The `floor`/`posQ8` split in that dump is the whole design in one line: `floor`
is the last thing the algorithm is willing to stand behind, `posQ8` is where the
car appears to be right now. They disagree during every trip, and that is
correct.

---

## 3. `0xE1` STATS - 24 bytes, 494 ms

Sent every 60 s unconditionally (`STATS_INTERVAL_MS`), moving or parked. It is
the heartbeat as well as the telemetry: the displays' 180 s staleness timer is
measured against it, so three consecutive misses are needed before a screen greys
out.

| off | size | type | field | units | range / meaning |
|---|---|---|---|---|---|
| 0 | 1 | u8 | `tag` | | 0xE1, `ELEV_TAG_STATS` |
| 1 | 1 | u8 | `txId` | | transmitter identity |
| 2 | 1 | u8 | `seq` | | wraps; loss statistics only |
| 3 | 2 | u16 | `batteryMv` | mV | pack voltage. A 4S LiFePO4 spans 11200-14600 mV. Divider is +/-2-3% uncalibrated |
| 5 | 2 | u16 | `dist24hM` | m | distance over the last 24 hourly buckets. Uptime-relative, not wall clock |
| 7 | 4 | u32 | `distTotalM` | m | lifetime odometer, survives reboot via NVS |
| 11 | 2 | u16 | `pitchMm` | mm | learned floor pitch. 2871 on the reference capture; the acceptance gate is +/-1 mm, which is why the unit is mm |
| 13 | 1 | u8 | `nFloors` | | learned floor count. 10 at Simmons |
| 14 | 2 | u16 | `trips` | | floor-to-floor moves counted |
| 16 | 2 | u16 | `stops` | | confirmed stops counted |
| 18 | 4 | u32 | `uptimeS` | s | seconds since boot |
| 22 | 1 | u8 | `flags` | bit field | see below |
| 23 | 1 | i8 | `tempC` | degC | BMP390 die temperature, signed |

`batteryMv` is millivolts rather than a float because the divider is only good to
+/-2-3% before a one-point `VBAT_CAL_NUM`/`VBAT_CAL_DEN` correction against a
DMM. A float would imply a precision the hardware does not have.

`trips` and `stops` are u16 and will wrap at 65535. The reference capture
produced 357 trips and 255 stops in 3 h - on the gap-filled stream the replay
uses, which is why it is not the 356/256 ALGORITHM.md §5 reports off the raw
radio log; `docs/ALGORITHM_PORT.md` §6 explains the one-in-each-direction
difference. Either way that is roughly a month of evening peaks before a wrap.
They are counters for interest, not for accounting.

### The `flags` byte at offset 22

```
  bit   7     6     5     4     3        2        1       0
      +-----+-----+-----+-----+--------+--------+-------+--------+
      |  -  |  -  |  -  |  -  | low    | nvs    | model | sensor |
      |     |     |     |     | Battery|Restored| Ready |  Err   |
      +-----+-----+-----+-----+--------+--------+-------+--------+
        reserved, sent as 0
```

| mask | name | meaning |
|---|---|---|
| 0x01 | `ELEV_FLAG_SENSOR_ERR` | the BMP390 is not answering |
| 0x02 | `ELEV_FLAG_MODEL_READY` | pitch established |
| 0x04 | `ELEV_FLAG_NVS_RESTORED` | the model came back from NVS on boot rather than being learned this run. Lets a display tell "restored and trusted" from "learned here" |
| 0x08 | `ELEV_FLAG_LOW_BATTERY` | below the 12.0 V warn threshold, ~20% remaining on a 4S LiFePO4 pack |

`lowBattery` is sent as a bit *as well as* a voltage on purpose. The threshold
lives on the transmitter, which is the only node that knows its own `VBAT_CAL_*`
correction. A display comparing `batteryMv` against a hardcoded number would be
comparing against an uncalibrated figure.

Accessors: `elevStatsSensorErr()`, `elevStatsModelReady()`,
`elevStatsNvsRestored()`, `elevStatsLowBattery()`, and the assembler
`elevStatsFlagsByte()`.

### A real STATS packet, decoded

Parked, healthy, a day into a run, model restored from NVS on the last reboot:

```
  offset:  0    1    2     3  4     5  6     7  8  9 10    11 12   13
         +----+----+----+-------+-------+-------------+-------+----+
         | E1 | 01 | 2B | C4 31 | 51 0D | B5 F5 01 00 | 37 0B | 0A |
         +----+----+----+-------+-------+-------------+-------+----+

  offset: 14 15   16 17   18 19 20 21    22   23
         +-------+-------+-------------+----+----+
         | 65 01 | FF 00 | 58 6E 01 00 | 06 | 16 |
         +-------+-------+-------------+----+----+

  E1           tag           STATS
  01           txId          1
  2B           seq           43
  C4 31        batteryMv     0x31C4 = 12740 mV = 12.74 V
                             13.3 V is rested-full and 12.0 V is the warn
                             threshold, so this pack is healthy
  51 0D        dist24hM      0x0D51 = 3409 m in the last 24 buckets
  B5 F5 01 00  distTotalM    0x0001F5B5 = 128437 m lifetime
  37 0B        pitchMm       0x0B37 = 2871 mm = 2.871 m  <- the learned pitch
  0A           nFloors       10
  65 01        trips         0x0165 = 357 floor-to-floor moves
  FF 00        stops         0x00FF = 255 confirmed stops
  58 6E 01 00  uptimeS       0x00016E58 = 93784 s = 26.1 h
  06           flags         0000 0110
                             bit0 sensorErr   = 0
                             bit1 modelReady  = 1
                             bit2 nvsRestored = 1   <- learned before this boot
                             bit3 lowBattery  = 0
  16           tempC         22 degC
```

If you are staring at a dump and the numbers look wrong, check the endianness
first. `37 0B` reads as 2871, not 14091. Everything multi-byte in both formats is
little-endian.

---

## 4. `posQ8`: what it is for, and what it is not for

This is the field a future reader is most likely to misuse, so it gets its own
section.

**What it is.** A signed 16-bit count of 1/256ths of a floor, measured from floor
1. `floor_monitor.h` computes the underlying quantity as
`posFloors = (level - datum) / pitch`, where `datum` is the current estimate of
the lowest landing's raw altitude. At the measured 2.871 m pitch, 1/256 of a
floor is 11 mm - well below what the barometer can resolve - and an i16 covers
+/-128 floors, which is more building than exists.

Two accessors, and nothing outside `elev_packet.h` should divide by 256:

- `elevStatePosFloors()` - signed offset from floor 1, in floors. Negative below floor 1.
- `elevStatePosFloorIndex()` - the same thing on the 1-based scale `floor` uses, so a display interpolates its digits between two labels without doing the `+1` itself.

**What it is for.** Animating the big digits. While the car moves, the display
sweeps 3 -> 4 -> 5 by tracking `posQ8`; on arrival it snaps to the confirmed
`floor`.

**What it is not for.** Deciding which floor the car is on. Ever.

`posQ8` is a reading off the drift-tracked datum, and that is precisely the
approach `ALGORITHM.md` §6 tried and rejected. Under the heading "Absolute
readout from a tracked datum", `floor = round((level - datum)/pitch)` scored
**73% correct at a 1 hPa/h pressure ramp**, against 98.3% for the jump-integrating
method that produces `floor`. The failure mode is not noise, it is structural:
the phase detector works modulo the pitch, so once the datum lags half a floor
the readout slips a whole floor and the residual goes quiet - the error stops
being visible to the loop that is supposed to correct it. The datum was measured
lagging **3.8 m**, more than a floor. No loop gain fixes a modulo error signal.

So the contract between the two fields is:

| | `floor` | `posQ8` |
|---|---|---|
| derived from | integrated jumps between confirmed stops | the tracked datum |
| measured accuracy | 98.3% at 1 hPa/h | 73% at 1 hPa/h |
| valid for | deciding the floor | animating between floors |
| updated | on arrival | every packet |
| authority | wins, always | corrected by `floor` on arrival |

A display that rounds `posQ8` and shows that number will be wrong in a way that
looks completely plausible for hours at a time. That is the trap.

---

## 5. The ESP-NOW frame

Bridge to displays, and display to display. One format, one relay rule, running
identically on every node - which is what removes the routing table. The only
asymmetry is that the bridge is `MESH_ROLE_ORIGIN` and may mint `origSeq`; the
displays are `MESH_ROLE_RELAY` and may not.

| off | size | type | field | value / meaning |
|---|---|---|---|---|
| 0 | 2 | u16 | `magic` | `MESH_MAGIC` = 0x5E1E, little-endian, so `1E 5E` on the wire |
| 2 | 1 | u8 | `version` | `MESH_VERSION` = 1. A node on a different build is rejected with `MESH_ERR_VERSION` |
| 3 | 1 | u8 | `hop` | transmissions remaining, including this one. Decremented in place at every relay |
| 4 | 2 | u16 | `origSeq` | monotonic, minted by the bridge. What dedup runs on - see section 6 |
| 6 | 1 | u8 | `type` | the LoRa tag, 0xE0 or 0xE1 |
| 7 | 1 | u8 | `len` | payload length, 0-32 (`MESH_MAX_PAYLOAD`) |
| 8 | `len` | | `payload` | the LoRa packet body from offset 3 onward (`ELEV_BODY_OFFSET`) |
| 8+`len` | 2 | u16 | `crc16` | CRC-16/CCITT-FALSE over bytes 0 .. 8+`len`-1 |

Sizes:

| | payload | frame |
|---|---|---|
| STATE (8-byte LoRa packet) | 5 | 15 bytes |
| STATS (24-byte LoRa packet) | 21 | 31 bytes |
| ceiling | 32 | 42 bytes (`MESH_MAX_FRAME_BYTES`) |

32 payload bytes leaves room for a LoRa format to grow without this envelope
becoming the reason it cannot, and 42 bytes total is far inside the 250 an
ESP-NOW frame carries. At `WIFI_PHY_RATE_LORA_250K` a 42-byte frame is about
1.3 ms on the air (`espnow_mesh.h`).

> `WIFI_PHY_RATE_LORA_250K` is Espressif's name for the 802.11 **LR** PHY. It has
> nothing to do with the SX1262's LoRa modulation. 250 kbps rather than 500 kbps
> because it is the more sensitive of the two LR rates, and at 42 bytes the
> airtime difference is under a millisecond.

### Wrapping

`meshWrapLora()` takes the LoRa packet exactly as it came off the SX1262 and:

- puts `lora[0]` (the tag) in `type`
- drops `lora[1]` (`txId`) and `lora[2]` (`seq`)
- copies `lora[3..]` into `payload`
- refuses anything with `loraLen <= 3` or a body over 32 bytes, returning 0

`txId` and `seq` are dropped because the mesh carries its own identity in
`origSeq` and only one transmitter exists. The consequence is worth stating
plainly: **a display cannot compute LoRa packet loss.** Only the bridge sees
`seq`. If you ever want per-floor link statistics on the screens, that is a
format change, not a firmware tweak.

### CRC coverage

`crc16Ccitt()` in `src/crc16.h` is **CRC-16/CCITT-FALSE**: polynomial 0x1021,
init 0xFFFF, no input or output reflection, no final XOR. It runs over the header
and the payload - bytes 0 through 8+`len`-1 - and the two result bytes are stored
little-endian at offset 8+`len`, outside their own coverage.

Why this leg carries a CRC when the LoRa leg does not:

- **The frame is rewritten at every hop.** `hop` is decremented in place, so a corrupted frame can be re-transmitted with a freshly computed 802.11 FCS by a node that never noticed the corruption.
- **The 802.11 FCS only ever covers one link.** This CRC travels with the payload end to end.
- It also catches a frame from an unrelated ESP-NOW device that happens to open with the right two bytes.

### The hop field

`hop` is the number of transmissions the frame has left, including the one
carrying it. `MESH_HOP_LIMIT` is 8. Five floors is the furthest any display sits
from the bridge on floor 5, so 8 leaves slack for a path that has to detour
around a dead node.

`meshDecrementHop()` does two things and forgetting either one breaks the mesh:

```c
frame[MESH_OFF_HOP]--;                                  // 1. decrement
meshPutU16(&frame[crcOffset], crc16Ccitt(frame, crcOffset));  // 2. rewrite the CRC
return frame[MESH_OFF_HOP] > 0;                         // relay only if any left
```

The CRC covers the header, and the header contains the field just changed. A
relay that decrements without rewriting sends a frame every downstream node
discards as corrupt - a mesh that works one hop from the bridge and nowhere else.

A frame arriving with `hop` already 0 is dropped rather than wrapping to 255.
A frame arriving with `hop == 1` is the last copy: it is decremented to 0 and
goes no further. So a frame minted at 8 is transmitted at most 8 times in total -
once by the bridge, then up to 7 relays.

### Flood rules

Identical on the bridge and all ten displays. These live in `espnow_mesh.cpp`;
the constants come from `[mesh_base]` in `platformio.ini`.

| rule | constant | value |
|---|---|---|
| channel | `MESH_CHANNEL` | 1 |
| TX power | `MESH_TX_POWER_QDBM` | 84 quarter-dBm, quantised by the PHY to 80 (20 dBm) |
| hop limit | `MESH_HOP_LIMIT` | 8 |
| back-to-back sends per frame | `MESH_REPEATS` | 3 |
| relay jitter | `MESH_RELAY_JITTER_MIN_MS` .. `MAX_MS` | random 5-40 ms |
| dedup ring depth | `MESH_DEDUP_RING` / `MESH_DEDUP_DEPTH` | 32 |
| PHY | `MESH_PHY_RATE` | `WIFI_PHY_RATE_LORA_250K` (802.11 LR) |

**The jitter is not a tuning refinement.** Ten displays hear the same frame
within microseconds of each other. If each relays as soon as it has decoded,
all ten transmit into the same air at the same instant - CSMA backoff is measured
against a channel that was idle a moment ago, so they do not back off from each
other - and every listener gets ten overlapping frames it can decode none of. The
relay then dies exactly where the mesh needed it most, one hop from the bridge. A
5-40 ms window spreads those ten transmits over far more than the ~1.3 ms a frame
occupies, so they queue instead of colliding.

Destination is always broadcast, `ff:ff:ff:ff:ff:ff`. There are no MAC addresses
configured anywhere in this system, so a display can be swapped for a spare with
no reflash of anything else.

Decode failures are counted, not reported. `MeshDecodeStatus` distinguishes
`MESH_ERR_SHORT`, `MESH_ERR_MAGIC`, `MESH_ERR_VERSION`, `MESH_ERR_LENGTH` and
`MESH_ERR_CRC`, and `MeshCounters` keeps a drop counter per reason plus
`dropHopExhausted`, `dropRxQueueFull`, `dropRelayFull` and `dropOversize`. A
display on floor 9 can be asked over Serial whether it is hearing the bridge
directly, hearing it through its neighbours, or hearing a frame per relay and
throwing most of them away.

### A real mesh frame, decoded

The STATE packet from section 2, wrapped by the bridge and freshly minted:

```
  offset:  0  1   2    3    4  5    6    7    8  9 10 11 12   13 14
         +-------+----+----+-------+----+----+---------------+-------+
         | 1E 5E | 01 | 08 | 11 10 | E0 | 05 | 03 9A 02 0B F2| A7 31 |
         +-------+----+----+-------+----+----+---------------+-------+
             |      |    |     |     |    |          |           |
             |      |    |     |     |    |          |           +-- crc16 = 0x31A7
             |      |    |     |     |    |          |               over bytes 0..12
             |      |    |     |     |    |          |
             |      |    |     |     |    |          +-- payload, 5 bytes:
             |      |    |     |     |    |              floor=3, posQ8=666,
             |      |    |     |     |    |              state=0x0B, conf=242
             |      |    |     |     |    |              (the LoRa packet from
             |      |    |     |     |    |               offset 3 onward)
             |      |    |     |     |    +-- len = 5
             |      |    |     |     +-- type = 0xE0, STATE
             |      |    |     +-- origSeq = 0x1011 = 4113
             |      |    +-- hop = 8, freshly minted
             |      +-- version = 1
             +-- magic = 0x5E1E, little-endian
```

The same frame after one display has relayed it:

```
         | 1E 5E | 01 | 07 | 11 10 | E0 | 05 | 03 9A 02 0B F2| 16 67 |
                        ^^                                     ^^^^^
                    hop 8 -> 7                         CRC recomputed:
                                                       0x31A7 -> 0x6716
```

One bit changed in the header and the CRC is entirely different. That is the
check: if you capture two copies of the same `origSeq` with different `hop`
values and identical CRCs, a relay is skipping the rewrite.

---

## 6. Sequence numbers

There are two, they are not the same thing, and confusing them is how a mesh
stops working.

| | LoRa `seq` | mesh `origSeq` |
|---|---|---|
| width | u8 | u16 |
| where | STATE and STATS, offset 2 | mesh envelope, offset 4 |
| minted by | the car | the bridge |
| wraps after | 256 packets | 65536 frames |
| at the 2 s STATE cadence | **512 s - 8 min 32 s** | ~36.4 h, about a day and a half |
| seen by displays | no, stripped at the bridge | yes |
| used for | loss statistics at the bridge | dedup, and nothing else |

### Why dedup cannot use the LoRa `seq`

Three independent reasons, any one of which is fatal:

1. **It wraps in under nine minutes.** A dedup ring 32 deep covers about a minute of traffic, so within any nine-minute window the same `seq` value legitimately appears twice. The ring would drop the second one as a duplicate. Every display would go blind for a random subset of packets, forever.
2. **STATE and STATS share the counter space.** Both formats carry `seq` at offset 2 from the same transmitter. Nothing in the format keeps them from colliding.
3. **The displays never see it.** The bridge strips `txId` and `seq` when it wraps the body. There is no `seq` in the mesh frame to dedup on.

So the bridge mints a fresh u16 per frame it floods, and that is the identity the
flood uses. `meshLastOrigSeq()` returns the last value minted (origin) or accepted
(relay); 0 means none yet, which is why minting starts at 1.

### Why dedup is set membership and never comparison

`origSeq` is u16 and *will* wrap - about every 36 hours at the STATE cadence. The
obvious rule, "relay only if `origSeq` is greater than the highest seen",
deadlocks the entire mesh for a full cycle at that moment, because every
subsequent frame looks old.

`MeshDedup` instead stores the last `MESH_DEDUP_DEPTH` = 32 values in a ring and
tests **equality** against every live entry. The wrap is not an event it can
observe. Nothing in `mesh_packet.h` compares two `origSeq` values with `<` or
`>`, and nothing added to it should.

The `count` field exists so a freshly initialised ring does not treat its 32
zeroed slots as having already seen `origSeq` 0 - which would silently drop the
first frame after every display reboot.

`meshDedupSeenOrInsert()` is the relay decision in one call: `false` means this
frame is new and has now been recorded, so the caller should jitter and
rebroadcast it exactly once.

32 entries is about a minute of STATE traffic at the 2 s cadence - long enough
that a frame arriving late by a few relay jitters is still recognised, short
enough to stay a linear scan over one cache line's worth of u16.

---

## 7. Extending the format without breaking deployed displays

Ten displays are screwed to walls in a stairwell. Assume you will not want to
reflash all of them.

**What is safe:**

| change | why it is safe |
|---|---|
| Setting a reserved bit in `state` (bits 5-7) or `flags` (bits 4-7) | old receivers mask for the bits they know and ignore the rest |
| Adding a new LoRa tag, e.g. `0xE2` | `elevPacketLength()` returns 0 for a tag it does not speak; `elevStateUnpack()`/`elevStatsUnpack()` return `ELEV_ERR_TAG`. An old display counts the drop and carries on. The mesh envelope passes any `type` through, so a new format floods to every floor before a single display understands it |
| Widening the mesh payload up to 32 bytes | `MESH_MAX_PAYLOAD` is already 32; the envelope does not need to change |

**What is not safe:**

| change | what breaks |
|---|---|
| Appending a field to STATE or STATS | `elevStateUnpack()` requires `len == ELEV_STATE_BYTES` exactly and returns `ELEV_ERR_LENGTH` otherwise. A 9-byte STATE is rejected by every deployed display, not partially decoded. **This is deliberate** - a length check that accepts extra bytes hides a truncation |
| Bumping `MESH_VERSION` | every node that has not been reflashed rejects every frame with `MESH_ERR_VERSION`. It is a hard cut, by design: a mixed-version flood with silently different field meanings is worse than a dead one |
| Reordering or resizing any existing field | there is no length or type information on the wire beyond `tag` and `len` |

**The airtime bill.** Before adding a byte, check section 1.1. Today:

- **STATE has exactly one free byte.** 8 or 9 bytes both cost 297 ms. The tenth byte costs 65.5 ms per packet, which at the 2 s cadence and 71.2% moving duty is **+3.3 mA on a ~19 mA budget** - roughly ten days off a two-month battery.
- **STATS has none.** 24 bytes is the top of its 48-symbol step. Byte 25 costs 65.5 ms, but only 0.15 mA, because it goes out once a minute instead of thirty times.

So: put a cheap field in STATS, not in STATE, unless it genuinely has to move at
2 s. And if you must grow STATE past 9 bytes, re-run the power budget in spec §2
rather than assuming the margin absorbs it.

**The recommended route** for anything new is a third tag. It costs nothing on
the deployed displays, it floods for free, and it lets you deploy the transmitter
side and the display side on separate weekends.

---

## 8. Where this differs from the design spec

The spec's section 4 was written before the headers. The field tables agree
exactly - every offset, size and unit in sections 2, 3 and 5 above matches
`elev_packet.h` and `mesh_packet.h` byte for byte. The differences are in what
the spec leaves out or names differently:

| | spec §4 | code |
|---|---|---|
| mesh payload ceiling | not stated | `MESH_MAX_PAYLOAD` 32, `MESH_MAX_FRAME_BYTES` 42 |
| mesh reject reasons | "magic, version, or CRC" | also `MESH_ERR_SHORT` and `MESH_ERR_LENGTH` |
| dedup depth constant | "32-deep ring" | `MESH_DEDUP_DEPTH` in `mesh_packet.h`, `MESH_DEDUP_RING` as the build flag; `espnow_mesh.cpp` static_asserts they agree |
| `txId` reaching displays | not mentioned | dropped at the bridge; displays never see it |
| `MESH_HOP_LIMIT` | "hop limit 8" | defined as 8 in both `platformio.ini` and `mesh_packet.h`, and the header's definition is not `#ifndef`-guarded - so it is not actually an overridable build knob |

Where the two disagree, the code wins.
