# Wire formats

Everything that goes on the air in this system, byte by byte, with the numbers
you need to decode a hex dump in a stairwell.

There are two links and they do not carry the same bytes:

```
  CAR (battery)                 BRIDGE, floor 5              DISPLAYS, 10 or 11
  XIAO + SX1262                 XIAO + SX1262                CYD, one per landing
        |                             |                            |
        |  LoRa 913/915/917 MHz       |  ESP-NOW, 802.11 LR        |
        |  SF10/BW125/CR4-8           |  channel 1, 6 or 11        |
        |  0xE0 STATE  8 B            |  0x5E1E envelope           |
        |  0xE1 STATS 24 B            |  15 B / 31 B on the wire   |
        +---------------------------->+--------------------------->+
                                                                   |
                                                            displays relay
                                                            to each other too
```

That is one shaft. Three of them are deployed - elevators A, B and C, 32
displays in total - and the only things that differ between them are the LoRa
frequency, the ESP-NOW channel, the `txId` on the wire and how many landings the
shaft has. Section 1.3 covers what happens when the three systems hear each
other. Everything else in this file describes one shaft and reads identically on
all three.

The bridge does not re-broadcast the LoRa bytes verbatim. It strips three bytes
(`tag`, `txId`, `seq`), puts the remaining body inside an ESP-NOW envelope with
its own sequence number and CRC, and floods that. So a display never sees `txId`
or the LoRa `seq` - those two exist only on the LoRa hop. Sections 5 and 6
explain why that is deliberate.

**Source of truth.** Every offset, mask and constant below is taken from
`src/elev_packet.h`, `src/mesh_packet.h` and `src/crc16.h`. Where the code and
the 2026-09-11 design spec's section 4 disagree, **the code is right** - that
spec is the older document. The disagreements found are small and are listed in
section 8; none of them touch a field offset. The three-elevator deployment spec
is a later, separate document that governs different things, and section 8 says
which.

---

## 1. The LoRa radio parameters

| parameter | value | build flag (`[lora_base]` unless noted) |
|---|---|---|
| frequency | per elevator, below | `LORA_FREQUENCY`, in `[elev_a]` / `[elev_b]` / `[elev_c]` |
| bandwidth | 125 kHz | `LORA_BANDWIDTH` |
| spreading factor | 10 | `LORA_SPREADING_FACTOR` |
| coding rate | 4/8 | `LORA_CODING_RATE` |
| sync word | 0x34 | `LORA_SYNC_WORD` |
| TX power | 22 dBm | `LORA_TX_POWER` |
| preamble | 8 symbols | `LORA_PREAMBLE_LEN` |
| header | explicit | RadioLib default |
| hardware CRC | on | RadioLib default |
| LDRO | off | `Ts` = 8.192 ms < 16 ms, so it is not required |

The frequency is the one row above that is not the same on every board. The
per-shaft plan, with the `txId` that goes with it (section 2):

| | elevator A | elevator B | elevator C |
|---|---|---|---|
| `LORA_FREQUENCY` | 913.0 MHz | 915.0 MHz | 917.0 MHz |
| `ELEV_TX_ID` | 0x41, `'A'` | 0x42, `'B'` | 0x43, `'C'` |

Section 1.3 derives the 2 MHz spacing. Everything else in the table above -
bandwidth, spreading factor, coding rate, sync word, TX power, preamble - is
identical on all three systems.

Both ends are the same module (Wio-SX1262 on a XIAO ESP32S3), and every
parameter except the frequency comes from the same `[lora_base]` section, so
there is no way for a car and a bridge to end up on different modulation
settings without editing one file. The frequency is the exception: it comes from
the shaft's own `[elev_*]` section, so a car flashed `elevator_tx_a` talking to a
bridge flashed `bridge_rx_b` sits 2 MHz away and is simply never heard. That
failure is total and silent - the bridge counts nothing at all, not even a CRC
error - which is why `docs/FLASHING.md` insists on labelling boards rather than
trusting memory.

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

Airtime is the power budget. The car averages ~23 mA and every millisecond on
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

What a step costs in current, at the 2026-09-11 spec §2.2 duty cycles and
140 mA:

| change | added current |
|---|---|
| STATE 8 -> 10 bytes, every 2 s at 90% STATE-active | **+4.2 mA** on a ~23 mA budget |
| STATS 24 -> 25 bytes, every 60 s | +0.15 mA |

Sanity check on the same arithmetic against the spec's own line items:
`297 ms / 2 s x 140 mA x 0.90 = 18.7 mA` and `494 ms / 60 s x 140 mA = 1.15 mA`,
which is the 18.7 mA and 1.2 mA in the 2026-09-11 spec §2.2. The 0.90 is the
STATE-active fraction, not the 71.2% moving fraction - STATE is not gated on
moving, it runs for `STATE_HOLD_AFTER_STOP_MS` = 10 s past every stop
(2026-09-11 spec §2.2, which states 0.90 in bold). Use 0.90 for anything costed
against this stream, here and in section 1.3.

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

### 1.3 Three systems in one building

Elevators A, B and C each run a complete copy of everything in this file. The
shafts are far apart, but far apart is not isolation: at 22 dBm a LoRa carrier
travels a long way through a building, and the design assumes all three
transmitters are audible everywhere rather than assuming they are not and
finding out during an evening peak.

On one frequency, three cars are pure ALOHA. Nothing in this link listens before
it transmits - there is no carrier sense anywhere in it. A STATE packet occupies
297 ms (section 1.1) and goes out every 2 s while the stream is active, so at the
0.90 STATE-active fraction of an evening peak, each car offers

```
  0.5 x 0.90 = 0.45 packets/s
```

0.90, not the 71.2% moving fraction from the reference capture: STATE is not
gated on moving, it keeps running for `STATE_HOLD_AFTER_STOP_MS` = 10 s past
every stop (2026-09-11 spec §2.2 states the 0.90 in bold). It is the same figure
the power budget in section 1.1 and the airtime bill in section 7 are costed at,
so all three now agree.

ALOHA's vulnerable window is two packet lengths, because a packet is destroyed
by an interferer starting any time from one packet length before it to one
packet length after it begins:

```
  window   = 2 x 0.297             = 0.594 s
  exponent = 2 x 0.45 x 0.594      = 0.5346
  P(survive two interferers) = exp(-0.5346) = 0.586
```

**About two STATE packets in five would be lost** - 41.4% - on top of the 5.21%
batch loss the shaft already costs (`ALGORITHM.md` §9), and concentrated in
exactly the evening peak when all three cars are busy at once. The displays'
180 s staleness timer absorbs the occasional hole; it is not sized for two
packets in five. (At the 0.712 moving fraction the same arithmetic gives
`exp(-0.422928) = 0.655`, a 34.5% loss - the smaller, wrong number, quoted here
only so nobody re-derives it and thinks it is the answer.)

Hence 913.0 / 915.0 / 917.0 MHz. 2 MHz against a 125 kHz occupied bandwidth is
roughly 16 channel widths, chosen against the near-far case - one shaft's
transmitter at arm's length from another shaft's bridge - rather than against the
typical case, where the shafts are far apart anyway. All three sit inside the
902-928 MHz ISM band with wide margin to both edges, and the modulation,
occupied bandwidth and duty cycle are unchanged from the single-elevator
deployment, so nothing about the regulatory picture changes.

**A per-elevator sync word is not the fix, and mistaking it for one is the most
likely wrong idea to have about this section.** `LORA_SYNC_WORD` stays 0x34 on
all three systems. The sync word is checked after preamble detection and
demodulation: by the time the modem can compare it, the frame has already been
received. It filters frames that arrived; it does nothing to two 22 dBm carriers
overlapping in the air, which destroy each other before any field of either
packet exists to be examined. Frequency separation is the only mechanism here
that prevents a collision rather than classifying the wreckage afterwards. The
same argument applies to the `tag` byte and to the `txId` check in section 2 -
all three are filters, and a filter runs strictly after the collision it cannot
prevent.

`txId` is still worth having, for a different reason: it turns "a foreign packet
reached this bridge" from an inference off a loss statistic into a counted event,
`dropForeignTxId` in the bridge's console summary. It moves for exactly one
cause: this bridge decoded, on its own frequency, a well-formed packet stamped
with another shaft's `txId`. It is not a general mis-flash counter - two
transmitters flashed for the same shaft would share both the frequency and the
`txId`, so their packets are accepted here and counted nowhere, and a missing
`LORA_FREQUENCY` is a compile error in `src/lora_link.h` rather than a board that
gets onto the wrong frequency. That makes it a bench instrument rather than a
field one - the bridge console is a wall-powered box mid-shaft that nobody is
standing at during normal operation - and with 2 MHz of separation it should read
0 forever. A non-zero value is the unambiguous evidence of cross-shaft leakage
that a loss statistic can only hint at.

The ESP-NOW side is separated the same way, channels 1, 6 and 11, one per shaft,
but for an entirely different class of reason: `origSeq` collisions between three
minting bridges, and a relay jitter window oversubscribed by 32 relaying
displays. `MESH.md` carries that argument. The envelope itself does not change;
section 5 says why.

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
| 1 | 1 | u8 | `txId` | `'A'`/`'B'`/`'C'` | which shaft this came from, as ASCII: 0x41, 0x42 or 0x43 (`ELEV_TX_ID`). The bridge drops any packet whose `txId` is not its own elevator, and strips the field before the mesh |
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

Elevator B, car ascending between floor 3 and floor 4, model good, clean lattice
fit:

```
  offset:   0    1    2    3    4    5    6    7
          +----+----+----+----+---------+----+----+
          | E0 | 42 | 2A | 03 |  9A 02  | 0B | F2 |
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
            |    +-- txId = 0x42 = 'B', so this is elevator B. A bridge
            |         built for A or C is 2 MHz away and normally never
            |         decodes this at all; if one does, it drops the
            |         packet here and counts it in dropForeignTxId.
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
| 1 | 1 | u8 | `txId` | | the shaft, ASCII `'A'`/`'B'`/`'C'` - same field, same bridge check and same stripping as STATE |
| 2 | 1 | u8 | `seq` | | wraps; loss statistics only |
| 3 | 2 | u16 | `batteryMv` | mV | pack voltage. A 4S LiFePO4 spans 11200-14600 mV. Divider is +/-2-3% uncalibrated |
| 5 | 2 | u16 | `dist24hM` | m | distance over the last 24 hourly buckets. Uptime-relative, not wall clock |
| 7 | 4 | u32 | `distTotalM` | m | lifetime odometer, survives reboot via NVS |
| 11 | 2 | u16 | `pitchMm` | mm | learned floor pitch. 2871 on the reference capture; the acceptance gate is +/-1 mm, which is why the unit is mm |
| 13 | 1 | u8 | `nFloors` | | learned floor count - the *span* of learned indices, `maxIndex() - minIndex() + 1`, not a tally of landings visited. 10 in a 1-10 shaft (elevator B), 11 where there is a basement (A and C). The display compares it against its own `FLOOR_LABEL_COUNT`; `docs/FLASHING.md` covers what it shows when the two disagree |
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
  bit   7     6     5      4        3        2        1       0
      +-----+-----+-----+--------+--------+--------+-------+--------+
      |  -  |  -  |  -  | crit   | low    | nvs    | model | sensor |
      |     |     |     | Battery| Battery|Restored| Ready |  Err   |
      +-----+-----+-----+--------+--------+--------+-------+--------+
        reserved, sent as 0
```

| mask | name | meaning |
|---|---|---|
| 0x01 | `ELEV_FLAG_SENSOR_ERR` | the BMP390 is not answering |
| 0x02 | `ELEV_FLAG_MODEL_READY` | pitch established |
| 0x04 | `ELEV_FLAG_NVS_RESTORED` | the model came back from NVS on boot rather than being learned this run. Lets a display tell "restored and trusted" from "learned here" |
| 0x08 | `ELEV_FLAG_LOW_BATTERY` | latched below 12.8 V since the last charge, ~20% remaining: screens show CHARGE SOON |
| 0x10 | `ELEV_FLAG_CRITICAL_BATTERY` | latched below 12.0 V since the last charge, ~10% remaining: screens show the CHARGE BATTERY banner. Always sent with 0x08 set too, so a receiver that only checks LOW still warns |

`lowBattery` is sent as a bit *as well as* a voltage on purpose. The threshold
lives on the transmitter, which is the only node that knows its own `VBAT_CAL_*`
correction. A display comparing `batteryMv` against a hardcoded number would be
comparing against an uncalibrated figure.

Accessors: `elevStatsSensorErr()`, `elevStatsModelReady()`,
`elevStatsNvsRestored()`, `elevStatsLowBattery()`, and the assembler
`elevStatsFlagsByte()`.

### A real STATS packet, decoded

Elevator B: parked, healthy, a day into a run, model restored from NVS on the
last reboot.

```
  offset:  0    1    2     3  4     5  6     7  8  9 10    11 12   13
         +----+----+----+-------+-------+-------------+-------+----+
         | E1 | 42 | 2B | 2C 33 | 51 0D | B5 F5 01 00 | 37 0B | 0A |
         +----+----+----+-------+-------+-------------+-------+----+

  offset: 14 15   16 17   18 19 20 21    22   23
         +-------+-------+-------------+----+----+
         | 65 01 | FF 00 | 58 6E 01 00 | 06 | 16 |
         +-------+-------+-------------+----+----+

  E1           tag           STATS
  42           txId          0x42 = 'B' - elevator B
  2B           seq           43
  2C 33        batteryMv     0x332C = 13100 mV = 13.10 V
                             13.3 V is rested-full and 12.8 V is where
                             CHARGE SOON latches, so this pack is healthy
  51 0D        dist24hM      0x0D51 = 3409 m in the last 24 buckets
  B5 F5 01 00  distTotalM    0x0001F5B5 = 128437 m lifetime
  37 0B        pitchMm       0x0B37 = 2871 mm = 2.871 m  <- the learned pitch
  0A           nFloors       10 landings. Elevator B has no basement;
                             the same field off A or C reads 0B
  65 01        trips         0x0165 = 357 floor-to-floor moves
  FF 00        stops         0x00FF = 255 confirmed stops
  58 6E 01 00  uptimeS       0x00016E58 = 93784 s = 26.1 h
  06           flags         0000 0110
                             bit0 sensorErr   = 0
                             bit1 modelReady  = 1
                             bit2 nvsRestored = 1   <- learned before this boot
                             bit3 lowBattery  = 0
                             bit4 critBattery = 0
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

### No elevator field, and why

**The three-elevator deployment does not change one byte of this envelope.**
`version` is still 1, `magic` is still 0x5E1E, and there is nothing here naming
the shaft.

An elevator id would never discriminate anything at runtime. The three floods
run on separate WiFi channels, so a display physically cannot receive another
shaft's frames; the check would be true by construction on every frame it ever
ran on. It would cost a `MESH_VERSION` bump, which section 7 explains is a hard
cut that strands every node not reflashed the same day.

The failure it looks like it would guard is a display flashed for the wrong
shaft, and no wire field could catch that one, because no frame ever reaches
that display to carry the field. `MESH_CHANNEL` is per shaft (6/1/11, the flood
table below) and 1, 6 and 11 are non-overlapping, so a display flashed for the
wrong shaft cannot hear its landing's bridge at all. It sits on the
`displaySplash("Simmevator", "waiting for the mesh")` screen forever: it does not
show a wrong floor, and it does not show `--`, which is the "frames heard, floor
not confirmed" screen. The check that catches it is a human walking the stairwell
during commissioning, which is what `docs/FLASHING.md` describes.

The `nFloors`-against-`FLOOR_LABEL_COUNT` comparison in section 3 is a different
check for a different fault. Every display in a shaft carries the same label
table and hears the same bridge, so when it trips it trips on all ten or eleven
screens at once, and what it means is that the car's learned `nFloors` disagrees
with the shaft's label count - in practice a car that has never visited its
lowest landing.

### Wrapping

`meshWrapLora()` takes the LoRa packet exactly as it came off the SX1262 and:

- puts `lora[0]` (the tag) in `type`
- drops `lora[1]` (`txId`) and `lora[2]` (`seq`)
- copies `lora[3..]` into `payload`
- refuses anything with `loraLen <= 3` or a body over 32 bytes, returning 0

`txId` and `seq` are dropped because neither means anything downstream of the
bridge. `txId` has already done its work by then: the bridge compared it against
its own `ELEV_TX_ID` and dropped the packet if it did not match, so everything
that reaches the wrapping step is this shaft's car by construction. `seq` is a
LoRa-hop loss counter, and the mesh carries its own identity in `origSeq`
instead. The consequence is worth stating
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

Identical on the bridge and on every display in that shaft - ten of them, or
eleven where there is a basement. These live in `espnow_mesh.cpp`;
the constants come from `[mesh_base]` in `platformio.ini`.

| rule | constant | value |
|---|---|---|
| channel | `MESH_CHANNEL` | per elevator: 1 (B), 6 (A), 11 (C) - `MESH.md` |
| TX power | `MESH_TX_POWER_QDBM` | 84 quarter-dBm, quantised by the PHY to 80 (20 dBm) |
| hop limit | `MESH_HOP_LIMIT` | 8 |
| back-to-back sends per frame | `MESH_REPEATS` | 3 |
| relay jitter | `MESH_RELAY_JITTER_MIN_MS` .. `MAX_MS` | random 5-40 ms |
| dedup ring depth | `MESH_DEDUP_RING` / `MESH_DEDUP_DEPTH` | 32 |
| PHY | `MESH_PHY_RATE` | `WIFI_PHY_RATE_LORA_250K` (802.11 LR) |

**The jitter is not a tuning refinement.** The ten or eleven displays in one
shaft hear the same frame within microseconds of each other. If each relays as
soon as it has decoded, they all transmit into the same air at the same instant -
CSMA backoff is measured against a channel that was idle a moment ago, so they do
not back off from each other - and every listener gets ten overlapping frames it
can decode none of. The relay then dies exactly where the mesh needed it most,
one hop from the bridge. A 5-40 ms window spreads those transmits over far more
than the ~1.3 ms a frame occupies, so they queue instead of colliding.

This window is also why the three shafts are not merged into one building-wide
flood: 32 displays and 3 bridges relaying a single frame would put roughly 96
transmissions into the same 35 ms, which re-creates precisely the collision the
jitter was introduced to prevent. `MESH.md` has that arithmetic.

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

Thirty-two displays are screwed to walls across three stairwells, and the three
transmitters are inside cars that have to be taken out of service to open.
Assume you will not want to reflash all of them.

**What is safe:**

| change | why it is safe |
|---|---|
| Setting a reserved bit in `state` (bits 5-7) or `flags` (bits 5-7) | old receivers mask for the bits they know and ignore the rest |
| Adding a new LoRa tag, e.g. `0xE2` | `elevPacketLength()` returns 0 for a tag it does not speak; `elevStateUnpack()`/`elevStatsUnpack()` return `ELEV_ERR_TAG`. An old display counts the drop and carries on. The mesh envelope passes any `type` through, so a new format floods to every floor before a single display understands it |
| Widening the mesh payload up to 32 bytes | `MESH_MAX_PAYLOAD` is already 32; the envelope does not need to change |

**What is not safe:**

| change | what breaks |
|---|---|
| Appending a field to STATE or STATS | `elevStateUnpack()` requires `len == ELEV_STATE_BYTES` exactly and returns `ELEV_ERR_LENGTH` otherwise. A 9-byte STATE is rejected by every deployed display, not partially decoded. **This is deliberate** - a length check that accepts extra bytes hides a truncation |
| Bumping `MESH_VERSION` | every node that has not been reflashed rejects every frame with `MESH_ERR_VERSION`. It is a hard cut, by design: a mixed-version flood with silently different field meanings is worse than a dead one |
| Reordering or resizing any existing field | there is no length or type information on the wire beyond `tag` and `len` |

**The airtime bill.** Before adding a byte, check section 1.1. Today:

- **STATE has exactly one free byte.** 8 or 9 bytes both cost 297 ms. The tenth byte costs 65.5 ms per packet, which at the 2 s cadence and the 90% STATE-active fraction is **+4.2 mA on a ~23 mA budget** - roughly nine days off an eight-week battery.
- **STATS has none.** 24 bytes is the top of its 48-symbol step. Byte 25 costs 65.5 ms, but only 0.15 mA, because it goes out once a minute instead of thirty times.

So: put a cheap field in STATS, not in STATE, unless it genuinely has to move at
2 s. And if you must grow STATE past 9 bytes, re-run the power budget in the
2026-09-11 spec §2 rather than assuming the margin absorbs it.

**The recommended route** for anything new is a third tag. It costs nothing on
the deployed displays, it floods for free, and it lets you deploy the transmitter
side and the display side on separate weekends.

---

## 8. Where this differs from the design specs

There are two specs now, and they govern different things:

| document | governs |
|---|---|
| `docs/superpowers/specs/2026-09-11-simmevator-design.md` | the original single-elevator design. Its §4 is the first draft of the two wire formats, and its §2.2 is the power budget quoted in section 1.1 |
| `docs/superpowers/specs/2026-09-18-three-elevator-deployment-design.md` | the three-elevator deployment: the frequency plan in §2, the `txId` assignment and bridge filter in §3, and the per-shaft build flags in §5.1 |

The second spec changes no byte of either format. It is a set of per-elevator
build flags plus one display state, so it is authoritative for the values in
section 1.3 and for the meaning of `txId`, and it touches nothing else in this
file. The field tables remain governed by the headers.

The 2026-09-11 spec's section 4 was written before the headers. The field tables
agree exactly - every offset, size and unit in sections 2, 3 and 5 above matches
`elev_packet.h` and `mesh_packet.h` byte for byte. The differences are in what
the spec leaves out or names differently:

| | 2026-09-11 spec §4 | code |
|---|---|---|
| mesh payload ceiling | not stated | `MESH_MAX_PAYLOAD` 32, `MESH_MAX_FRAME_BYTES` 42 |
| mesh reject reasons | "magic, version, or CRC" | also `MESH_ERR_SHORT` and `MESH_ERR_LENGTH` |
| dedup depth constant | "32-deep ring" | `MESH_DEDUP_DEPTH` in `mesh_packet.h`, `MESH_DEDUP_RING` as the build flag; `espnow_mesh.cpp` static_asserts they agree |
| `txId` reaching displays | not mentioned | dropped at the bridge; displays never see it |
| `txId` values | "transmitter identity", unassigned | ASCII `'A'`/`'B'`/`'C'`, assigned by the 2026-09-18 spec §3 and checked against `ELEV_TX_ID` |
| `MESH_HOP_LIMIT` | "hop limit 8" | defined as 8 in both `platformio.ini` and `mesh_packet.h`, and the header's definition is not `#ifndef`-guarded - so it is not actually an overridable build knob |

Where the two disagree, the code wins.
