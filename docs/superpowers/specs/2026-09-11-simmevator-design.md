# Simmevator — elevator floor tracker and per-floor displays

Design spec. Simmons Hall, MIT.

A barometer in the elevator car decides which floor the car is on and broadcasts
it over LoRa to a bridge on floor 5, which floods it over ESP-NOW to a display on
every floor. The car node runs on a battery for at least a month.

The floor-detection algorithm is not designed here. It exists, it is measured, and
it is specified in `../../../../elevatormons/docs/ALGORITHM.md` with a reference
implementation in `elevatormons/tools/floor_algorithm.py`. This spec covers
porting it faithfully to C++ and building a system around it. **The algorithm's
tuning constants are not to be re-tuned.**

---

## 1. Why the system is shaped this way

Three constraints drive every decision below.

**The floor decision must happen in the car.** ALGORITHM.md §9: the node holds the
sensor, so it sees an unbroken 1 Hz stream. Radio loss can only age what a display
shows — it can never corrupt the floor, and one received packet resyncs
everything. Moving the decision to the bridge would make every dropout a
correctness problem instead of a latency problem.

**Absolute altitude cannot decide the floor.** Over the 3 h reference capture the
sea-level reference wandered 9.9 m — more than three floor heights. Floors are
read from *jumps between stops*, which are short enough that weather cannot move
them. This is why there is a learned building model and why it is worth
persisting.

**Airtime is the power budget.** At SF10 a LoRa packet costs hundreds of
milliseconds at ~118 mA. The transmit schedule, not the sleep current, is what
decides whether the battery lasts a month.

---

## 2. Power budget

Battery: LiFePO4 4S, 12.8 V nominal, 10 Ah = 128 Wh. Through a buck regulator at
~85% into 3.3 V that is ≈31 Ah at 3.3 V. For 30 days (720 h) the average draw
must stay under **43 mA**.

### 2.1 Radio configuration

ALGORITHM.md §9 records the field run as `SF10 / BW125 / 14 dBm` → 5.21% batch
loss over 3 h. `elevatormons/platformio.ini` sets `baro_base` to BW62.5, which has
no in-shaft evidence. **This project uses BW125** — the configuration that was
actually measured — at 22 dBm, which is 8 dB more margin than the run that
produced that 5.21% figure.

| config | 8-byte airtime | sensitivity | in-shaft evidence |
|---|---|---|---|
| SF10/BW62.5/CR4-8 | 725 ms | −135 dBm | none |
| **SF10/BW125/CR4-8** | **297 ms** | −132 dBm | 5.21% loss at 14 dBm |

§9 also records that loss barely tracks SNR (r = −0.13) — it is shadowing and
interference, not a decode floor — so buying margin with spreading factor buys
little while costing airtime linearly.

Airtime derivation (SF10, BW125 kHz, CR 4/8, explicit header, LDRO off since
Ts = 8.192 ms < 16 ms):

```
Ts       = 2^10 / 125000          = 8.192 ms
preamble = (8 + 4.25) * Ts        = 100.4 ms
nPayload = 8 + ceil((8*PL - 4*SF + 28 + 16) / (4*SF)) * 8
PL =  8 -> 8 + ceil(68/40)*8  = 24 sym -> 196.6 ms -> total  297 ms
PL = 24 -> 8 + ceil(196/40)*8 = 48 sym -> 393.2 ms -> total  494 ms
```

### 2.2 Transmit schedule

Measured from the reference capture, the car is "moving" (by the algorithm's own
stillness test) **71.2% of an evening peak**. Over a full day including nights it
is far lower, but the budget below uses the peak figure.

| mode | trigger | cadence |
|---|---|---|
| STATE | car moving, or within 10 s of a stop | every 2 s, first packet fires immediately on motion onset |
| STATS | always | every 60 s |

| line item | current |
|---|---|
| XIAO light-sleeping between samples | ~3 mA |
| STATE: 297 ms / 2 s × 140 mA × 0.712 | ~15 mA |
| STATS: 494 ms / 60 s × 140 mA | ~1.2 mA |
| divider + BMP390 | ~0.1 mA |
| **total at evening-peak traffic** | **~19 mA → ~68 days** |

140 mA is the SX1262 at +22 dBm (~118 mA) plus the ESP32-S3 awake. Deep sleep is
impossible: the algorithm needs an unbroken 1 Hz stream. Light sleep between
samples is what makes the baseline 3 mA instead of ~25 mA.

If measured life falls short, the knobs in order of preference are: STATE cadence
2 s → 3 s (−5 mA), TX power 22 → 20 dBm (−2 mA), parked heartbeat 60 s → 120 s
(−0.6 mA).

---

## 3. Hardware

### 3.1 Transmitter — in the car, on battery

Seeed XIAO ESP32S3 + Wio-SX1262 (a stack, no wiring) + BMP390 on I²C bus 1.

| signal | pin | note |
|---|---|---|
| BMP390 SDA / SCL | D4 = GPIO5 / D5 = GPIO6 | `Wire1`, 400 kHz, addr 0x77 |
| SX1262 SCK/MISO/MOSI | D8/D9/D10 = GPIO7/8/9 | via the expansion stack |
| SX1262 NSS/DIO1/BUSY/NRST | GPIO41/39/40/42 | |
| **battery sense** | **D0 = GPIO1 (ADC1_CH0)** | see below |

D0 is the only free ADC1 pad — D4/D5 are I²C, D8/D9/D10 are SPI, and GPIO39–42
belong to the radio.

### 3.2 Battery sense divider

```
  BATT+ ──[ R1 = 1 MΩ 1% ]──┬── D0 (GPIO1)
                            │
                       [ R2 = 200 kΩ 1% ]   ‖  100 nF to GND
                            │
  BATT− ──────────────────── ┴── GND (common with XIAO GND)
```

Ratio 6.0. 14.6 V → 2.433 V; 10.0 V → 1.667 V. The whole span sits in the
ESP32-S3 ADC's linear region at 12 dB attenuation, which misbehaves below ~0.15 V
and above ~2.8 V. Divider draw is 12 µA — 0.06% of the budget, so no switching
MOSFET.

Read with `analogReadMilliVolts()` (applies the factory eFuse calibration),
averaged over 32 samples, once per STATS packet. A one-point correction
`-DVBAT_CAL_NUM` / `-DVBAT_CAL_DEN` is set once against a DMM.

LiFePO4 4S thresholds for the display: 13.3 V rested-full, 12.8 V nominal,
**12.0 V warn (~20% remaining)**, 11.2 V critical.

### 3.3 Power chain

`LiFePO4 4S 10 Ah → 1 A fuse → MP1584EN buck set to 3.3 V → XIAO 3V3 pad`.

Feeding the 3V3 pad bypasses the XIAO's own LDO, which is the efficient path.
**Do not connect USB and the buck at the same time** — the LDO output would be
contested. Unplug the battery before flashing.

The MP1584 is PWM-only with no light-load PFM mode, so it is poor at the 3 mA
sleep current but runs ~80–85% at the ~19 mA average, which is what §2 assumes. A
TPS62203 or MP2338 would recover a few days; not required.

**The enclosure must be vented.** A sealed box turns the barometer into a
thermometer and the system stops working.

### 3.4 Bridge — floor 5, wall powered

XIAO ESP32S3 + Wio-SX1262, USB-C 5 V. LoRa receive continuous. ALGORITHM.md §9
flags floor 9 as the one landing whose SNR crosses the decode floor — an
intermittent shadow — so bridge antenna placement should favour the upper shaft.

### 3.5 Displays — one per floor

ELEGOO ESP32 CYD 2.8", ESP32-WROOM-32, ILI9341 240×320, USB-C. Board
`esp32dev`. Pin map (the standard ESP32-2432S028R):

| function | pins |
|---|---|
| TFT SPI | MOSI 13, MISO 12, SCLK 14, CS 15, DC 2, RST −1 |
| backlight | 21 (active high, PWM) |
| touch XPT2046 | CLK 25, MOSI 32, MISO 39, CS 33, IRQ 36 |
| LDR | 34 (analog) |
| RGB LED | 4 / 16 / 17 (active low) |

TFT_eSPI is configured entirely through `build_flags` in `platformio.ini` so
nothing outside this repo is edited.

### 3.6 Parts list

**Transmitter (×1 assembly)**

| Part | Qty | Notes |
|---|---|---|
| Seeed XIAO ESP32S3 | 1 | |
| Seeed Wio-SX1262 for XIAO | 1 | stacks onto the XIAO |
| 915 MHz antenna | 1 | usually ships with the Wio-SX1262 |
| BMP390 breakout | 1 | Adafruit 4816 or equivalent |
| LiFePO4 12.8 V 10 Ah pack with BMS | 1 | 4S; BMS required |
| LiFePO4 charger, 14.6 V | 1 | a Li-ion or lead-acid charger will not terminate correctly |
| MP1584EN buck module | 1 | set to 3.3 V **before** connecting the XIAO |
| Resistor 1 MΩ 1% | 1 | divider top |
| Resistor 200 kΩ 1% | 1 | divider bottom |
| Ceramic cap 100 nF | 1 | across R2 |
| Electrolytic 220 µF / 25 V | 1 | buck input bulk |
| Inline fuse 1 A + holder | 1 | on BATT+ |
| XT60 or 5.5 mm barrel pair | 1 | so the pack can be unplugged to charge |
| Vented project enclosure | 1 | must not be airtight |
| Hookup wire / JST leads | — | |

**Bridge (×1)**

| Part | Qty |
|---|---|
| Seeed XIAO ESP32S3 | 1 |
| Seeed Wio-SX1262 for XIAO | 1 |
| 915 MHz antenna | 1 |
| USB-C 5 V adapter + cable | 1 |

**Displays (×10)**

| Part | Qty | Notes |
|---|---|---|
| ELEGOO ESP32 CYD 2.8" ILI9341 240×320 | 10 | sold in 2-packs → 5 packs |
| USB-C 5 V / 1 A adapter | 10 | |
| USB-C cable | 10 | |
| Wall mount or stand | 10 | 3D-printable; STLs out of scope |

**Totals:** 2 × XIAO ESP32S3 · 2 × Wio-SX1262 · 1 × BMP390 · 1 × battery +
charger · 1 × buck · 10 × CYD · 11 × USB-C supplies.

---

## 4. Wire protocols

### 4.1 LoRa: car → bridge

Tag-byte discrimination, as `elevatormons/src/baro_packet.h` does. No application
CRC: the SX1262's hardware CRC-16 is enabled and rejects corrupt frames before
RadioLib hands them over. All multi-byte fields little-endian.

**`0xE0` STATE — 8 bytes, 297 ms**

| off | size | field | meaning |
|---|---|---|---|
| 0 | u8 | `tag` | 0xE0 |
| 1 | u8 | `txId` | transmitter identity |
| 2 | u8 | `seq` | wraps; for loss statistics only |
| 3 | u8 | `floor` | confirmed 1-based index; 0 = model not ready |
| 4 | i16 | `posQ8` | live fractional position in 1/256 floor units, relative to floor 1 |
| 6 | u8 | `state` | bit0 moving · bits1-2 direction (0 idle, 1 up, 2 down) · bit3 modelReady · bit4 sensorErr |
| 7 | u8 | `confidence` | 0–255, from the lattice fit in `_arrive()` |

`posQ8` is what lets a display animate the car climbing through floors. It is
derived from the drift-tracked datum, which ALGORITHM.md §6 shows is unreliable
for *deciding* a floor — so it is used only for animation and is always corrected
by `floor` on arrival.

**`0xE1` STATS — 24 bytes, 494 ms**

| off | size | field | units |
|---|---|---|---|
| 0 | u8 | `tag` | 0xE1 |
| 1 | u8 | `txId` | |
| 2 | u8 | `seq` | |
| 3 | u16 | `batteryMv` | mV |
| 5 | u16 | `dist24hM` | metres in the last 24 h |
| 7 | u32 | `distTotalM` | metres, lifetime |
| 11 | u16 | `pitchMm` | learned floor pitch |
| 13 | u8 | `nFloors` | learned floor count |
| 14 | u16 | `trips` | |
| 16 | u16 | `stops` | |
| 18 | u32 | `uptimeS` | |
| 22 | u8 | `flags` | bit0 sensorErr · bit1 modelReady · bit2 nvsRestored · bit3 lowBattery |
| 23 | i8 | `tempC` | |

### 4.2 ESP-NOW: bridge → displays → displays

```
off  size  field
0    u16   magic     0x5E1E
2    u8    version   1
3    u8    hop       remaining hops; dropped at 0
4    u16   origSeq   monotonic, minted by the bridge
6    u8    type      0xE0 or 0xE1
7    u8    len       payload length
8..  ...   payload   the LoRa packet body from offset 3 onward
+2   u16   crc16     CCITT over everything before it
```

`origSeq` is minted by the bridge and is u16 monotonic — the LoRa `seq` is u8 and
wraps far too fast to dedup on.

**Flood rules**, identical on the bridge and all ten displays:

- Keep a 32-deep ring of seen `origSeq`. Rebroadcast each new value exactly once.
- Hop limit 8. Five floors is the furthest any display is from the bridge, so 8
  leaves slack for a detour around a dead node.
- **Wait a random 5–40 ms before relaying.** Ten displays hearing the same frame
  and relaying at once is a guaranteed collision; the jitter is the difference
  between a mesh that works and one that does not.
- Send each frame 3× back-to-back.
- `WIFI_PROTOCOL_LR` — Espressif's long-range PHY, ~7 dB more sensitivity than
  802.11b. Every node here is an ESP32, so there is no compatibility cost, and
  floor-to-floor through concrete is exactly the marginal link it is for.
- Channel 1, maximum TX power (84 quarter-dBm, quantised to 20 dBm).
- A frame failing magic, version, or CRC is dropped silently and counted.

---

## 5. The algorithm port

`src/floor_monitor.h` — board-agnostic, no `Arduino.h`, no allocation, so the
`native` environment compiles the exact code the ESP32 runs.

Four places a port silently diverges from the Python reference:

1. **`double`, not `float`.** `_trusted_rate()` computes
   `med²/(med² + (k·mad)²)` on numbers around 1e-4. float32 does not survive that.
2. **`np.std` is population std (ddof = 0).** The sample std is 12% larger over a
   5-window, which shifts the stillness threshold.
3. **`heights` / `stop_counts` dicts → fixed arrays.** `int16` floor index with
   origin +32 in a 64-entry table plus an occupied bitmap. Out-of-range indices
   clamp and raise a flag rather than growing.
4. **`estimate_pitch`'s sweep** (2.2 → 4.0 m, step 5e-4) runs once over 12 jumps:
   ~43k double operations, microseconds on this chip. Port it literally.

Every constant in ALGORITHM.md §7 is carried over unchanged. Structure mirrors
`floor_algorithm.py` function for function — `update`, `_close_dwell`, `_arrive`,
`_confirm`, `_revisit_rate`, `_trusted_rate`, `_replay`, `_learn`,
`_refine_pitch`, `estimate_pitch` — so the two can be diffed by eye.

### 5.1 Persistence

Pitch, the learned height ladder, stop counts, current floor index, `ref`,
odometer, trips, stops, and the 24 h buckets are written to NVS **every 5 minutes
when something has changed**. That is 288 writes/day. Saving on every confirmed
stop would be ~2000/day, which is a flash-endurance problem: the reference capture
has 256 confirmed stops in 3 h.

On boot the model is restored and `modelReady` is set immediately, so a reboot or
battery swap does not restart the 5-minute bootstrap. A genuinely fresh unit still
withholds output until the pitch is established, per ALGORITHM.md §8.

A stored record carries a schema version and a CRC; a mismatch is discarded and
treated as a cold boot.

### 5.2 The 24-hour odometer

24 rolling one-hour buckets in RAM, mirrored to NVS. There is no RTC — buckets are
indexed by `uptime / 3600 mod 24`, and advancing into a bucket clears it. "Last
24 h" therefore means "the last 24 hourly buckets", which is what the display
claims.

---

## 6. Displays

320×240 landscape.

```
┌────────────────────────────┬──────────────┐
│                            │      ▲       │   ▲/▼ only while moving
│      ██████     ██         │    UP        │
│      ██  ██     ██         │              │
│      ██  ██     ██         │  12.7 V      │   colour-coded bar
│      ██  ██     ██         │  ■■■■■□      │
│      ██████     ██         │              │
│                            │  2.1 mi      │   last 24 h
│                            │  today       │
└────────────────────────────┴──────────────┘
```

- **Big digits are drawn as vector 7-segment shapes**, not a bundled font. Any
  font large enough looks soft at ~180 px, and vectors cost nothing.
- While moving, the number tracks `posQ8` so it sweeps 3 → 4 → 5; on arrival it
  snaps to the confirmed `floor`.
- Redraws are region-limited via sprites, so the panel does not flicker.
- Floor labels come from a compile-time table, `-DFLOOR_LABELS="1,2,...,10"`.
  Simmons serves floors 1–10 with no basement, so index *n* maps to label *n*,
  but the table is where a basement or a skipped floor would be handled.
- **Staleness**: no STATE for 10 s dims the readout; no STATS for 180 s greys it
  and turns the RGB LED amber, so a dead link is visible from the corridor.
- The LDR on GPIO34 drives backlight PWM on GPIO21 for night dimming.
- Distance is extrapolated locally between heartbeats from observed floor changes
  × the transmitted pitch, then corrected to the transmitter's figure on each
  STATS packet — so it ticks in real time but all ten screens agree every minute.

---

## 7. Verification

### 7.1 Host unit tests — `pio test -e native`

- `test/test_elev_packet/` — round-trip every field of both formats at limits and
  boundaries; truncated, wrong-tag, wrong-length, and corrupt-CRC inputs.
- `test/test_mesh_packet/` — wrap/unwrap, hop decrement, CRC rejection, dedup ring
  behaviour including `origSeq` wraparound.
- `test/test_floor_monitor/` — synthetic lattice ascents/descents, door cycles
  below `MIN_JUMP_M`, off-lattice plateau rejection, bootstrap replay, and the
  shrinkage behaviour of `_trusted_rate()` under clustered vs scattered input.

### 7.2 Replay against the real capture

`sim/replay_main.cpp` builds in the `native` environment, streams
`elevatormons/data/baro-20260910-195011.csv` through the **actual C++
`FloorMonitor`**, and emits a CSV of broadcasts.
`sim/compare_to_python.py` diffs that against `floor_algorithm.py` sample by
sample.

The capture has **49 dropouts totalling 570 missing samples, worst case 41 s** —
radio loss, which the real node never sees because it holds the sensor. Gaps are
filled by linear interpolation of *pressure* onto an even 1 Hz grid, and filled
samples are marked in the output. The 41 s gap becomes an invented smooth ride;
that is documented, not hidden.

**Acceptance gates** — from ALGORITHM.md §5 and confirmed by re-running the
reference implementation:

| quantity | required |
|---|---|
| floor sequence | identical to the Python reference, sample for sample |
| floors served | 10 |
| learned pitch | 2.871 m ± 1 mm |
| distance travelled | 3409 m ± 0.1% |
| floor-to-floor moves | 356 |
| confirmed stops | 256 |

Anything less means the port is wrong, not that the gates are too strict.

---

## 8. Repository layout

```
platformio.ini        envs: elevator_tx · bridge_rx · floor_display · native
src/
  crc16.h  elev_packet.h  mesh_packet.h  floor_monitor.h     board-agnostic
  bmp390_sensor.{h,cpp}  lora_link.{h,cpp}                   adapted from elevatormons
  espnow_mesh.{h,cpp}  battery.{h,cpp}  nvs_model.{h,cpp}
  display_ui.{h,cpp}
  elevator_tx.cpp  bridge_rx.cpp  floor_display.cpp          the three mains
test/   test_elev_packet/  test_mesh_packet/  test_floor_monitor/
sim/    replay_main.cpp  compare_to_python.py
docs/   HARDWARE.md  FLASHING.md  PROTOCOL.md  ALGORITHM_PORT.md  MESH.md
```

Code from `elevatormons` is copied in and adapted, not referenced — that repo is
the bring-up bench and is not modified by this project.

---

## 9. Known limits carried forward

Everything in ALGORITHM.md §8 still applies: no absolute anchor, ~1.5–2×
turbulence headroom over a gusty evening, the uniform-pitch assumption in the
ladder clamp, and constants tuned to one building and one sensor. Persisting the
model to NVS removes the cold-boot window in practice but does not change any of
these.

New to this system:

- **A flood mesh has no delivery guarantee.** Three sends plus multiple paths make
  loss unlikely, not impossible. A display showing a stale floor is the expected
  failure mode, which is why staleness is displayed rather than hidden.
- **Battery voltage is ±2–3% before calibration**, roughly ±0.4 V at 12 V. Enough
  to answer "does this need charging?", not enough for a fuel gauge.
- **The 24 h window is uptime-relative**, not wall-clock. After a reboot it
  reports less than 24 h of history and does not pretend otherwise.
