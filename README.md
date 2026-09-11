# simmevator

An elevator tracker and floor display for Simmons Hall, MIT.

A barometer rides in the elevator car and works out which floor the car is on. It
broadcasts that over LoRa to a bridge on the fifth floor, which relays it over
ESP-NOW to a screen on every floor. Each screen shows the floor in numbers you can
read from down the corridor, an up/down arrow while the car is moving, the
transmitter's battery voltage, and how far the elevator has travelled in the last
day.

The car node runs on a battery, unattended, for about two months.

---

## How it works

### The hard part is not what you would guess

A floor is about 2.9 m of altitude, and a BMP390 resolves that easily — noise in
this shaft is 0.045 m, sixty times smaller. The difficulty is that **the reference
pressure moves.** Over a 3 h capture in this building the sea-level reference
wandered 9.9 m of apparent altitude, more than three floor heights. Any fixed
pressure-to-floor table slides through several floors in an evening.

So absolute altitude cannot decide the floor. Two facts rescue it:

- **A trip is short** — 5 to 60 seconds, over which the weather moves the
  reference by less than 0.05 m. The *change* in altitude across a trip is clean
  even when the absolute value is not.
- **Floors are a lattice.** Every jump between stops is an integer multiple of the
  floor pitch, so `round(jump / pitch)` is a strong error-correcting code.

The algorithm therefore reads floor *changes* from jumps between stops and treats
absolute position as a slowly-tracked side quantity. It learns the building as it
goes — the floor spacing, each landing's height, how many floors exist — with
nothing configured in advance. On the reference capture it converges in five
minutes to a pitch of 2.871 m and ten floors, and stays within ±1%.

The algorithm is not this repo's invention. It was developed and measured against
real data in the [`elevatormons`](../elevatormons) bring-up repo, and is fully
documented in `elevatormons/docs/ALGORITHM.md` — including the four approaches
that were tried and failed, which are more instructive than the one that worked.
This repo ports it to C++ and builds a system around it.

### Why the decision happens in the car

The node holds the sensor, so it sees an unbroken 1 Hz stream and decides the
floor locally. It broadcasts the answer, not the raw pressure.

That one choice makes radio loss harmless. A dropped packet can only *age* what a
screen shows; it can never corrupt the floor, because the next packet carries the
current floor outright. There is no state to resync and no way for a bad link to
produce a wrong number. If the bridge decided the floor instead, every dropout
would become a correctness problem.

### The three devices

```
  ELEVATOR CAR (battery)          FLOOR 5 (wall)            FLOORS 1-10 (wall)
  ┌──────────────────────┐        ┌──────────────┐          ┌──────────────┐
  │ XIAO ESP32S3         │        │ XIAO ESP32S3 │  ESP-NOW │ CYD x10      │
  │  BMP390 @ 4 Hz       │  LoRa  │ + Wio-SX1262 │ ───────► │ ILI9341      │
  │  FloorMonitor @ 1 Hz │ ─────► │              │  flood   │ relay + draw │
  │  model in NVS        │ 915MHz │ LoRa -> mesh │          │              │
  │  battery sense       │        │ origin node  │ ◄──────► │ peer relay   │
  │  Wio-SX1262          │        └──────────────┘          └──────────────┘
  └──────────────────────┘
```

**The transmitter** samples the barometer, runs the floor algorithm, and sends two
kinds of packet: an 8-byte `STATE` every 2 seconds while the car is moving, and a
24-byte `STATS` every 60 seconds carrying battery voltage, distance travelled, and
the learned building model.

**The bridge** does nothing but listen on LoRa and re-emit over ESP-NOW. It holds
no state worth losing.

**The displays** flood packets to each other so the signal reaches floors the
bridge cannot: every screen rebroadcasts each new sequence number exactly once, up
to eight hops. Flooding rather than a fixed 5→6→7 chain means one unplugged screen
does not cut off everything above it.

### Why the update rate is what it is

Airtime is the entire power budget. A LoRa packet at SF10 costs hundreds of
milliseconds at about 118 mA, and the car in this building is *moving* 71% of an
evening peak. A flat 2-second cadence is a 15–50% transmit duty cycle, and the
month is gone in under three weeks.

Two changes fix it without making the display feel slower:

- **Transmit only while the car is moving.** Nothing changes while it is parked,
  so there is nothing to say. A 60-second heartbeat carries the slow statistics
  and proves the transmitter is alive.
- **Detect motion at 4 Hz, run the algorithm at 1 Hz.** The stillness test gets
  fast samples so motion onset is caught in about half a second and the first
  packet fires immediately; the algorithm itself still sees the 1 Hz stream its
  constants were tuned for. Between samples the chip light-sleeps, which is the
  difference between a 3 mA baseline and a 25 mA one.

That lands at roughly **19 mA average, about 68 days** on a 10 Ah LiFePO4 pack —
and that figure uses the evening-peak traffic rate, so a real week including nights
does better.

### One correction to the radio settings

`elevatormons/platformio.ini` configures the barometer link at **BW62.5 kHz**, but
`ALGORITHM.md §9` records that the actual field run — the one that measured 5.21%
packet loss over three hours in this shaft — was at **BW125 kHz and only 14 dBm**.

This project uses BW125 at 22 dBm. That is the configuration with real in-shaft
evidence, running with 8 dB more margin than the run that produced the 5.21%
figure, and it costs 297 ms per packet instead of 725 ms. The same section notes
that loss barely tracks signal-to-noise (r = −0.13) — it is shadowing and
interference, not a decode floor — so buying margin with a higher spreading factor
would cost airtime linearly and buy very little.

---

## What each screen shows

```
┌────────────────────────────┬──────────────┐
│                            │      ▲       │
│      ██████     ██         │    UP        │
│      ██  ██     ██         │              │
│      ██  ██     ██         │  12.7 V      │
│      ██  ██     ██         │  ■■■■■□      │
│      ██████     ██         │              │
│                            │  2.1 mi      │
│                            │  today       │
└────────────────────────────┴──────────────┘
```

The floor number is drawn as vector segments rather than a font — anything large
enough to read at distance looks soft as a bitmap. While the car is moving the
number sweeps through the intermediate floors and the arrow lights up; on arrival
it snaps to the floor the algorithm confirmed.

Battery voltage and the 24-hour distance come from the transmitter once a minute,
so all ten screens agree. Between heartbeats each screen extrapolates the distance
from floor changes it observes, so the number ticks in real time rather than
jumping once a minute.

If a screen stops hearing the mesh it dims, then greys out and turns its onboard
LED amber — a dead link is visible from the corridor rather than silently showing
a floor from twenty minutes ago.

---

## Documentation

| Document | What is in it |
|---|---|
| [Design spec](docs/superpowers/specs/2026-09-11-simmevator-design.md) | The whole system: power budget, protocols, verification gates, parts list |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Wiring, the battery divider, the buck regulator, antenna placement |
| [docs/FLASHING.md](docs/FLASHING.md) | Which firmware goes on which board, and how |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Both wire formats, field by field |
| [docs/MESH.md](docs/MESH.md) | How the ESP-NOW flood behaves and how to debug it |
| [docs/ALGORITHM_PORT.md](docs/ALGORITHM_PORT.md) | How the C++ port maps to the Python reference, and the evidence it matches |
| `elevatormons/docs/ALGORITHM.md` | The algorithm itself — the source of truth, not maintained here |

---

## Building

PlatformIO. Four environments:

```bash
pio run -e elevator_tx   -t upload    # the car node
pio run -e bridge_rx     -t upload    # the floor-5 bridge
pio run -e floor_display -t upload    # each of the ten screens
pio test -e native                    # host unit tests
```

The port is verified by replaying the real 3 h capture through the actual C++
implementation and diffing it against the Python reference sample by sample:

```bash
pio run -e sim && .pio/build/sim/program sim/cpp_broadcasts.csv
python3 sim/compare_to_python.py sim/cpp_broadcasts.csv
```

This has to reproduce the reference numbers exactly — ten floors, 2.871 m pitch,
356 floor-to-floor moves, 3409 m travelled — or the port is wrong.

---

## What this will not do

- **There is no absolute anchor.** The floor index is integrated from jumps, so a
  rounding that goes wrong stays wrong until the car happens to correct it. An
  absolute readout from a tracked datum was tried and is measurably worse.
- **It is safe to about 3 hPa/h of pressure change** with the floor count still
  exact, and degrades rather than diverges past that. A typical weather front is
  1–2 hPa/h; the fastest rate in the reference capture was 0.88 hPa/h.
- **Turbulence headroom is roughly 1.5–2×** a confirmed gusty evening. Not zero,
  not large.
- **The constants are tuned to this building and this sensor.** They are measured,
  not universal.
- **Battery voltage is ±2–3% before calibration**, about ±0.4 V at 12 V. Enough to
  answer "does this need charging?", not a fuel gauge.
- **The 24-hour window is relative to uptime**, not wall-clock — there is no RTC.
  After a reboot it reports less than a day of history and says so.
