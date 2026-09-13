# Building the hardware

Three devices, built in this order: the transmitter (the only one with any
wiring in it), the bridge (twenty minutes, mostly deciding where to put it), and
ten displays (unbox, plug in).

| device | where it lives | power | what it does |
|---|---|---|---|
| transmitter | on top of / inside the elevator car | LiFePO4 4S pack | reads the barometer, decides the floor, transmits LoRa |
| bridge | floor 5 | USB-C wall adapter | receives LoRa, floods it over ESP-NOW |
| display ×10 | one per floor | USB-C wall adapter | shows the floor, relays the mesh |

Everything below is drawn from the design spec
(`docs/superpowers/specs/2026-09-11-simmevator-design.md`), `platformio.ini`,
and the module headers in `src/`. Where a number is not from one of those, it
says so.

---

## 1. Parts

Reproduced from spec section 3.6. This list has already been trimmed once - see
the note after the tables before you add anything back.

**Transmitter (×1 assembly)**

| Part | Qty | Notes |
|---|---|---|
| Seeed XIAO ESP32S3 | 1 | |
| Seeed Wio-SX1262 for XIAO | 1 | stacks onto the XIAO; ships with its 915 MHz antenna |
| BMP390 breakout | 1 | Adafruit 4816 or equivalent |
| LiFePO4 12.8 V 10 Ah pack with BMS | 1 | 4S; BMS required |
| LiFePO4 charger, 14.6 V | 1 | a Li-ion or lead-acid charger will not terminate correctly |
| MP1584EN buck module | 1 | set to 3.3 V **before** connecting the XIAO |
| Resistor 100 kΩ 1% | 1 | divider top |
| Resistor 20 kΩ 1% | 1 | divider bottom |
| Spade terminal pair | 1 | so the pack can come off for charging |
| Hookup wire / JST leads | — | |

No divider filter cap (oversampled in software instead), no buck input
electrolytic (the node averages ~23 mA, so the module's own ceramics are
sufficient), and no inline fuse (the pack's BMS covers overcurrent). The
enclosure is 3D printed.

**Bridge (×1)**

| Part | Qty |
|---|---|
| Seeed XIAO ESP32S3 | 1 |
| Seeed Wio-SX1262 for XIAO | 1 |
| USB-C 5 V adapter + cable | 1 |

**Displays (×10)**

| Part | Qty | Notes |
|---|---|---|
| ELEGOO ESP32 CYD 2.8" ILI9341 240×320 | 10 | sold in 2-packs → 5 packs |
| USB-C 5 V / 1 A adapter | 10 | |
| USB-C cable | 10 | |

Mounts are 3D printed.

**Totals to buy:** 2 × XIAO ESP32S3 · 2 × Wio-SX1262 · 1 × BMP390 · 1 × battery +
charger · 1 × MP1584EN · 2 × resistors · 1 × spade pair · 10 × CYD · 11 × USB-C
supplies. Enclosure and the ten display mounts are printed.

### 1.1 Where is the fuse?

The four things a reader usually expects to find in this list and does not:

| left out | why |
|---|---|
| inline fuse | the pack's BMS is the overcurrent and undervoltage protection. It is inside the pack, on the cell side of the terminals, and it is the reason the pack has to be a BMS pack rather than four bare cells. |
| buck input electrolytic | the node averages ~23 mA. The MP1584EN module's own input ceramics handle that; the electrolytic is there for modules running amps. |
| filter cap across the divider bottom leg | the reading is taken once a minute off a rail that moves over hours. The filtering is done in software instead: 32 samples, median not mean (spec 3.2, `battery.h`). A median also throws away the occasional wild sample the S3's SAR ADC produces, which a capacitor would not. |
| barrel jack, separate antennas, enclosure, display mounts | the Wio-SX1262 ships with its antenna; the enclosure and the ten mounts are 3D printed; the pack connects through spade terminals so it can come off for charging without a connector in the middle. |

If you decide to add the fuse anyway, put it on the pack side of the spade pair
so it protects the wiring and not just the buck.

### 1.2 Tools

Not in the parts list, but you cannot do section 3 without them:

- **A multimeter.** Non-negotiable. Section 3.4 is entirely a meter procedure.
- Soldering iron, thin solder, heat-shrink.
- A small flat screwdriver or trimmer tool that fits the MP1584EN's pot.
- Wire strippers, and a crimp tool if your spade terminals are crimp type.
- A USB-C cable for flashing - and a habit of not having it plugged in at the
  same time as the pack (section 3.5).

---

## 2. Transmitter pinout

Everything the car node uses, in one table. From spec 3.1, `lora_link.h`, and
`platformio.ini`.

| signal | XIAO pad | GPIO | where it comes from |
|---|---|---|---|
| BMP390 SDA | D4 | 5 | your wire |
| BMP390 SCL | D5 | 6 | your wire |
| SX1262 SCK / MISO / MOSI | D8 / D9 / D10 | 7 / 8 / 9 | the stack, no wire |
| SX1262 NSS / DIO1 / BUSY / NRST | - | 41 / 39 / 40 / 42 | the stack, no wire |
| battery sense | D0 | 1 (ADC1_CH0) | your divider |
| 3.3 V in | 3V3 | - | the buck |
| ground | GND | - | common with the pack minus |

The BMP390 runs on `Wire1` at 400 kHz, address 0x77. The radio is 915 MHz,
SF10 / BW125 / CR4-8 at 22 dBm.

---

## 3. Transmitter assembly

Build in this order. Each step is testable before the next one adds a way for
it to fail.

### 3.1 Stack the radio onto the XIAO

The Wio-SX1262 is an expansion board for the XIAO form factor. There is no
wiring: the SPI bus and the four control lines in the table above all run
through the two 7-pin headers.

1. Match the silkscreen. **D0 must sit over D0 and GND over GND.** Both boards
   label their pads; check two on opposite corners before you press them
   together, because the headers will physically mate either way round and the
   wrong way puts the pack rail onto a control line.
2. Press them home evenly. They should sit flat with no gap.
3. Screw the 915 MHz antenna that came with the Wio-SX1262 onto its connector.

**Do not power the radio without the antenna attached.** That is general RF
practice rather than anything measured here - an unterminated PA reflects its
own output back into itself.

Also, from the note in `platformio.ini` next to `LORA_TX_POWER=22`: the
receiver's absolute maximum input is around +10 dBm, and the transmitter puts
out 22 dBm. When you bench-test the transmitter and the bridge together, keep
the two antennas metres apart, not touching.

### 3.2 BMP390, four wires

| BMP390 breakout | XIAO |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA (SDI on Adafruit's silkscreen) | D4 |
| SCL (SCK on Adafruit's silkscreen) | D5 |

Leave SDO alone. Floating or pulled high gives address 0x77, which is what
`platformio.ini` sets. If your breakout straps it low you get 0x76, and
`bmp390BringUp()` falls back to that address and reports the mismatch on the
serial console rather than hiding it - so a board strapped the other way still
works, it just tells you.

Keep the leads short. The bus runs at 400 kHz and there is no reason for it to
be longer than it needs to be.

Sensor placement matters more than it looks. `bmp390_sensor.h` notes that this
is the module the 3 h reference capture was taken with, and that the Bosch
indoor-navigation preset it configures (x8 pressure, x2 temperature, IIR 3,
~5 cm of altitude noise) is what every constant in `floor_monitor.h` was tuned
against. The stillness threshold is 0.08 m, sitting in the gap between ~0.05 m
of stop noise and a car doing up to 1.78 m/s (ALGORITHM.md sections 2 and 5).
Substituting a noisier part - a BMP280, say - does not degrade the system
gracefully, it invalidates the tuning. Fit the BMP390.

Mount the sensor near where the enclosure's vent will be (section 3.6) and not
pressed against the buck module. The vent is what lets it see shaft pressure at
all; sitting it on the warmest thing in the box is an unmeasured risk rather
than a measured one, but it costs nothing to avoid.

### 3.3 The battery sense divider

From spec 3.2:

```
  BATT+ ──[ R1 = 100 kΩ 1% ]──┬── D0 (GPIO1)
                            │
                       [ R2 = 20 kΩ 1% ]
                            │
  BATT− ──────────────────── ┴── GND (common with XIAO GND)
```

Ratio 6.0. Use 1% parts: the ratio is the calibration, and section 4 corrects
one unit's error, not a drifting one.

What the ADC pin actually sees, computed from that ratio:

| pack | pin | meaning |
|---|---|---|
| 14.6 V | 2.433 V | a charger at its termination voltage |
| 13.3 V | 2.217 V | rested full |
| 12.8 V | 2.133 V | CHARGE SOON, ~20% remaining |
| 12.0 V | 2.000 V | CHARGE BATTERY, ~10% remaining |
| 10.0 V | 1.667 V | BMS cutoff territory |

The whole span sits inside the ESP32-S3 ADC's linear region at 12 dB
attenuation, which misbehaves below ~0.15 V and above ~2.8 V. That is the point
of this ratio: not to use the full scale, but to keep both ends of
the pack's range comfortably inside the part of the range that behaves, with
headroom at the top for a charger that overshoots.

Divider draw is ~122 µA at 14.6 V, which is about 0.5% of the ~23 mA average
the 30-day budget is built on - roughly a third of a day off the ~56-day
figure. That is why there is no switching MOSFET across it -
the switch would cost more parts and more ways to fail than it saves.

**Why kilohms and not megohms.** With no filter capacitor, the ADC's internal
sample capacitor charges straight through the divider's Thevenin resistance -
R1 ∥ R2, ~16.7 kΩ here. At 1 MΩ / 200 kΩ that would be ~167 kΩ, slow enough
that each sample can read low and pick up more noise. The ratio, and so every
voltage in the table above, is unchanged.

**Why D0 and not somewhere else.** D0 = GPIO1 = ADC1_CH0 is the only free ADC1
pad on this board:

| pad | taken by |
|---|---|
| D4 / D5 | the BMP390's I²C bus |
| D8 / D9 / D10 | the radio's SPI |
| GPIO39 / 40 / 41 / 42 | the radio's DIO1, BUSY, NSS, NRST |

It has to be ADC1 rather than any ADC. `battery.h`: ADC2 is unusable while WiFi
is active, and while the car node itself does not run WiFi, the bridge and the
displays do - the constraint is written into the shared design so the choice is
the same everywhere.

There is deliberately no capacitor here. See section 1.1.

### 3.4 Set the buck regulator

> ### STOP. This is the step that destroys hardware.
>
> The MP1584EN module arrives with its trimpot at **whatever position it was
> left in on the production line**. That is not 3.3 V, and there is no reason
> for it to be anywhere near 3.3 V. The module's adjustment range runs from
> under a volt to within shouting distance of its input, so fed from a 12.8 V
> pack it can present a voltage at its output that is many times what an
> ESP32-S3 and a BMP390 will survive.
>
> Feeding the XIAO's 3V3 pad goes **straight past the XIAO's own LDO** - there
> is nothing between the buck's output and the chip. Whatever the pot is set to
> is what the silicon gets.
>
> **Set the output with nothing connected to it. Meter it. Only then wire it to
> the XIAO.**
>
> There is no recovery step for getting this wrong. Check the ESP32-S3
> datasheet's absolute-maximum supply figure before you decide a reading is
> close enough; this document deliberately does not quote you a number to feel
> comfortable about.

Procedure:

1. Disconnect the buck's output from **everything**. No XIAO, no BMP390, no
   divider top leg. Nothing but two bare wires.
2. Wire the pack to the buck input through the spade pair, minding polarity.
   IN+ to pack positive, IN− to pack negative.
3. Meter across the buck's OUT+ and OUT−, DC volts, on a range that covers
   15 V - not a 3 V range that will just say "OL" while the pot is high.
4. Turn the pot in small increments and watch the meter. Do not assume which
   way is up, and do not assume a quarter turn will do anything: some of these
   modules carry multi-turn trimmers and take many revolutions to cross the
   range. Turn, read, turn, read.
5. Land on **3.30 V**. A few tens of millivolts under is fine and is the safer
   side to err on.
6. Disconnect the pack. Wait, re-connect, and confirm it still reads 3.3 V -
   you want to know the pot is actually set and not just resting where your
   screwdriver left it.

The MP1584 is PWM-only with no light-load PFM mode (spec 3.3), so it does not
pulse-skip and the no-load reading you just took is representative of the
loaded one. Re-measure once with the node running anyway, at the XIAO's 3V3 pad
rather than at the buck, so the measurement includes your wiring.

That same PWM-only behaviour is why the node's ~3 mA light-sleep current is
served badly by this module. It runs ~80-85% at the ~23 mA average, which is
what the 30-day budget in spec 2 assumes. A TPS62203 or MP2338 would recover a
few days. Not required.

### 3.5 Wire the buck to the XIAO, and never plug in USB at the same time

Only after section 3.4 reads 3.3 V:

1. Buck OUT+ → XIAO **3V3** pad.
2. Buck OUT− → XIAO **GND**, and the same ground node as the divider's bottom
   leg and the pack minus. One ground, not two.
3. Divider top leg to pack positive on the buck-input side of the spade pair,
   so the divider measures the pack and not the buck.

The power chain, end to end (spec 3.3):

```
LiFePO4 4S 10 Ah → spade terminals → MP1584EN buck at 3.3 V → XIAO 3V3 pad
```

Feeding the 3V3 pad bypasses the XIAO's onboard LDO. That is the efficient
path, and it is the whole reason the buck exists - running 12.8 V into a linear
regulator would burn the battery budget as heat.

> **USB and the buck must never be connected at the same time.**
>
> The XIAO's 3V3 pad is its LDO's *output*. Plug in USB and the LDO drives that
> node from 5 V; connect the pack and the buck drives the same node from 3.3 V.
> Two regulators, hard outputs tied together, neither one current-limited in
> the direction of the other.
>
> Whichever sits higher sources current into the other one's output stage. The
> 12.8 V pack has the energy to keep doing that indefinitely. It may survive;
> it may take out the buck, the XIAO's LDO, or the USB port on whatever you are
> flashing from. It is not a failure with a predictable outcome, which is
> exactly why it is not worth characterising empirically on your only XIAO.
>
> **Unplug the pack at the spade terminals before you plug in USB.** Every
> time. That is what the spade pair is for as much as charging is.

### 3.6 The enclosure must be vented

The enclosure is 3D printed, so this is a note for whoever draws it.

**A sealed box turns the barometer into a thermometer and the system stops
working.** Inside a sealed volume the pressure the BMP390 reads is set by the
temperature of the air trapped with it, not by the altitude of the car. Every
constant in the algorithm is measured against shaft pressure.

What is needed is modest: a small hole with a scrap of foam or filter cloth
over it. It has to equalise on the timescale of seconds, not breathe freely.
The car moves a floor in a couple of seconds and the algorithm's stillness
window is 5 samples at 1 Hz, so a vent with a time constant of seconds is
invisible to it and a vent with a time constant of minutes is fatal to it.

Design notes for the print:

- Vent placed away from the buck module, and near where the BMP390 sits.
- Cloth or foam over the hole - it is there to keep dust and drafts off the
  sensor port, not to seal.
- The pack comes out for charging, so the spade pair has to be reachable
  without dismantling the electronics.
- The antenna wants to be outside the box, or at least not wrapped in it.

---

## 4. Calibrating the battery reading

Uncalibrated, the divider is worth ±2-3%, about ±0.4 V at 12 V (spec 9). That
answers "does this need charging?" and is not a fuel gauge. A one-point
correction with a DMM gets most of it back, and it takes two minutes.

The firmware applies a plain integer ratio to the computed pack voltage:

```
reported_mV  =  pin_mV × (R1 + R2) / R2 × VBAT_CAL_NUM / VBAT_CAL_DEN
```

`VBAT_CAL_NUM` and `VBAT_CAL_DEN` both default to 1 in `platformio.ini`, which
means uncalibrated.

**Procedure**

1. Assemble the node and power it from the pack. Let it settle - a pack just
   off the charger is still relaxing.
2. Meter the pack directly at the spade terminals. Write it down in millivolts.
   Call it `TRUE`.
3. Read what the node thinks. The serial console is the place to get it: the
   display's readout is rounded to 0.1 V (spec 6) and that is too coarse to
   calibrate against. Call it `REPORTED`.
4. Set:

```
-DVBAT_CAL_NUM=TRUE
-DVBAT_CAL_DEN=REPORTED
```

5. Rebuild and reflash (pack disconnected - section 3.5).
6. Re-read. It should now agree with the DMM to within the ADC's own noise.

**Worked example.** The DMM reads 13.28 V at the terminals. The node reports
13112 mV. Then:

```
-DVBAT_CAL_NUM=13280
-DVBAT_CAL_DEN=13112
```

and the node's next reading becomes 13112 × 13280 / 13112 = 13280 mV. The
correction is 1.28% high, which is exactly the order of error the ±2-3%
uncalibrated figure predicts.

Use the raw millivolt integers. Do not reduce them to a rounded percentage -
the maths is integer division, and `13280/13112` carries more resolution than
`101/100` does. The intermediate is computed in 64-bit, so there is no headroom
problem with numbers this size.

This is **per unit**. It corrects the resistors you actually soldered and the
eFuse calibration curve of the chip you actually have. If you ever build a
second transmitter, note in the repo which numbers belong to which board,
because nothing in the firmware can tell them apart.

---

## 5. Charging the pack

**14.6 V LiFePO4 charger. Nothing else.**

| charger type | why not |
|---|---|
| Li-ion / LiPo | charges to 4.2 V per cell. A 4S Li-ion charger terminates around 16.8 V, which is well above the 14.6 V a 4S LiFePO4 pack wants. |
| lead-acid / "12 V car battery" | wrong termination behaviour. Lead-acid chargers hold a float voltage indefinitely and some apply temperature compensation or an equalise stage; LiFePO4 wants constant current to 14.6 V, then to be left alone. |

Those chemistry voltages are standard figures, not measurements from this
project. The one number this project does specify is the 14.6 V termination in
spec 3.6.

The pack's BMS will defend itself against a lot of this, and you should not
make it. A BMS disconnecting on overvoltage mid-charge is a fault condition,
not a charging strategy.

**Disconnect the pack at the spade terminals before charging.** The spade pair
exists for exactly this.

What the rest of the system does while it is off, and when it comes back:

| | screens |
|---|---|
| pack off | keep the last floor for 2.5 min, dim, then at 3 min grey out with the LED amber. A CHARGE BATTERY warning stays red throughout |
| pack back on | within a minute the STATS heartbeat arrives, the screens un-grey, and a charged pack clears the battery warning. No pairing, nothing to press |
| until the car visits **floor 1 and floor 10** | the floor shows `--` |
| after that | normal |

The `--` is deliberate. The transmitter keeps the learned building across a
reboot, but not the car's position: the car may have moved and the weather will
have shifted the pressure reference, and a floor restored one out would stay one
out forever. Seeing both ends of the building fixes the position exactly. **So
after reconnecting, ride to floor 1 and floor 10.** It does not correct itself
over time without that, and it does not need to - the first time the car
naturally visits both ends it locks in.

### Voltage to state of charge

From spec 3.2, set in `battery.h` as `VBAT_WARN_MV` / `VBAT_CRITICAL_MV` /
`VBAT_CHARGED_MV`:

| voltage | what every screen shows | notice at ~23 mA |
|---|---|---|
| 13.3 V | rested full - clears any warning | - |
| **12.8 V** | **CHARGE SOON**: amber badge in place of the battery bar, voltage in amber | ~3.5 days (~20% left) |
| **12.0 V** | **CHARGE BATTERY**: steady red banner across the bottom, red badge, onboard LED steady red | under 2 days (~10% left) |
| ~10 V | the BMS disconnects the pack and the car node goes silent | - |

Each 10% of a 10 Ah pack is about 1.8 days at the budgeted draw, so these give
less notice than the voltages suggest. An earlier version warned at 12.0 V
believing it was ~20%; it is nearer 10%.

The warning **latches**. Once a reading crosses a threshold the level stays up,
even if a later reading wobbles back above it, and clears only when the pack
reads 13.3 V - which in practice means it has been charged. A warning that
disappeared by itself would be the easy one to miss. The latch lives on the
transmitter, and the level travels in the STATS flags, so all ten screens agree.

A warning also does not grey out when the link goes stale, although everything
else on the screen does. If the car node goes silent right after CHARGE BATTERY,
the likeliest reason is that the battery ran out, and that is the moment the
warning should stay on the screen.

The thresholds sit on the flat part of the curve, where ±2-3% uncalibrated
accuracy is about ±0.4 V. **Do the one-time calibration** (above) or CHARGE SOON
may come days early or not until CHARGE BATTERY.

Expect to charge roughly every two months if the node is behaving: spec 2
budgets ~23 mA average at evening-peak traffic, which is ~56 days, against a
30-day requirement.

---

## 6. Bridge

Mechanically trivial. The only decision is where it goes.

1. Stack the Wio-SX1262 onto the XIAO exactly as in section 3.1, silkscreen
   matched, antenna screwed on.
2. Flash it (`pio run -e bridge_rx -t upload`).
3. Plug it into a USB-C 5 V adapter on floor 5. It sits in continuous LoRa
   receive and floods what it hears over ESP-NOW, so it wants to stay powered.

**Antenna placement is the part that matters.** ALGORITHM.md section 9 measured
the link and found that loss barely tracks SNR at all (r = −0.13): 5.21% batch
loss over 3 h, 49 dropouts, and even above +6 dB of margin there is still 0.3%
loss. That is shadowing and interference, not a decode floor - which is why
this project spends power on bandwidth and TX power rather than on spreading
factor.

The exception is floor 9. It is the one landing whose SNR range crosses the
SF10 decode floor, bottoming at −2.0 dB against a healthy +14.2 dB median. An
intermittent shadow, and section 9's own conclusion is that it is addressable
by antenna placement rather than by spreading factor.

So:

- Favour line of sight up the shaft. The bridge is on floor 5 and the problem
  landing is above it.
- Get the antenna out of the cabinet, out from behind the steel door, away from
  the ductwork.
- Vertical, and clear of large metal for as much of its length as you can
  manage.
- If floor 9 still shows staleness in service, move the bridge antenna before
  you touch any radio setting. The measurement says placement is the lever.

The bridge also has to reach at least one display over ESP-NOW, since it is the
origin of the flood. In practice the floor 5 display is next to it, so this is
rarely a constraint - but do not put the bridge somewhere the mesh cannot
follow it.

---

## 7. Displays

Ten ELEGOO CYD boards. USB-C power and nothing else - no wiring, no soldering,
no sensors. Flash each one (`pio run -e floor_display -t upload`), mount it, and
plug it into a 5 V / 1 A adapter.

**The mesh is the constraint on where they go.** There is no WiFi network here
and no router. Every display has to hear either the bridge or another display
directly, over ESP-NOW on channel 1, and then relay onward. Spec 4.2:

- Flood rules are identical on the bridge and all ten displays; every node
  relays each new `origSeq` exactly once.
- Hop limit 8. Five floors is the furthest any display sits from the bridge on
  floor 5, so 8 leaves slack for a detour around a dead node.
- `WIFI_PROTOCOL_LR`, Espressif's long-range PHY, is worth about 7 dB over
  802.11b - which is the margin that makes floor-to-floor through concrete
  work at all.

Which means:

- **Do not mount a display inside a metal enclosure.** A shielded box around an
  antenna defeats the mesh, and because the flood is multi-hop it does not just
  break that one screen - it can remove the path the screens above it were
  relying on.
- **Do not put one behind a steel fire door** and expect it to hear the bridge
  through it.
- Prefer a position in the corridor with a path toward the stairwell or the
  shaft, and toward the floors above and below.
- The 3D-printed mounts should hold the board with its back open to the
  corridor, not sandwiched against a steel frame.

A display that cannot hear anything is not silent: it greys out and turns its
RGB LED amber after 180 s without a STATS packet (spec 6). That is how you find
a bad mounting position - walk the building and look at the LEDs.

Power draw is small enough that a 1 A adapter is generous, and the LDR on
GPIO34 handles night dimming on its own, so the board does not need to be
positioned for a light switch.

---

## 8. Pre-power-on checklist

Run this on the transmitter with the pack **disconnected**, before it is ever
connected. Each line is here because getting it wrong costs a board.

| # | check | how | pass looks like |
|---|---|---|---|
| 1 | No shorts on the pack rail | meter in continuity/resistance across the spade pair, pack off | not a short. You should read roughly the divider's 120 kΩ in parallel with whatever the buck input looks like. A beep here means stop. |
| 2 | Polarity, pack to buck | trace IN+ to pack positive and IN− to pack negative by eye and by meter | the buck's silkscreen agrees with the wires |
| 3 | Ground is one node | continuity: XIAO GND to buck OUT− to divider bottom leg to pack minus | all four beep together |
| 4 | **Buck output is 3.3 V** | section 3.4, with the output disconnected from everything | 3.30 V ± a few tens of mV on the meter, checked after a power cycle |
| 5 | Buck output polarity | OUT+ goes to the **3V3** pad, not 5V, not any signal pad | read the silkscreen twice |
| 6 | Divider ratio is sane | meter R1 and R2 in circuit: ~100 kΩ top, ~20 kΩ bottom | ratio near 6.0. If R1 and R2 got swapped the pin sees 5/6 of the pack - over 10 V straight onto a 3.3 V pin. |
| 7 | Divider tap goes to D0 | continuity from the R1/R2 junction to the D0 pad | beeps, and does **not** beep to D1, 3V3 or GND |
| 8 | Nothing on D4/D5 but the BMP390 | visual | SDA to D4, SCL to D5, not crossed |
| 9 | Antenna attached | visual | screwed down before any power |
| 10 | **USB unplugged** | visual | see section 3.5 |

Then, in order:

1. Connect the pack at the spade terminals.
2. Meter the XIAO 3V3 pad against GND with the node running. Still 3.3 V.
3. Check the serial console: the BMP390 should come up (and say which address
   it found), and `batteryBegin()` prints the divider and calibration
   configuration it was compiled with.
4. Meter the pack at the terminals and compare with what the node reports. If
   the two disagree by more than a few percent, go back to check 6 before
   reaching for section 4 - a calibration ratio will happily paper over a
   wrong resistor and leave you with a reading that is right at one voltage and
   wrong everywhere else.

Only when all of that passes does the node go in the car.
