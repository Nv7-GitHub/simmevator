# Flashing

Twelve boards, three firmwares, one toolchain. Everything here is run from the
repository root, `simmevator/`, because PlatformIO finds `platformio.ini` by
walking up from the working directory.

Two of the twelve are XIAO ESP32S3 modules with a Wio-SX1262 stacked on top and
distinctly odd USB behaviour. Ten are ELEGOO CYD panels that all take the
identical binary. The ten are the tedious part, and there is a loop for it
further down.

---

## 1. Getting PlatformIO

PlatformIO Core is a Python program. The installer drops it in a virtualenv
under `~/.platformio/penv/` and does **not** put it on `PATH`.

If you already have it:

```bash
~/.platformio/penv/bin/pio --version
```

If that prints a version, skip to the PATH note. If it does not:

```bash
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py)"
```

Installing the VS Code PlatformIO IDE extension gets you the same `penv` - the
extension is a front end over exactly this core, so either route works and they
share one toolchain directory.

### Putting `pio` on PATH

For the current shell only:

```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"
```

To make it permanent, on the zsh that macOS ships:

```bash
echo 'export PATH="$HOME/.platformio/penv/bin:$PATH"' >> ~/.zshrc
```

Open a new terminal afterwards, or `source ~/.zshrc`.

If you would rather not touch `PATH` at all, every command in this document
works with the full path substituted for `pio`:

```bash
~/.platformio/penv/bin/pio run -e bridge_rx -t upload
```

The rest of this document writes `pio`. Expand it in your head if you skipped
the PATH step.

### First build is slow

The first `pio run` for an ESP32 environment downloads the Espressif platform,
the Arduino framework and the xtensa/riscv toolchains - several hundred MB,
several minutes. Later builds are cached. The two host environments (`native`,
`sim`) need only a C++17 compiler and are fast from the start.

---

## 2. What goes on which board

| Board | Count | Environment | Chip / USB | Upload command |
|---|---|---|---|---|
| Car transmitter - XIAO + Wio-SX1262 + BMP390 | 1 | `elevator_tx` | ESP32-S3, native USB-Serial-JTAG | `pio run -e elevator_tx -t upload` |
| Floor-5 bridge - XIAO + Wio-SX1262 | 1 | `bridge_rx` | ESP32-S3, native USB-Serial-JTAG | `pio run -e bridge_rx -t upload` |
| Floor display - ELEGOO CYD 2.8" | 10 | `floor_display` | ESP32-WROOM-32 behind a CH340 | `pio run -e floor_display -t upload` |

Add `-t monitor` to open the serial console straight after the upload:

```bash
pio run -e elevator_tx -t upload -t monitor
```

All three run the console at 115200 baud (`monitor_speed` in `[base]`).

The two XIAO environments are **not** interchangeable. `elevator_tx` builds the
BMP390 driver, the battery sense and NVS persistence; `bridge_rx` builds the
ESP-NOW mesh instead and never touches a sensor. Flashing the bridge firmware
onto the car node gives you a board that is deaf to its own barometer and
happily floods nothing.

`pio run` with no `-e` builds all three firmware environments (that is what
`default_envs` in `[platformio]` is for) but uploads nothing, so it is a useful
"does this still compile" check.

---

## 3. Unplug the battery before flashing the transmitter

**Do this before the USB cable goes in. Every time.**

The car node is powered by feeding the MP1584EN buck's 5 V output into the
XIAO's **5V pad**, which is the board's supply input. That pad and the USB
connector's VBUS are the same node, so plugging in USB while the pack is
connected ties the buck's output straight to the host's 5 V rail - two supplies
across each other, with the buck also back-feeding whatever you are flashing
from. Nothing in that arrangement is current limited by design.

So: pull the spade terminals, then plug in USB. That is what the spade
terminals are for - they exist so the pack can come off for charging, and
flashing is the same disconnect.

The bridge and the displays have no battery and no second supply. Plug them in
and go.

While the terminals are off, it is also worth metering the buck's output before
reconnecting - see `docs/HARDWARE.md`. It should read 5 V, and if your module
has a trim pot rather than a fixed output, that is a screw a jostled enclosure
can move.

---

## 4. Picking the port

With exactly one board plugged in, PlatformIO finds it and you can ignore this
section entirely. With several plugged in it will pick one, and it will not
necessarily be the one you meant.

```bash
pio device list
```

On macOS the two board families are easy to tell apart by name:

| Board | Typical device name |
|---|---|
| XIAO ESP32S3 (native USB) | `/dev/cu.usbmodem*` |
| CYD (CH340 bridge) | `/dev/cu.usbserial-*` or `/dev/cu.wchusbserial*` |

On Linux they are `/dev/ttyACM*` and `/dev/ttyUSB*` respectively. On Windows
both are `COM<n>` and you have to read the Device Manager description.

Name the port explicitly with `--upload-port`, and `--monitor-port` if you are
also opening the console:

```bash
pio run -e floor_display -t upload -t monitor \
    --upload-port /dev/cu.usbserial-1410 \
    --monitor-port /dev/cu.usbserial-1410
```

One gotcha specific to the XIAOs: because the USB device is implemented by the
ESP32-S3 itself rather than by a separate UART chip, the port **disappears and
comes back** across a reset, a reflash, or a jump into the ROM bootloader, and
it can come back under a different name. If a monitor session dies mid-upload
that is normal. Re-run `pio device list` rather than assuming yesterday's name.

---

## 5. The two XIAOs

```bash
# car node - battery disconnected, see section 3
pio run -e elevator_tx -t upload -t monitor

# floor-5 bridge
pio run -e bridge_rx -t upload -t monitor
```

Both are single boards flashed once, so there is no batching to do. Both
environments set `monitor_dtr = 0` and `monitor_rts = 0`, which matters - see
the next section.

---

## 6. The ten displays

There is no per-unit configuration. Every CYD gets a byte-identical binary.

That is deliberate, not an omission: a display does not know or care what floor
it is on. It shows where the *car* is, and every screen shows the same thing at
the same time. The floor label table (`-DFLOOR_LABELS="1,2,...,10"`) is a
building-wide map, not a per-board setting, and the mesh addresses everything to
the broadcast MAC, so there is nothing to configure and no board to keep track
of. A spare CYD flashed today can replace any of the ten tomorrow with no
reflash of anything else.

Practically, that means ten repetitions of one command. Build once first so the
loop is only doing the upload:

```bash
pio run -e floor_display
```

### One at a time, one cable

The least error-prone version. Plug in a board, press Return, wait, unplug,
repeat.

```zsh
# zsh
while true; do
  read "?Plug in the next CYD and press Return (Ctrl-C when done) "
  pio run -e floor_display -t upload || echo ">>> FAILED - retry this board"
done
```

```bash
# bash: same thing, different read syntax
while true; do
  read -p "Plug in the next CYD and press Return (Ctrl-C when done) "
  pio run -e floor_display -t upload || echo ">>> FAILED - retry this board"
done
```

With one board attached there is no `--upload-port` to get wrong, which is the
whole point of doing it this way.

### Several at once, on a hub

Faster if you have a powered hub with four or five free ports. Each CYD draws
its own backlight current, so an unpowered hub will brown out partway through.

```bash
for p in /dev/cu.usbserial-* /dev/cu.wchusbserial*; do
  [ -e "$p" ] || continue
  echo "=== $p"
  pio run -e floor_display -t upload --upload-port "$p" \
    || echo ">>> FAILED $p"
done
```

Uploads run sequentially, roughly ten seconds each once the build is cached.
The `|| echo` matters: one board that refuses to enter its bootloader should not
leave you wondering which of the five it was. Keep flashed boards separated from
unflashed ones physically - the glob does not tell you which is which, and a
double-flashed board looks exactly like a missed one.

Confirming all ten are alive is easier from the mesh than from the bench - see
section 9.

---

## 7. XIAO ESP32S3 quirks

### The silent console

The XIAO's USB-C port is the ESP32-S3's own USB-Serial-JTAG peripheral, not a
CH340 or a CP2102. On that peripheral, **DTR and RTS are wired to the reset and
boot0 logic** - the same lines `esptool` pulses to put the chip into its
bootloader.

Most serial monitors assert DTR and RTS when they open a port. On a normal USB
UART that does nothing interesting. Here it holds the board in reset. The
symptom is maddening because nothing looks wrong: the port enumerates, the
monitor connects without error, `pio device list` shows it, and not one byte
ever arrives. It reads as dead firmware when the board is only being held down.

That is why both XIAO environments carry:

```ini
monitor_dtr = 0
monitor_rts = 0
```

which tells PlatformIO's monitor to deassert both on open, so opening the
console is a pure listen.

If you use a different terminal program - `screen`, `minicom`, the Arduino IDE,
a Python script - you must do the equivalent there, or you will get the same
silence. `screen /dev/cu.usbmodemXXXX 115200` generally behaves; monitors with a
"toggle DTR on connect" option generally do not. When a XIAO console is silent,
suspect the monitor before the firmware.

Note that `floor_display` deliberately does **not** set these. See section 8.

### Forcing the bootloader by hand

`esptool` normally puts the chip into download mode by itself, using those same
DTR/RTS lines. When it cannot - a firmware that crashes early enough to stop
servicing USB, a half-finished flash, a board that comes up in a boot loop - the
upload fails with a connect or handshake error and you do it manually.

The XIAO has two small buttons beside the USB connector: **B** (BOOT) and **R**
(RESET).

1. Hold **B** down.
2. Tap **R** and release it.
3. Keep holding **B** for another second, then release.
4. Re-run `pio device list`. The port has re-enumerated and **may have a
   different name** than before.
5. Run the upload against that port.

The board is now sitting in the ROM download stub and will stay there until it
is reset, so there is no rush between steps 4 and 5. After the upload completes,
tap **R** once to run the new firmware.

If the upload still fails after that, the usual causes are a charge-only USB
cable and a hub that cannot supply enough current for the SX1262's transmit
peaks. Try the cable and a direct port before suspecting the board.

---

## 8. CYD quirks

The CYD is an entirely different animal from the XIAOs: an ESP32-WROOM-32
(not an S3) behind a **CH340** USB-to-serial chip. That is why the
`floor_display` environment is `board = esp32dev` and inherits none of the
XIAO's USB-CDC flags.

### CH340 driver

The chip needs a driver on macOS and on Windows. Linux has it in-kernel and
needs nothing.

- **macOS**: recent versions ship a usable CH34x driver; if the board does not
  appear in `pio device list`, install WCH's CH34x macOS driver and reboot.
- **Windows**: install WCH's CH341SER package. Without it the board shows in
  Device Manager as an unknown device.
- **Linux**: it should appear as `/dev/ttyUSB0` on plug-in. If it does not, the
  usual cause is `brltty` grabbing the device on some distributions, not a
  missing driver.

### Do not disable DTR/RTS here

This is the exact inverse of the XIAO. On the CYD, DTR and RTS run through the
normal two-transistor auto-reset circuit to EN and IO0, which is how `esptool`
gets the board into its bootloader without anyone touching a button. Setting
`monitor_dtr = 0` / `monitor_rts = 0` on `floor_display` would break automatic
uploads and leave you pressing buttons for ten boards.

`platformio.ini` does not set them for `floor_display`, and that is correct.
If you are copying monitor settings between environments, this is the line not
to copy.

### When auto-reset misbehaves

Some units are marginal - the auto-reset capacitor value and the USB port's
current all play into it. The failure looks like `esptool` retrying and then
giving up with a connect error.

The manual route on the CYD:

1. Hold the **BOOT** button (labelled BOOT or IO0, next to the reset button).
2. Start the upload.
3. Release BOOT once `esptool` reports it has connected.

Or press and release RESET while holding BOOT, then start the upload, as with
the XIAO. Either gets you into download mode.

A slower flash baud also helps on a long or flaky cable. PlatformIO maps any
project option to an environment variable, so this needs no edit to
`platformio.ini`:

```bash
PLATFORMIO_UPLOAD_SPEED=115200 pio run -e floor_display -t upload
```

Add `-v` to the upload if you want `esptool`'s own chatter about what it
detected and where it gave up.

---

## 9. Did it come up?

What follows describes the *shape* of each board's serial output, not literal
strings - the exact wording lives in `src/elevator_tx.cpp`, `src/bridge_rx.cpp`
and `src/floor_display.cpp` and may be reworded without this document being
wrong. Open the console at 115200 and look for these things.

### The transmitter

A good boot prints, in order:

1. **A radio banner.** `loraBringUp()` announces the role and the SX1262
   settings it applied. It **halts with an explanation on Serial** if the radio
   will not initialise, so a dead or unseated Wio-SX1262 shows up as a bring-up
   failure line followed by nothing at all. A board that prints a failure and
   stops is not a crashed board - it is the radio layer refusing to continue,
   because every later call would fail identically.
2. **An I²C scan and sensor report.** `bmp390BringUp()` prints which addresses
   answered on `Wire1`, which one the driver attached to, and the chip
   revision. A missing or miswired BMP390 shows as a scan with nothing at 0x77
   and nothing at the 0x76 fallback. Unlike the radio, the sensor does not halt
   the node: bring-up returns false and the firmware carries on, so watch for
   the scan result rather than for silence. Downstream, that condition rides on
   the air as the `sensorErr` bit in both packet types.
3. **An NVS line.** `nvsModelLoad()` reports one of the statuses in
   `nvs_model.h` by name. On a board that has never run, `NVS_MODEL_EMPTY` is
   the correct and expected answer. `NVS_MODEL_OK` means a learned model came
   back and the node is already up to speed. Anything in the
   `nvsModelStatusIsFault()` set - bad magic, bad size, bad schema, bad CRC,
   open or write errors - means there was a record and it was unusable, which
   is worth a second look rather than a shrug.
4. **A running stream** at the algorithm's 1 Hz cadence, carrying altitude, the
   floor decision and the packet schedule.

Failure signatures worth recognising:

| What you see | What it means |
|---|---|
| Radio banner fails, output stops | SX1262 not talking - reseat the expansion module, check the stack is fully home |
| Scan finds no BMP390 at 0x77 or 0x76 | Sensor wiring on D4/D5, or a board strapped to the other address |
| Everything boots, floor stays unavailable | Normal on a fresh unit. See section 11 |
| Console completely silent, port healthy | The monitor is asserting DTR/RTS. See section 7 |

### The bridge

The bridge brings up two radios. Expect the LoRa banner, then a receive-start
status, then an ESP-NOW mesh banner - `meshBringUp()` halts on failure the same
way `loraBringUp()` does, so a mesh that will not start is visible rather than
silent.

After that it is quiet until the car transmits. When it hears a packet you get
a per-packet line: the tag, the length, and the link quality from
`loraLastRssi()` / `loraLastSnr()`. Something arriving every 2 s means the car
is moving; a line roughly once a minute means the car is parked and you are
seeing the STATS heartbeat alone. Nothing at all for over a minute means the
LoRa link is down, not that the car is idle.

Each accepted packet is also wrapped and flooded, so the mesh counters'
`published` count should track the packets heard. `meshPrintCounters()` puts
that whole structure on one line.

### A display

Expect the mesh banner, then counters. The useful fields, all defined in
`espnow_mesh.h`:

| Counter | Reading it |
|---|---|
| `heard` | Frames the WiFi task handed up, before validation. Zero means the board is not in radio range of anything, or is on the wrong channel |
| `accepted` | Decoded and new to this node. This is the one that means "the mesh reaches this floor" |
| `deduped` | Decoded, but already seen. Large and growing is *healthy* - it means several neighbours are relaying to you |
| `relayed` | Put back on the air after the 5-40 ms jitter wait |
| `dropCrc`, `dropMagic`, `dropVersion` | Non-zero suggests interference or a node running a different build |
| `dropHopExhausted` | Frames that ran out of their 8 hops here. A few is fine; a lot means the flood is looping further than it should |

The panel itself is the other half of the check. Backlight up and a rendered
layout means TFT_eSPI is configured and the SPI bus works. A dash in the big
digit position rather than a number means "floor unknown" - either no packet
has arrived yet or the transmitter is reporting that its model is not ready.
Both are expected on a cold start; neither is a display fault.

Staleness is your link indicator from across a corridor: no STATE for 10 s dims
the readout, no STATS for 180 s greys it out and turns the RGB LED amber. Since
STATS goes out every 60 s, three consecutive misses are needed before a screen
greys - so an amber LED is a real link problem, not a hiccup.

---

## 10. Verifying without any hardware

Both host environments run on the laptop and need no board plugged in. Run them
before a flashing session, not after - if the port is broken, a stairwell is a
bad place to find out.

### Unit tests

```bash
pio test -e native
```

This builds the three suites in `test/` - `test_elev_packet`,
`test_mesh_packet`, `test_floor_monitor` - against the headers in `src/`.
Nothing from `src/*.cpp` is compiled in (`build_src_filter = -<*>`), so a
failure points at a header, not at a `main()`. Unity prints one line per test
case and a per-suite summary; PASS is every suite reporting zero failures and
the run ending in a green summary.

### The replay

This is the one that proves the algorithm port is faithful. It streams the 3 h
reference capture through the same `FloorMonitor` the car runs and diffs the
result against the Python implementation in `elevatormons`, sample for sample.

```bash
pio run -e sim
.pio/build/sim/program sim/cpp_broadcasts.csv
python3 sim/compare_to_python.py sim/cpp_broadcasts.csv
```

The comparator needs `numpy` and `pandas`, and it imports `floor_algorithm.py`
from `elevatormons/tools/` - the Python reference is imported rather than
copied, because a copied golden reference drifts.

PASS looks like this:

- a line reporting the floor sequence **identical over 10950 samples**, which is
  the whole capture on the gap-filled 1 Hz grid;
- a `PASS` on each of the five gates, printed as measured against required:
  `n_floors` 10, `pitch` 2.871 m ±1 mm, `distance_m` 3409 ±0.1%, `trips` 357 ±1,
  `stops` 255 ±1;
- the gates printed twice - once for the Python reference, once for the C++ -
  because both are held to them;
- a final `VERDICT: PASS`, and exit status 0.

The floor-sequence line is the gate that matters. The five numeric gates only
bound how far the gap-filled run may drift from the published measurement; the
sequence comparison is what "the port is faithful" actually means. If the
sequence diverges, the comparator prints the first twenty divergences with three
samples of altitude context either side and a marker on the filled samples, so
you can see immediately whether a miss landed inside an interpolated gap.

One number here looks wrong and is not: `trips`/`stops` are 357/255, while
`ALGORITHM.md` §5 reports 356/256. §5 measured the *raw* capture, which contains
49 dropouts. The replay runs the gap-filled stream the node would actually have
seen, and filling the worst hole - 41 s between two consecutive real samples,
40 invented grid points wide, the same hole ALGORITHM.md §9 records as the
worst dropout - invents one smooth ride across it, which is one extra
floor-to-floor move and one fewer dwell long enough to confirm. The Python
reference produces 357/255 on this input too, so the two implementations still
agree exactly. `docs/ALGORITHM_PORT.md` §6 is the long version.

---

## 11. First run: the five minutes someone will panic in

A freshly flashed transmitter with empty NVS **does not report a floor.** It
reports `floor = 0`, "model not ready", and the displays draw a dash instead of
a number.

This is correct behaviour and it is measured, not defensive. `ALGORITHM.md` §8:
every stop in the reference capture that landed more than a metre off its rail
happened before t = 226 s, inside the bootstrap window. Those are label errors
from before the model exists, not pressure events. After the model converges,
nothing exceeds a metre. A node that published during that window would publish
wrong floors, so it withholds instead.

What it needs is not time but **traffic**: the model is learned from jumps
between stops, so an elevator nobody is riding teaches it nothing. On the
reference capture - normal evening use - the pitch and floor count were
established in about five minutes. An idle Sunday morning could take
considerably longer. This is unmeasured beyond that one capture.

So, the sequence someone should expect on a brand-new install:

| When | What you see |
|---|---|
| Minute 0 | Boot lines, NVS reports empty, screens show a dash |
| Minute 2 | Still a dash. **This is the moment people conclude it is broken.** It is not |
| ~Minute 5, given normal traffic | Pitch and floor count settle, the model goes ready, screens start showing numbers and they stay right |
| Every reboot after that | Screens show a dash **until the car has visited the bottom floor and the top floor**, in either order. Then the floor appears and is right. No second bootstrap |

A battery swap, a reset, a reflash of unrelated code - none of them cost you
another five-minute bootstrap, because the learned pitch and height ladder come
back out of flash. `nvsModelLoad()` reporting `NVS_MODEL_OK` at boot is that
happening.

What does *not* come back is where the car is. The weather moves the pressure
reference while the node is off, and the car may have moved too, and a floor
restored one out would stay one out forever. So the node waits until it has seen
both ends of the building, which pins the position exactly. The transmitter's
serial log says `(position unknown - ride to the bottom and top floors)` while it
waits. **After every battery swap: ride to floor 1 and floor 10.**

The one case where you *want* the bootstrap back is a node moved to a different
building, where the learned pitch and ladder are wrong rather than stale.
`nvsModelErase()` exists for that - a deliberate cold boot.

---

## 12. Command reference

| Task | Command |
|---|---|
| Build everything that goes on hardware | `pio run` |
| Flash the car node | `pio run -e elevator_tx -t upload` |
| Flash the bridge | `pio run -e bridge_rx -t upload` |
| Flash one display | `pio run -e floor_display -t upload` |
| Flash and watch | add `-t monitor` |
| Pick a port | add `--upload-port /dev/... --monitor-port /dev/...` |
| List ports | `pio device list` |
| Console only, no flash | `pio device monitor -e elevator_tx` |
| Host unit tests | `pio test -e native` |
| Build the replay | `pio run -e sim` |
| Run the replay | `.pio/build/sim/program sim/cpp_broadcasts.csv` |
| Check the replay | `python3 sim/compare_to_python.py sim/cpp_broadcasts.csv` |
| Throw away the build cache | `pio run -t clean` |

`pio device monitor -e <env>` picks up that environment's `monitor_speed`,
`monitor_dtr` and `monitor_rts` - which for the two XIAOs is the difference
between a working console and a silent one, so prefer it over a bare
`pio device monitor`.
