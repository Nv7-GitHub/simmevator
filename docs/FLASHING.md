# Flashing

Thirty-eight boards, three firmwares, nine environments, one toolchain.
Everything here is run from the repository root, `simmevator/`, because
PlatformIO finds `platformio.ini` by walking up from the working directory.

Six of the thirty-eight are XIAO ESP32S3 modules with a Wio-SX1262 stacked on
top and distinctly odd USB behaviour - a car transmitter and a mid-shaft bridge
for each of the three shafts. The other thirty-two are ELEGOO CYD panels, and
they no longer all take the same binary: there is one display image per shaft,
and the three are indistinguishable on the bench. The thirty-two are the tedious
part, and there is a loop for it further down.

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
~/.platformio/penv/bin/pio run -e bridge_rx_a -t upload
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

Nine environments, three roles across three shafts. The sources are identical
for all three shafts; four build flags are the entire difference, which is
exactly what makes a mis-flashed board hard to see.

| Shaft | Board | Count | Environment | LoRa | Mesh ch | `ELEV_TX_ID` | Labels |
|---|---|---|---|---|---|---|---|
| A | Car transmitter - XIAO + Wio-SX1262 + BMP390 | 1 | `elevator_tx_a` | 913.0 MHz | - | `0x41` `'A'` | - |
| A | Mid-shaft bridge - XIAO + Wio-SX1262 | 1 | `bridge_rx_a` | 913.0 MHz | 6 | accepts `'A'` | - |
| A | Floor display - ELEGOO CYD 2.8" | 11 | `floor_display_a` | - | 6 | - | 11: `B,1..10` |
| B | Car transmitter | 1 | `elevator_tx_b` | 915.0 MHz | - | `0x42` `'B'` | - |
| B | Mid-shaft bridge | 1 | `bridge_rx_b` | 915.0 MHz | 1 | accepts `'B'` | - |
| B | Floor display | 10 | `floor_display_b` | - | 1 | - | 10: `1..10` |
| C | Car transmitter | 1 | `elevator_tx_c` | 917.0 MHz | - | `0x43` `'C'` | - |
| C | Mid-shaft bridge | 1 | `bridge_rx_c` | 917.0 MHz | 11 | accepts `'C'` | - |
| C | Floor display | 11 | `floor_display_c` | - | 11 | - | 11: `B,1..10` |

Elevator B's four values are the single-elevator system's values, unchanged. The
two XIAO roles are ESP32-S3 with native USB-Serial-JTAG (section 7); the CYD is
an ESP32-WROOM-32 behind a CH340 (section 8). Upload with
`pio run -e <env> -t upload`, and add `-t monitor` to open the serial console
straight after:

```bash
pio run -e elevator_tx_a -t upload -t monitor
```

All nine run the console at 115200 baud (`monitor_speed` in `[base]`).

There are now two independent ways to flash the wrong image, and they fail very
differently.

**Wrong role, and it is loud.** `elevator_tx_*` builds the BMP390 driver, the
battery sense and NVS persistence; `bridge_rx_*` builds the ESP-NOW mesh instead
and never touches a sensor. Flashing the bridge firmware onto a car node gives
you a board that is deaf to its own barometer and happily floods nothing. The
console says so on the first boot.

**Wrong shaft, and it is quiet.** Put shaft C's transmitter image on shaft A's
car and bridge A hears nothing at all - it is listening on 913.0 MHz while that
car talks on 917.0 - which reads exactly like a dead SX1262. Worse, bridge C
*will* hear it, accept it, and publish it, because the packet carries `txId`
`'C'` and is a genuine C packet as far as any receiver can tell. Shaft C's
eleven screens would then show shaft A's car. The bridge's `txId` filter
catches a packet from a foreign shaft; it cannot catch an identity you flashed
into the transmitter. That is what section 2.1 is for.

`pio run` with no `-e` builds all nine firmware environments (that is what
`default_envs` in `[platformio]` is for) but uploads nothing, so it is a useful
"does this still compile" check - and see section 10 for why compiling cleanly
is not, on its own, evidence that the nine images differ.

### 2.1 Label every board before it leaves the bench

Write the shaft letter on the board before you unplug the USB cable, in the
same motion as the upload. Not later, not at the end of the batch. A role and a
letter is the whole label: `A-TX` and `A-BR` on the two XIAOs, a bare `A` on
each of the eleven displays, in ink or tape on the back of the case next to the
USB connector where it is readable after installation. Displays need nothing
more than the letter, because within a shaft they are genuinely interchangeable
(section 6). Keep three piles, one per shaft, and a board joins a pile only
after it is labelled.

The failure this prevents is specifically an A-versus-C failure. Both shafts
have eleven landings and therefore the identical label table, `B,1..10`, and
`FLOOR_LABEL_COUNT` 11. A panel running `floor_display_a` and a panel running
`floor_display_c` boot to the same layout, the same glyphs and the same startup
screen. They differ in one number that never appears on the panel: the mesh
channel, 6 versus 11. Install the wrong one of those two and it hears nothing -
`heard` stays at 0, the screen shows `--` forever - which is the same symptom
as a dead bridge, a flat car pack, or a panel simply out of mesh range. The one
board that is actually wrong is the one board that looks blameless.

**CHECK SHAFT (section 12) does not catch this case.** That check compares the
transmitter's floor count against `FLOOR_LABEL_COUNT`, and A and C both say 11.
B is the easy one in both directions - its ten-entry table disagrees with the
other two, so a label table crossed in `platformio.ini` surfaces immediately -
but two images that differ only in a channel number have no software backstop at
all. Ink is the backstop.

The second witness is the firmware itself: every one of the nine builds now
prints its shaft in the startup banner (section 9). So the ink is checkable.
Open the console, read the letter, compare. If they disagree, believe the banner
and reflash - the banner is what the board will actually do. For a car node that
means USB, which means the pack comes off first; see section 3.

---

## 3. Unplug the battery before flashing the transmitter

**Do this before the USB cable goes in. Every time.**

Each of the three car nodes is powered by feeding the MP1584EN buck's 5 V
output into the XIAO's **5V pad**, which is the board's supply input. That pad
and the USB connector's VBUS are the same node, so plugging in USB while the
pack is connected ties the buck's output straight to the host's 5 V rail - two
supplies across each other, with the buck also back-feeding whatever you are
flashing from. Nothing in that arrangement is current limited by design.

So: pull the spade terminals, then plug in USB. That is what the spade
terminals are for - they exist so the pack can come off for charging, and
flashing is the same disconnect.

The three bridges and the thirty-two displays have no battery and no second
supply. Plug them in and go.

This rule is also the reason section 12 exists. `HARDWARE.md` §3.5
forbids USB while the buck feeds the 5V pad, and a commissioning run is by
definition a car running on its pack - so during the one procedure that most
needs a console, the car node is the one board that may not have one.

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
pio run -e floor_display_a -t upload -t monitor \
    --upload-port /dev/cu.usbserial-1410 \
    --monitor-port /dev/cu.usbserial-1410
```

One gotcha specific to the XIAOs: because the USB device is implemented by the
ESP32-S3 itself rather than by a separate UART chip, the port **disappears and
comes back** across a reset, a reflash, or a jump into the ROM bootloader, and
it can come back under a different name. If a monitor session dies mid-upload
that is normal. Re-run `pio device list` rather than assuming yesterday's name.

---

## 5. The six XIAOs

```bash
# car node, shaft A - battery disconnected, see section 3
pio run -e elevator_tx_a -t upload -t monitor

# mid-shaft bridge, shaft A
pio run -e bridge_rx_a -t upload -t monitor
```

Then the same two commands with `_b` and with `_c`. Six boards flashed once
each, so there is no batching to do - but there are six chances to flash the
wrong shaft, so do one shaft at a time, both of its boards, and label both
(section 2.1) before touching the next letter. Leaving all six unlabelled on the
bench and sorting them out afterwards does not work: a XIAO carries no external
sign of what is on it, and telling two of them apart costs a console session
each, which for a car node also costs a battery disconnect.

All six XIAO environments set `monitor_dtr = 0` and `monitor_rts = 0`, which
matters - see the next section.

---

## 6. The thirty-two displays

Three images, eleven plus ten plus eleven boards. **Within** a shaft there is
still no per-unit configuration: every CYD in shaft A gets a byte-identical
binary.

That is deliberate, not an omission: a display does not know or care what floor
it is on. It shows where the *car* is, and every screen in the shaft shows the
same thing at the same time. The floor label table
(`-DFLOOR_LABELS="B,1,2,...,10"` in A and C, `"1,2,...,10"` in B) is a
shaft-wide map, not a per-board setting, and the mesh addresses everything to
the broadcast MAC, so there is nothing to configure and no individual board to
keep track of. A spare CYD flashed for shaft A today can replace any of A's
eleven tomorrow with no reflash of anything else. It cannot stand in for one of
C's, though - same eleven landings, same labels, different channel - so spares
are stocked and labelled per shaft like everything else.

**Across** shafts it is now three separate batches, and the loop below must not
be allowed to run past the end of one pile into the next. Build all three images
first, so the loops are only doing uploads:

```bash
pio run -e floor_display_a -e floor_display_b -e floor_display_c
```

### One shaft at a time, one cable

The least error-prone version. Plug in a board, press Return, wait, unplug,
label it, repeat. The shaft letter is fixed once per pile and echoed back on
every prompt, so there is no point in the batch where the screen stops telling
you which image you are about to write.

```zsh
# zsh - set the letter once per pile
shaft=a
while true; do
  read "?[shaft $shaft] plug in the next CYD and press Return (Ctrl-C when done) "
  pio run -e floor_display_$shaft -t upload || echo ">>> FAILED - retry this board"
done
```

```bash
# bash: same thing, different read syntax
shaft=a
while true; do
  read -p "[shaft $shaft] plug in the next CYD and press Return (Ctrl-C when done) "
  pio run -e floor_display_$shaft -t upload || echo ">>> FAILED - retry this board"
done
```

With one board attached there is no `--upload-port` to get wrong, which is the
whole point of doing it this way.

Count the pile before you start and count the flashed boards after: eleven for
A, ten for B, eleven for C. When a pile is finished, **Ctrl-C out of the loop
and start it again with the next letter.** There is no way to change `shaft`
without leaving the loop, and that is the point - a running loop whose variable
you edited in another window is still flashing the old image, and the boards it
produces are the ones section 2.1 describes as blameless-looking.

### Several at once, on a hub

Faster if you have a powered hub with four or five free ports. Each CYD draws
its own backlight current, so an unpowered hub will brown out partway through.

The glob writes one image to every board attached, which is now a way to put
shaft A's firmware onto shaft C's panels in a single command that reports no
error whatsoever. **Only one shaft's boards may be on the hub at a time**, and
the letter gets re-typed every time the hub is repopulated:

```bash
shaft=a          # re-type this every time the hub is repopulated
for p in /dev/cu.usbserial-* /dev/cu.wchusbserial*; do
  [ -e "$p" ] || continue
  echo "=== $p -> floor_display_$shaft"
  pio run -e floor_display_$shaft -t upload --upload-port "$p" \
    || echo ">>> FAILED $p"
done
```

Uploads run sequentially, roughly ten seconds each once the build is cached.
The `|| echo` matters: one board that refuses to enter its bootloader should not
leave you wondering which of the five it was. Keep flashed boards separated from
unflashed ones physically - the glob does not tell you which is which, a
double-flashed board looks exactly like a missed one, and a board carrying the
neighbouring shaft's image looks exactly like both.

Confirming all thirty-two are alive is easier from the mesh than from the bench,
one shaft at a time - see section 9.

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

That is why all six XIAO environments carry:

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

Note that the three `floor_display_*` environments deliberately do **not** set
these. See section 8.

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
(not an S3) behind a **CH340** USB-to-serial chip. That is why all three
`floor_display_*` environments are `board = esp32dev` and inherit none of the
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
`monitor_dtr = 0` / `monitor_rts = 0` on a display environment would break
automatic uploads and leave you pressing buttons for thirty-two boards.

`platformio.ini` does not set them for the display environments, and that is
correct.
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
PLATFORMIO_UPLOAD_SPEED=115200 pio run -e floor_display_a -t upload
```

Add `-v` to the upload if you want `esptool`'s own chatter about what it
detected and where it gave up.

---

## 9. Did it come up?

What follows describes the *shape* of each board's serial output, not literal
strings - the exact wording lives in `src/elevator_tx.cpp`, `src/bridge_rx.cpp`
and `src/floor_display.cpp` and may be reworded without this document being
wrong. Open the console at 115200 and look for these things.

All nine builds now open with their shaft, so the first line of any console
session is a check of the ink on the case (section 2.1). What each one is
expected to say:

| Shaft | Banner says | Bridge mesh | Screens in the shaft |
|---|---|---|---|
| A | elevator `A`, LoRa 913.0 MHz | channel 6 | 11, labels `B,1..10` |
| B | elevator `B`, LoRa 915.0 MHz | channel 1 | 10, labels `1..10` |
| C | elevator `C`, LoRa 917.0 MHz | channel 11 | 11, labels `B,1..10` |

### The transmitter

A good boot prints, in order:

1. **A radio banner.** `loraBringUp()` announces the role, the shaft and the
   SX1262 settings it applied - including the frequency, and `txId` printed as a
   character rather than a number, so a transmitter reads `A`, `B` or `C`
   directly. It **halts with an explanation on Serial** if the radio
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
| Banner letter disagrees with the ink on the case | Mis-flashed board. Believe the banner and reflash - do not re-ink the label, because the label is what the next person installing it will read |

### The bridge

The bridge brings up two radios. Expect the LoRa banner, then a receive-start
status, then an ESP-NOW mesh banner - `meshBringUp()` halts on failure the same
way `loraBringUp()` does, so a mesh that will not start is visible rather than
silent. The banner names which elevator this build is for, with its frequency
and its mesh channel, so a bridge flashed for the wrong shaft is caught in the
first two seconds of a console session rather than by a shaft full of dark
screens a week later.

After that it is quiet until the car transmits. When it hears a packet you get
a per-packet line: the tag, the length, and the link quality from
`loraLastRssi()` / `loraLastSnr()`. Something arriving every 2 s means the car
is moving; a line roughly once a minute means the car is parked and you are
seeing the STATS heartbeat alone. Nothing at all for over a minute means the
LoRa link is down, not that the car is idle.

Every packet the bridge accepts is from its own car: a frame whose `txId` is
not this bridge's elevator is dropped before dispatch and counted in
`dropForeignTxId`, which `printSummary()` prints alongside `dropTagCount` and
`dropLenCount`. With 2 MHz between the shafts that counter should read 0
forever, and that is the point - a non-zero value is evidence of cross-shaft
leakage rather than an inference from a loss statistic. It is a bench
instrument, though, not a field one; section 13 says why.

Each accepted packet is also wrapped and flooded, so the mesh counters'
`published` count should track the packets heard. `meshPrintCounters()` puts
that whole structure on one line.

| What you see | What it means |
|---|---|
| Banner right, `dropForeignTxId` climbing | Another shaft's car is on this frequency - two transmitters flashed for the same shaft, or a frequency flag that did not take. Section 10 |
| Banner right, nothing heard at all for minutes | The link is down, as it always was - but check the car node's banner letter before the radio, because a transmitter flashed for another shaft is talking on a frequency this bridge never listens to |
| `foreignOrigins` non-zero | A second bridge is minting frames on this channel. Two bridges flashed for the same shaft; their `origSeq` values collide and get deduped against each other |

### A display

Expect the mesh banner - which names the elevator this build is for - then
counters. The useful fields, all defined in `espnow_mesh.h`:

| Counter | Reading it |
|---|---|
| `heard` | Frames the WiFi task handed up, before validation. Zero means the board is not in radio range of anything, or is on the wrong channel - which now most often means it is carrying another shaft's image (section 2.1) |
| `accepted` | Decoded and new to this node. This is the one that means "the mesh reaches this floor" |
| `deduped` | Decoded, but already seen. Large and growing is *healthy* - it means several neighbours are relaying to you |
| `relayed` | Put back on the air after the 5-40 ms jitter wait |
| `dropCrc`, `dropMagic`, `dropVersion` | Non-zero suggests interference or a node running a different build |
| `dropHopExhausted` | Frames that ran out of their 8 hops here. A few is fine; a lot means the flood is looping further than it should |

The panel itself is the other half of the check. Backlight up and a rendered
layout means TFT_eSPI is configured and the SPI bus works. A dash in the big
digit position rather than a number means "nothing heard since boot", and is
expected for up to one STATS interval on a cold start.

A screen reading LEARNING, ANCHORING or CHECK SHAFT is not a display fault
either - those are the commissioning readout, and section 12 is entirely about
what they mean and what to do about each.

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

This builds the suites in `test/` - `test_elev_packet`, `test_mesh_packet`,
`test_floor_monitor`, and the commissioning suite added for this deployment -
against the headers in `src/`.
Nothing from `src/*.cpp` is compiled in (`build_src_filter = -<*>`), so a
failure points at a header, not at a `main()`. Unity prints one line per test
case and a per-suite summary; PASS is every suite reporting zero failures and
the run ending in a green summary.

`native` defines none of the four per-elevator flags, so this is one run, not
nine: the eleven-entry label table, the `'B'` glyph, the `txId` filter and the
four commissioning states are all exercised as data passed in, rather than by
building the firmware three times. The tests therefore prove the logic is right
for every shaft, and prove nothing at all about which values reached which
image - that is the next check.

### The nine builds must actually differ

`pio run` compiling all nine environments is not evidence that the nine images
are different. A build matrix that has collapsed - three shafts all inheriting
B's numbers, or a display environment that lost its label flags - compiles
cleanly, uploads cleanly, and behaves like the single-elevator system with two
extra shafts of screens that never show anything. Nothing in a passing build
reveals it, so check it directly.

`pio project config` prints the configuration PlatformIO computed after
resolving `extends`, which is where a collapse would happen:

```bash
pio project config > /tmp/pioconf.txt
for e in elevator_tx_{a,b,c} bridge_rx_{a,b,c} floor_display_{a,b,c}; do
  printf '%-17s' "$e"
  awk -v env="env:$e" '$0==env{i=1;next} i&&/^env:/{exit} i' /tmp/pioconf.txt \
    | grep -oE -- '-D(LORA_FREQUENCY|MESH_CHANNEL|ELEV_TX_ID|FLOOR_LABEL_COUNT)=[^ ]+' \
    | sed 's/^-D//' | tr '\n' ' '
  echo
done
```

Nine rows, and exactly three distinct sets of values among them - every `_a`
environment agreeing with every other `_a`, and disagreeing with `_b` and `_c`:

```
elevator_tx_a    LORA_FREQUENCY=913.0f MESH_CHANNEL=6  ELEV_TX_ID=0x41 FLOOR_LABEL_COUNT=11
elevator_tx_b    LORA_FREQUENCY=915.0f MESH_CHANNEL=1  ELEV_TX_ID=0x42 FLOOR_LABEL_COUNT=10
elevator_tx_c    LORA_FREQUENCY=917.0f MESH_CHANNEL=11 ELEV_TX_ID=0x43 FLOOR_LABEL_COUNT=11
...
```

Compare each row against the table in section 2. A row carrying B's `915.0f`
where it should carry A's `913.0f` is the collapse. Whether a row lists all
four names or only the ones its role uses depends on how the per-elevator
sections are arranged in `platformio.ini` - a car node never reads
`MESH_CHANNEL` and a display never reads `LORA_FREQUENCY` - so what is being
checked is that every value a row does carry is its own shaft's, and that a
display environment carries a `FLOOR_LABEL_COUNT` at all. This reads the
computed configuration rather than the built image, which is the right place to
read it: the nine images are compiled from identical sources and these four
flags are the whole difference, so a configuration that is right cannot produce
an image that is wrong.

Belt and braces after a full `pio run`, since three identical images is the
symptom being hunted:

```bash
md5 -q .pio/build/floor_display_{a,b,c}/firmware.bin   # md5sum on Linux
```

Three different checksums. Two that match means two shafts built the same image.

### The replay

This is the one that proves the algorithm port is faithful. It streams the 3 h
reference capture through the same `FloorMonitor` the car runs and diffs the
result against the Python implementation in `elevatormons`, sample for sample.

Like `native`, the `sim` environment defines no elevator flags - the algorithm
is identical on all three shafts, which is the whole premise of this
deployment. The `n_floors` 10 gate below is the reference capture's building,
not a statement about elevator B, and it does not become 11 because two of the
three shafts now have a basement.

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
reports `floor = 0`, "model not ready", and the displays show a dash until the
first STATS packet arrives and then switch to LEARNING (section 12).

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
| Minute 0 | Boot lines, NVS reports empty, screens show `--` - nothing has been heard yet |
| Within one STATS interval, so within a minute | The screens switch to **LEARNING**, a count in the big digits over `OF 11` (`OF 10` in B). This is the system working, not failing |
| Minute 2 | Still LEARNING, and the count is sitting still or has jumped several floors at once. **This is the moment people conclude it is broken.** It is not, and section 12 explains why the count is a span rather than a progress bar |
| ~Minute 5, given normal traffic | Pitch and floor count settle, the model goes ready, screens start showing numbers and they stay right |
| Every reboot after that | Screens show **ANCHORING** until the car has visited **the bottom landing and the top landing**, in either order. Then the floor appears and is right. No second bootstrap |
| Any time, on one screen | **CHECK SHAFT** - that panel is carrying the wrong shaft's label table. Section 12 |
| Any time, on every screen in a shaft | **CHECK SHAFT** - the car has never been to its lowest landing. In A and C that is the basement. Section 12 |

A battery swap, a reset, a reflash of unrelated code - none of them cost you
another five-minute bootstrap, because the learned pitch and height ladder come
back out of flash. `nvsModelLoad()` reporting `NVS_MODEL_OK` at boot is that
happening.

What does *not* come back is where the car is. The weather moves the pressure
reference while the node is off, and the car may have moved too, and a floor
restored one out would stay one out forever. So the node waits until it has
seen both ends of the building, which pins the position exactly. The
transmitter's serial log says `(position unknown - ride to the bottom and top
floors)` while it waits, and the screens say ANCHORING. **After every battery
swap: ride to the bottom landing and the top landing.** In elevators A and C
the bottom landing is the basement, not floor 1 - a car sent to 1 and to 10 in
those shafts has not been to the bottom, and will wait exactly as long as if it
had not moved.

The one case where you *want* the bootstrap back is a node moved to a different
building, where the learned pitch and ladder are wrong rather than stale.
`nvsModelErase()` exists for that - a deliberate cold boot.

---

## 12. Commissioning: reading the verdict off the screens

Every shaft goes through this. The trap it exists to catch is specific to A and
C, because it is about the lowest landing never being visited, and a basement
is far likelier to go unpressed for hours than a lobby is. B's commissioning is
what it always was - section 11 - and nothing here changes it.

### 12.1 There is no console to read this from

The obvious commissioning step is to read the floor count off a serial console.
In this building that is not available:

- **The bridge console is unreachable in practice.** It is a wall-powered box
  mid-shaft. Reading it means standing at it with a laptop, but the answer is
  only known after a full end-to-end run, by which time you are somewhere else
  in the building.
- **The transmitter console is forbidden.** `HARDWARE.md` §3.5 requires that
  USB is never plugged in while the buck feeds the XIAO's 5V pad, and a
  commissioning run is by definition the car running on its pack. See section 3.

So during the exact procedure that needs verifying, **there is no serial port
anywhere in the system.** The displays are the only instrument available, which
is why the verdict is rendered on them.

### 12.2 The four states

All four are derived from fields the wire already carries - the floor count and
the NVS-restored flag out of STATS, the model-ready bit out of STATE - so there
is no new packet, no new flag and nothing to enable.

| Screen | What it means | What to do |
|---|---|---|
| `--` | Nothing heard since boot. Not a commissioning state at all | Wait one STATS interval (60 s). Still `--` after that, see section 9 |
| **LEARNING**, count in the big digits with `OF 11` beneath (`OF 10` in B) | The model is not ready and the span learned so far is smaller than the building | Nothing. Let the car run. Read 12.4 before watching the number |
| **ANCHORING**, in words rather than digits | The model came back from NVS intact, but the node does not yet know where the car is | Ride to both ends, in either order. Section 12.5 |
| A floor label | Normal operation | Nothing |
| **CHECK SHAFT** | The model is ready and confident, and reports a floor count this building does not have | Section 12.3 |

### 12.3 CHECK SHAFT means one of exactly two things

Either **the car has never visited its lowest landing**, or **this display is
carrying the wrong shaft's label table.**

Tell them apart by counting screens. Every screen in the shaft showing it points
at the car: send the car to the basement, and to the top, and the learned span
grows to eleven and the screens leave CHECK SHAFT by themselves. One screen out
of eleven showing it points at that board: reflash it with its own shaft's
environment (sections 2 and 2.1) - there is nothing wrong with the car.

The first case is worth being precise about, because it is the reason this
state exists at all. `floor` index 1 is defined as *the lowest landing ever
seen*, not as a name, and the model is learned at runtime from confirmed stops.
Commission elevator A over a period in which nobody presses B, and it learns a
span of ten landings, sets `modelReady`, and starts broadcasting confident
indices 1-10 - which the display maps onto the labels `B,1,2,...,9`. **Every
label on all eleven of A's screens is then one floor too low, indefinitely, and
looks entirely plausible.**

Nothing inside the algorithm can catch that. `anchorStep()` locks when the span
of indices seen since boot equals the learned span
(`floor_monitor.h:879-900`); in this failure the learned span and the span seen
since boot are the same number, so they agree, and the model is internally
consistent and wrong. The only thing in the whole system that knows
the building has eleven landings is `FLOOR_LABEL_COUNT` on the display, so the
display is the only place the check can live.

### 12.4 The count is not a progress bar

`nFloors()` is `maxIndex() - minIndex() + 1` (`floor_monitor.h:530`) - **the
span of learned indices, not a tally of landings visited.** Two consequences,
both of which contradict the obvious reading:

- An express run from the basement to floor 10 produces indices 1 and 11, and
  the readout jumps **straight from 2 to 11**. It will not tick 3, 4, 5.
- The span only means anything once pitch is established, and pitch comes from
  ordinary multi-stop traffic rather than from one long run.

Someone watching the digits expecting a progress bar will call it broken twice:
once while the number sits still for longer than feels reasonable, and again
when it jumps several floors without passing through anything in between.

So the commissioning instruction is **"let the car run normally for a while,
then walk the shaft and check that every screen shows a floor rather than CHECK
SHAFT"**, not "watch it count up".

### 12.5 Order does not matter

Bottom-then-top and top-then-bottom are the same operation.
`floor_monitor.h:888` already says so for the restore case - *"ride to the
bottom floor and the top floor, in either order"* - and for a fresh learn the
span grows the same either way. Neither the code nor the procedure prefers a
direction. This is stated explicitly because the alternative is that someone
invents a ritual, teaches it to the next person, and it becomes a step that a
future failure gets blamed on.

### 12.6 The procedure, end to end

1. Flash, label and install (sections 2.1, 5, 6).
2. Let the car run under ordinary traffic. The model is learned from jumps
   between stops, so an elevator nobody rides teaches it nothing - about five
   minutes on the reference capture's evening traffic, longer on a quiet
   Sunday (section 11).
3. In A and C, make sure the car has actually been to the basement and to the
   top at least once, in either order. Nobody may press B for hours; do it
   deliberately rather than waiting for a passenger to.
4. Walk the shaft. Every screen showing a floor label means done. Any screen
   showing CHECK SHAFT means section 12.3.

---

## 13. The bench coexistence test

Do this before any of it goes into the building: all three transmitters and all
three bridges powered on one table, within a metre of each other, at full
22 dBm.

That geometry is deliberately worse than the building can produce. In the
building the shafts are far apart and each bridge hears its own car far more
strongly than it hears the other two. On one table all three cars are
equidistant from all three bridges, so the near-far margin that frequency
separation is meant to provide is removed on purpose. Passing here means the
separation itself is doing the work, not the floor plan.

All six XIAOs run from USB here, with the packs off - which is what makes every
console in the system legal at once, and is the reverse of the situation in
section 12.1. No displays are needed; nothing being tested lives past the
bridge.

Budget time for it. Six boards sitting on a table are six *parked* cars, so the
traffic is the STATS heartbeat alone, one packet per car per minute. Half an
hour gives each bridge about thirty packets to count, which is enough to
compare against a baseline; three packets is not.

Pass conditions, all read from the bridge consoles:

- each bridge's `publishCount` advances at **its own** car's cadence, and only
  its own - one a minute parked, every 2 s from a car that is actually moving
  (section 9)
- each bridge's `dropForeignTxId` stays at 0
- each bridge's `foreignOrigins` stays at 0
- no bridge's loss statistics degrade when the other two are powered versus when
  they are not

The last one needs a baseline: run each bridge with the other two switched off
first and write the numbers down. The comparison is the test; the absolute value
is just the shaft's own 5.21% (`ALGORITHM.md` §9) and says nothing about
coexistence on its own.

### 13.1 Why this is a bench test and not a building test

Because both new counters live in the bridge console, and section 12.1 has
already established that the bridge console is unreachable during real
operation. **`dropForeignTxId` and `foreignOrigins` are bench instruments, not
field instruments**, and it is worth saying so rather than implying a field
visibility that does not exist. The bench is the one place where a laptop is
already attached to all three bridges at once, which is exactly why the test
belongs here.

In the field the argument for non-interference is the frequency and channel
separation itself, plus the fact that a filtered foreign packet degrades to
silence rather than to a wrong floor. The field instruments are the screens:
section 12.2's four states.

---

## 14. Command reference

In everything below, `<x>` is the shaft letter: `a`, `b` or `c`.

| Task | Command |
|---|---|
| Build everything that goes on hardware (all nine) | `pio run` |
| Flash a car node | `pio run -e elevator_tx_<x> -t upload` |
| Flash a bridge | `pio run -e bridge_rx_<x> -t upload` |
| Flash one display | `pio run -e floor_display_<x> -t upload` |
| Build the three display images | `pio run -e floor_display_a -e floor_display_b -e floor_display_c` |
| Check the nine builds differ | `pio project config`, then section 10 |
| Flash and watch | add `-t monitor` |
| Pick a port | add `--upload-port /dev/... --monitor-port /dev/...` |
| List ports | `pio device list` |
| Console only, no flash | `pio device monitor -e elevator_tx_<x>` |
| Host unit tests | `pio test -e native` |
| Build the replay | `pio run -e sim` |
| Run the replay | `.pio/build/sim/program sim/cpp_broadcasts.csv` |
| Check the replay | `python3 sim/compare_to_python.py sim/cpp_broadcasts.csv` |
| Throw away the build cache | `pio run -t clean` |

`pio device monitor -e <env>` picks up that environment's `monitor_speed`,
`monitor_dtr` and `monitor_rts` - which for the six XIAOs is the difference
between a working console and a silent one, so prefer it over a bare
`pio device monitor`. Any of the three letters will do for that purpose, since
the monitor settings are identical across the shafts; the letter matters for
`-t upload`, and only for `-t upload`.
