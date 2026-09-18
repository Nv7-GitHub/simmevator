# Three elevators: A, B and C

Deploying this system across three shafts instead of one, where two of them
have a basement.

The guiding constraint: **elevator B must behave exactly as the single-elevator
system does today.** Every change here is either a per-elevator build flag or a
new state that only appears when something is misconfigured. There is no change
to either wire format, no change to the algorithm, and no change to the normal
operating UI.

---

## 1. What is actually being deployed

| | elevator A | elevator B | elevator C |
|---|---|---|---|
| landings | B, 1-10 | 1-10 | B, 1-10 |
| landing count | 11 | 10 | 11 |
| transmitters | 1 | 1 | 1 |
| bridges | 1 | 1 | 1 |
| displays | 11 | 10 | 11 |

Three transmitters, three bridges, 32 displays. The three shafts are far apart
in the building, but far apart is not a guarantee - LoRa at 22 dBm carries a
long way indoors, and ESP-NOW on the 802.11 LR PHY is more sensitive than
ordinary WiFi. The design assumes all three systems can hear each other and
makes that harmless, rather than assuming they cannot and finding out later.

---

## 2. The frequency and channel plan

| | elevator A | elevator B | elevator C |
|---|---|---|---|
| `LORA_FREQUENCY` | 913.0 MHz | **915.0 MHz** (unchanged) | 917.0 MHz |
| `MESH_CHANNEL` | 6 | **1** (unchanged) | 11 |
| `ELEV_TX_ID` | `0x41` `'A'` | `0x42` `'B'` | `0x43` `'C'` |
| `FLOOR_LABEL_COUNT` | 11 | **10** (unchanged) | 11 |
| `FLOOR_LABELS` | `B,1..10` | **`1..10`** (unchanged) | `B,1..10` |

Everything else - spreading factor, bandwidth, coding rate, sync word, TX power,
preamble, hop limit, repeats, jitter, dedup depth - is identical on all three
systems and unchanged from today.

### 2.1 Why the LoRa link needs separate frequencies

Three cars on one frequency is pure ALOHA with no carrier sense. A STATE packet
is 297 ms and goes out every 2 s while the stream is active. The right duty
figure is **0.90, the STATE-active fraction, not the 0.712 moving fraction** -
the 2026-09-11 spec §2.2 makes this correction in bold, and for the same reason
it matters here: STATE is not gated on motion, it runs for
`STATE_HOLD_AFTER_STOP_MS` (10 s) past every stop, and those packets are the
same 297 ms at the same 22 dBm and collide identically. So each car offers
`0.5 x 0.90 = 0.45` packets per second. ALOHA's vulnerable window is two packet
lengths, `2 x 0.297 = 0.594 s`, so the probability a given STATE packet survives
two interferers is

```
P(survive) = exp(-2 x 0.45 x 0.594) = exp(-0.5346) = 0.586
```

**About two STATE packets in five are lost**, on top of the 5.21% the shaft
already costs (`ALGORITHM.md` §9), and concentrated in exactly the evening peak
when all three cars are busy simultaneously. That is not a degradation the
display's staleness logic is designed to absorb.

(`exp(-2G)` assumes Poisson arrivals and STATE is a periodic 0.5 Hz stream, so
this is an estimate rather than a prediction. It does not need to be better than
that: both the figure above and the one the moving fraction would give are
decisively against sharing a frequency, which is the only decision it informs.)

**A per-elevator sync word does not fix this and must not be mistaken for a
fix.** The sync word is checked after preamble detection and demodulation; it
filters frames that were already received successfully. It does nothing to stop
two 22 dBm carriers from destroying each other in the air. Frequency separation
is the only mechanism here that prevents collisions rather than merely
classifying the wreckage.

2 MHz of separation against a 125 kHz occupied bandwidth is roughly 16 channel
widths. That margin is chosen against the near-far case - a transmitter in one
shaft at close range to another shaft's bridge - not against the typical case,
where the shafts are far apart anyway. 913.0 / 915.0 / 917.0 all sit inside the
902-928 MHz ISM band with wide margin to both edges, and the modulation,
occupied bandwidth and duty cycle are unchanged from the single-elevator
deployment, so nothing about the regulatory picture changes.

The sync word stays 0x34 on all three. Changing it would buy nothing real and
is one more per-elevator value to get wrong.

### 2.2 Why the ESP-NOW meshes are separated by channel

No display ever needs another shaft's data. Isolating the three floods on
channels 1, 6 and 11 means **each mesh is exactly the ten-node, single-origin
system that was already measured and tuned**, so every number in `MESH.md`
remains valid as written.

The alternative - one building-wide flood carrying elevator-tagged frames -
creates two problems that would then have to be solved:

1. **`origSeq` collides.** Three bridges each mint a monotonic u16 into one
   flood. Bridge A's frame 4113 and bridge C's frame 4113 are indistinguishable
   to `meshDedupSeenOrInsert()`, which tests equality against a ring of recent
   values with no notion of who minted them. The second one is dropped as a
   duplicate. Not once - continuously, at random, forever. Fixing it means an
   origin field in the envelope and a `MESH_VERSION` bump.
2. **The relay jitter stops working.** `MESH_REPEATS` is 3 and the jitter window
   is 5-40 ms. With 32 displays and 3 bridges, a single frame reaching every
   node produces around 96 transmissions contending for a 35 ms window, each
   about 1.3 ms long - roughly 3.5x oversubscribed. `MESH.md` §5 is explicit
   that the jitter window existing at all is the difference between a mesh that
   works and one that does not; oversubscribing it re-creates precisely the
   collision it was introduced to prevent.

Channel isolation makes both problems not exist, rather than patching them. It
costs nothing that is wanted: the only thing given up is cross-shaft path
diversity, which is worthless when the shafts are far apart.

A side benefit worth recording: because `MESH_CHANNEL` is per-elevator, any one
shaft can be moved to a different channel independently if a site survey finds
its channel congested (`MESH.md` §11). The only rule is that two shafts within
earshot must not share a channel.

### 2.3 Why the mesh needs no elevator field

Adding an elevator id to the mesh envelope was considered and rejected. With the
floods on separate channels, a display physically cannot receive another shaft's
frames, so the field would never discriminate anything at runtime.

The same fact disposes of the failure it would have guarded. A display flashed
for the wrong shaft is not shown the wrong car - it is **deaf**. It sits on a
channel its landing's bridge does not transmit on, hears nothing at all, and
stays on the boot splash. That is the loudest failure available: a screen that
never comes up is noticed on the day it is installed, which is exactly when
somebody is standing in front of it. An elevator field in the envelope would be
one byte spent to detect a condition in which no bytes arrive.

`MESH_VERSION` therefore stays 1, `MESH_MAGIC` stays 0x5E1E, and the envelope is
byte-for-byte what is deployed today.

---

## 3. `txId`, and the bridge's foreign-packet filter

`txId` has been in both LoRa formats since the beginning, reserved for exactly
this. It is now meaningful: `'A'`, `'B'` or `'C'` as ASCII, so that a hex dump
reads `E0 42 ...` and identifies the shaft without a lookup table. That choice
is for the benefit of someone decoding a capture in a stairwell, which is the
audience `PROTOCOL.md` is written for.

`bridge_rx.cpp` does not look at `txId` today - `handleFrame()` checks the tag,
dispatches on it, and wraps whatever arrives. It gains one check: a packet whose
`txId` is not this bridge's elevator is dropped and counted in a new
`dropForeignTxId` counter, alongside the existing `dropTagCount` and
`dropLenCount`.

With 2 MHz of separation that counter should read 0 forever. **That is the
point.** A non-zero value is unambiguous evidence of cross-shaft leakage rather
than an inference from a loss statistic, and it is the pass condition for the
bench test in §7.2.

The bridge continues to strip `txId` before wrapping, exactly as
`meshWrapLora()` does today. Displays still never see it. Nothing downstream of
the bridge changes.

### 3.1 The pre-existing foreign-origin warning

`bridge_rx.cpp:295` already warns when it hears a mesh frame on its channel that
it did not mint, with the text *"a second origin on channel N would collide in
the origSeq space"*. That guard was written for the single-elevator system and
happens to be exactly the instrument that detects two bridges accidentally
sharing a channel. It needs no change, but it does need documenting as a
deliberate part of the three-elevator verification story.

---

## 4. The basement, and the commissioning readout

This is the part that is a correctness problem rather than a configuration
problem, and it deserves the most care.

### 4.1 The failure

`floor` index 1 is defined as *the lowest landing ever seen*, not as a name. The
model is learned at runtime from confirmed stops. If elevator A is commissioned
over a period in which nobody presses B, the model learns a span of 10 landings,
sets `modelReady`, and starts broadcasting confident floor indices 1-10 - which
the display maps onto labels `B,1,2,...,9`.

**Every label on all eleven of A's displays is then one floor too low,
indefinitely, and looks entirely plausible.**

Nothing inside the algorithm can detect this. `anchorStep()` exists to resolve an
unknown *offset* after an NVS restore, and it locks when the span of indices seen
since boot equals the learned span (`floor_monitor.h:879-900`). In this failure
the learned span is 10 and the seen span is 10; they agree. The model is
internally consistent and wrong.

The only thing in the system that knows the building has eleven landings is
`FLOOR_LABEL_COUNT` on the display. So the check must live there.

### 4.2 Why the check cannot live on a console

The obvious commissioning step - read `nFloors` off a serial console - is
unusable in this building:

- **The bridge console is unreachable in practice.** It is a wall-powered box
  mid-shaft. Reading it means standing at it with a laptop, but the answer is
  only known after a full end-to-end run, by which time the person is elsewhere.
- **The transmitter console is forbidden.** `HARDWARE.md` §3.5 requires that USB
  is never plugged in while the buck is feeding the XIAO's 5V pad. A
  commissioning run happens with the car on battery, which is precisely the
  condition under which no USB cable may be attached.

During the exact procedure that needs verifying, there is no serial port
anywhere in the system. The displays are the only instrument available, so the
verdict has to be rendered on them.

### 4.3 The four states, from fields already on the wire

`elevator_tx.cpp:270` sends `floor = 0` whenever `modelReady` is clear, and
`floor_monitor.h:1032` sets `b.modelReady = modelReady() && !positionUnknown_` -
so the learning case and the post-restore anchoring case are indistinguishable
from STATE alone. `ELEV_FLAG_NVS_RESTORED` in STATS separates them, which is the
"restored and trusted versus learned here" distinction `PROTOCOL.md` §3 already
describes that flag as existing for.

| `nFloors` | `MODEL_READY` | `NVS_RESTORED` | screen |
|---|---|---|---|
| `< labelCount` | 0 | 0 | **LEARNING** - big digits show `nFloors`, the word `LEARNING` and `OF 11` beneath |
| `== labelCount` | 0 | 1 | **ANCHORING** - words, not digits: ride to both ends |
| `== labelCount` | 1 | - | normal operation |
| `!= labelCount` | 1 | - | **CHECK SHAFT** - the off-by-one trap |

No wire change. No new flag. No reserved bit spent.

**All three inputs must be taken from STATS.** `MODEL_READY` is also in STATE,
and reading it from there is wrong in a way that is not obvious: STATE only runs
while the car moves plus a 10 s hold, so on a display that booted into a parked
shaft it is false because nothing has arrived, not because the car said
anything. Combined with an `nFloors` that did arrive, that reads as "full span,
not ready" - ANCHORING - so every screen in a healthy parked shaft would ask to
be ridden to both ends after any display reset. One packet, one snapshot.

The last row is the one that matters: a model that is ready and confident but
reports a floor count the building does not have. **It has exactly one cause** -
a car that has never reached its lowest landing - and it necessarily appears on
every screen in the shaft at once, because every display in a shaft carries the
same label table and hears the same bridge.

A display carrying the wrong shaft's label table is *not* a second cause, though
it looks like one. Such a display is on the wrong mesh channel too, so it hears
nothing and renders nothing (§2.3). CHECK SHAFT on one screen out of eleven is
not a diagnosis to plan for - it cannot happen.

### 4.4 What the count readout is, and is not

`nFloors()` is `maxIndex() - minIndex() + 1` (`floor_monitor.h:530`) - **the span
of learned indices, not a tally of landings visited.** Two consequences the
documentation must state plainly, because the obvious reading is wrong:

- **It is not a progress bar.** An express run from the basement to floor 10
  produces indices 1 and 11 and the readout jumps straight from 2 to 11. It will
  not tick 3, 4, 5.
- **It is only meaningful once pitch is established**, which needs ordinary
  multi-stop traffic rather than one long run.
- **It must not be shown as a bare number.** The learned span of a shaft being
  commissioned runs 2..11, which is exactly the range of real floor labels, and
  it is drawn in the same seven-segment glyphs the floor number uses. Without
  the word `LEARNING` at a size that survives corridor distance, a passer-by
  reads the count as the floor the car is on. The `OF 11` caption does not
  carry that on its own - it is the first thing to stop resolving as you walk
  away.

So the commissioning instruction is *"let the car run normally for a while, then
check that every screen in the shaft shows a floor rather than CHECK SHAFT"* -
not *"watch it count up"*.

### 4.5 Order does not matter

Bottom-then-top and top-then-bottom are the same operation.
`floor_monitor.h:888` already says so for the restore case: *"ride to the bottom
floor and the top floor, in either order."* For the fresh-learn case the span
grows regardless of direction. Neither the procedure nor the code prefers a
direction, and the documentation should say so explicitly so that nobody invents
a ritual.

### 4.6 `--` does not go away

`--` remains correct for "frames are arriving but there is no confirmed floor",
which is a different condition from all four states above and must not be given
a number.

A display that has heard *nothing* since boot is a third thing again, and it
does not show `--` either: `floor_display.cpp` only calls `displayUpdate()` once
a frame has arrived, so until then the boot splash stays on the glass. Three
conditions, three appearances - splash for "nothing has ever arrived", `--` for
"arriving, no floor yet", and the commissioning screens for "arriving, and the
model and the building disagree".

### 4.7 Rendering the basement label

No rendering change is needed. `segmentsFor()` in `display_ui.cpp:97` already has
a seven-segment glyph for `'B'`, and `DISPLAY_MAX_LABELS` is 32, so an
eleven-entry label table fits with room to spare. This was verified by reading
the code, not assumed.

---

## 5. Code changes

Deliberately small. Every item is either a build flag or a new state that only
appears when something is misconfigured.

### 5.1 `platformio.ini`

Three new per-elevator sections holding four values each:

```ini
[elev_a]
build_flags = -DLORA_FREQUENCY=913.0f -DMESH_CHANNEL=6 -DELEV_TX_ID=0x41
[elev_b]
build_flags = -DLORA_FREQUENCY=915.0f -DMESH_CHANNEL=1 -DELEV_TX_ID=0x42
[elev_c]
build_flags = -DLORA_FREQUENCY=917.0f -DMESH_CHANNEL=11 -DELEV_TX_ID=0x43
```

`LORA_FREQUENCY` and `MESH_CHANNEL` are already defined in `[lora_base]` and
`[mesh_base]`. The per-elevator value must win. Since both are consumed through
`#ifndef` guards in `lora_link.h` and `espnow_mesh.h`, a duplicate `-D` on the
command line is a redefinition warning at best and a silent wrong value at
worst. **The base sections must stop defining the per-elevator values entirely**,
so that each value is defined in exactly one place, and a missing elevator
section is a compile error rather than a silent fallback to B's numbers.

Nine firmware environments: `elevator_tx_{a,b,c}`, `bridge_rx_{a,b,c}`,
`floor_display_{a,b,c}`. `default_envs` lists all nine. `native` and `sim` are
unchanged.

The label flags move from `[env:floor_display]` into the per-elevator display
envs: A and C get `FLOOR_LABEL_COUNT=11` and `FLOOR_LABELS="B,1,...,10"`, B keeps
10 and `1,...,10`.

### 5.2 `src/bridge_rx.cpp`

- `dropForeignTxId` counter next to `dropTagCount` / `dropLenCount`.
- In `handleFrame()`, after the tag check and before dispatch: read `txId` at
  `ELEV_OFF_TXID` and drop if it is not `ELEV_TX_ID`, logging the offending
  value and the RSSI in the same style as the unknown-tag path.
- The counter appears in `printSummary()`.
- The startup banner prints which elevator, frequency and channel this build is
  for, so a console session immediately reveals a mis-flashed bridge.

### 5.3 `src/display_ui.h` / `src/display_ui.cpp`

- `DisplayUiState` gains `uint8_t modelFloors` and `bool modelFloorsValid`.
- A commissioning state enum derived from those plus the existing `modelReady`
  and an `nvsRestored` flag that must also be plumbed through.
- Rendering for LEARNING (count in the big digits, `OF n` beneath), ANCHORING
  (text) and CHECK SHAFT (text), reusing the existing centred-message path
  rather than inventing a new layout.

### 5.4 `src/floor_display.cpp`

- `applyStats()` already unpacks `nFloors` and `flags` and throws both away
  after a `Serial.printf`. Capture `s.nFloors`, `elevStatsNvsRestored(&s)` and
  `elevStatsModelReady(&s)` into statics.
- `buildUiState()` populates the new fields. Note that `modelReady` for the
  commissioning decision is the STATS copy, while `positionValid` keeps using
  the STATE copy - see §4.3 for why crossing the two breaks a parked shaft.
  These are two different questions: "may this node animate right now" follows
  STATE, "what did the car last say about its model" follows the heartbeat.
- The startup banner names the elevator this build is for.

### 5.5 `src/elevator_tx.cpp`

- `ELEV_TX_ID` loses its `#define ELEV_TX_ID 1` fallback at line 86-87, so that
  building a transmitter without an elevator selected fails loudly.
- The startup banner already prints `txId`; it should print it as a character.

### 5.6 Tests (`test/`)

New host-side coverage, run by `pio test -e native`:

- an eleven-entry label table parses, and index 1 is `B`
- the four commissioning states are selected correctly from every relevant
  combination of `nFloors`, `modelReady` and `nvsRestored`
- a `txId` that is not this elevator's is rejected; one that is, is accepted
- `'B'` has a non-zero seven-segment glyph

---

## 6. Documentation changes

| file | change |
|---|---|
| `README.md` | three elevators, the deployment table, the per-shaft radio plan |
| `docs/PROTOCOL.md` | §1 gains the frequency plan; `txId` is no longer "only one car exists"; a new section on the three-system collision analysis and why sync words are not the answer |
| `docs/MESH.md` | the channel plan and the isolation argument; why not one flood; the `origSeq` and jitter arithmetic from §2.2; §11's survey becomes per-shaft |
| `docs/FLASHING.md` | nine environments, the which-image-on-which-board table, board labelling, the commissioning procedure, the four screen states, the bench coexistence test |
| `docs/HARDWARE.md` | 3x parts, per-shaft bridge placement, eleven-landing shafts |
| `docs/ALGORITHM_PORT.md` | untouched - the algorithm does not change |

---

## 7. Verification

### 7.1 Every link, and why it does not collide

The table the documentation must carry, one mechanism per row:

| communication | competes with | isolation mechanism | observable |
|---|---|---|---|
| car X -> bridge X (LoRa) | cars Y, Z | 2 MHz frequency separation | bridge loss statistics |
| car Y -> bridge X (LoRa) | - | frequency, then `txId` filter | `dropForeignTxId` (bench) |
| bridge X -> displays X (ESP-NOW) | bridges Y, Z | separate channels 1/6/11 | `foreignOrigins` (bench) |
| display <-> display within X | its own shaft | unchanged jitter and dedup | `MeshCounters` |
| mesh X | building WiFi on that channel | per-shaft site survey | `MESH.md` §11 procedure |
| LoRa (915 MHz) | ESP-NOW (2.4 GHz) | different band | none needed |

### 7.2 Bench coexistence test

All three transmitters and all three bridges powered at once, on a bench, with
laptops attached.

**Keep every board at least 1 m from every other board.** This is a minimum, not
a maximum, and it is a hardware limit rather than a matter of taste. The
SX1262's absolute maximum RF input is about +10 dBm (`HARDWARE.md` §3.1) and the
transmitters run at 22 dBm, so in free space at 915 MHz:

```
  0.10 m  ->  +10.3 dBm    over the absolute maximum
  0.25 m  ->   +2.4 dBm
  0.50 m  ->   -3.6 dBm
  1.00 m  ->   -9.7 dBm    about 20 dB of margin
```

The crossover is 10.4 cm. Six boards "on one table" is exactly how two of them
end up 5 cm apart, and the damage is to a receiver front end rather than to the
test.

**What this proves, and what it does not.** Equidistant boards are a near-far
ratio of zero, so this is not the worst case - it is the case where every
foreign signal arrives at the *same* power as the wanted one. That is a real
stress on adjacent-channel rejection, and far harsher than the typical building
geometry where a foreign car is much further away than your own. But the genuine
worst case - a foreign car a metre from your bridge while your own car is ten
floors up the shaft - is not reproducible on a bench, and this test should not
be described as though it were.

Pass conditions:

- each bridge's `publishCount` advances at its own car's cadence
- each bridge's `dropForeignTxId` stays at 0
- **no** `foreignOrigins` field appears in any bridge's summary line -
  `printSummary()` prints that counter only when it is non-zero, so the pass
  condition is its absence, not a printed zero
- no bridge's loss statistics degrade when the other two are powered versus when
  they are not

This is where `dropForeignTxId` earns its place, because a laptop is attached.

### 7.3 Instrument honesty

**`dropForeignTxId` and `foreignOrigins` are bench instruments, not field
instruments.** Both live in the bridge console, which §4.2 establishes is
unreachable during normal operation. The documentation must say so rather than
implying field visibility that does not exist. In the field, the argument for
non-interference is the frequency and channel separation, plus the fact that a
filtered foreign packet degrades to silence rather than to a wrong floor.

The field instruments are the displays: §4.3's four states.

### 7.4 Build and test verification

- `pio test -e native` - all existing tests plus §5.6, on the host
- `pio run` - all nine firmware environments compile
- a check that the nine environments actually differ: extract
  `LORA_FREQUENCY`, `MESH_CHANNEL`, `ELEV_TX_ID` and `FLOOR_LABEL_COUNT` from
  each built environment and confirm they match the table in §2. A build matrix
  that silently collapses to three identical images is the most likely way this
  change goes wrong, and it is invisible in a passing build.

---

## 8. What is deliberately not being done

- **No mesh envelope change.** No elevator field, no `MESH_VERSION` bump. §2.3.
- **No LoRa format change.** Both formats are byte-identical to today.
- **No per-elevator sync word.** §2.1 - it does not prevent collisions.
- **No shared display showing all three cars.** One display per elevator per
  floor, so the existing single-car UI is unchanged.
- **No runtime elevator selection.** Identity is a build flag. The mitigation
  for a mis-flashed board is §4.3's CHECK SHAFT state plus physical labelling,
  not a touch-screen picker.
- **No anchoring progress on the wire.** The seen-span is not transmitted and
  will not be. Spending a reserved `state` bit on a condition that resolves
  itself in one round trip is not worth the wire surface.
