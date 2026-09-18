# The ESP-NOW flood

Eleven ESP32s in elevator B - the bridge on floor 5 and one display on each of
floors 1 through 10 - share one broadcast address and one rule: *if you have
not seen this `origSeq` before, wait a random moment and send it on once*.
There is no routing table, no pairing, no per-floor configuration and no MAC
address written down anywhere. A display is interchangeable with a spare
straight out of the box.

Elevators A and C run that same mesh with twelve nodes, the extra one being the
basement display. The three shafts are three separate floods on three separate
channels, and no node ever hears another shaft's traffic - that is the central
decision of the three-elevator deployment and it has its own section, §13.
Everything between here and there describes one shaft, and says so wherever the
node count is part of the arithmetic.

This file covers why it is built that way, which numbers are safe to turn, and
how to work out what is wrong while standing in the stairwell with a laptop.

The frame itself is specified in `src/mesh_packet.h` and in section 4.2 of the
design spec. The radio and schedule are in `src/espnow_mesh.h`. Everything
below is sourced from those two, from `platformio.ini`, or from the spec.

---

## 1. What is on the air

```
off  size  field
0    u16   magic     0x5E1E
2    u8    version   1
3    u8    hop       remaining hops, dropped at 0
4    u16   origSeq   monotonic, minted by the bridge
6    u8    type      0xE0 STATE or 0xE1 STATS
7    u8    len       payload length
8..  ...   payload   the LoRa packet body, from offset 3 onward
+2   u16   crc16     CCITT over everything before it
```

8 bytes of header, 2 of CRC, and the LoRa body with its `tag`/`txId`/`seq`
stripped - the mesh carries the tag in `type` and has its own identity in
`origSeq`.

| carrying | LoRa bytes | mesh payload | mesh frame |
|---|---|---|---|
| STATE `0xE0` | 8 | 5 | 15 |
| STATS `0xE1` | 24 | 21 | 31 |
| ceiling (`MESH_MAX_PAYLOAD` 32) | - | 32 | 42 |

42 bytes is far inside the 250 an ESP-NOW frame carries, so there is room to
add a field to a LoRa format without this layer being the reason it cannot be
added.

**Why this frame carries a CRC when the LoRa formats do not.** `elev_packet.h`
leans on the SX1262's hardware CRC-16 and spends no bytes on its own. Here the
frame is rewritten at every hop - `hop` is decremented in place - and the
802.11 FCS only ever covers one link, so a frame corrupted in a relay's RAM
goes back out with a perfectly valid FCS and nobody notices. The CCITT CRC
travels end to end. It also catches the unrelated ESP-NOW device whose frame
happens to open with the right two bytes.

Remember for §10: the CRC-16 in this envelope is the *second* integrity check a
frame passes. Anything mangled in flight is already gone, dropped by the 802.11
FCS before ESP-NOW ever hands it up.

---

## 2. Why a flood and not a 5 -> 6 -> 7 chain

The obvious design is a chain: floor 5 hands to 6, 6 to 7, 7 to 8. It is
cheaper on the air and it is trivially easy to reason about. It was not chosen,
for two reasons.

**One unplugged display kills everything past it.** A chain has exactly one
path. Floor 7's display goes to a repair, or someone needs the outlet for a
vacuum cleaner, and floors 8, 9 and 10 go dark - not stale, dark, and they stay
that way until someone notices and walks up. A flood has as many paths as the
RF allows. If floor 5 can reach floor 7 directly, which through two floors of
concrete it plausibly can, losing floor 6 costs nothing at all.

**A chain has to know who its neighbours are.** That means a per-floor build,
or a per-floor configuration step, or a MAC address written on a sticker. Swap
two displays during a mount change and the chain is quietly wrong in a way
nothing reports. The flood broadcasts to `ff:ff:ff:ff:ff:ff` and every node
runs identical code from an identical binary, so a display carries no identity
at all and a spare is a drop-in.

### What the flood actually costs

Not nothing, and it is worth being precise about where the cost lands.

*Transmissions:* roughly a wash. A flood is 11 transmissions per frame - the
bridge plus ten displays, each relaying once. A chain covering 5 -> 10 and
5 -> 1 is 8, since floors 1 and 10 are endpoints and relay to nobody. In A and
C, with a basement display on the bottom of the chain, the same two counts are
12 and 9. Flooding a line is not the blowup it is in a dense mesh, because a
line has no fan-out to amplify.

*Receptions:* this is where it lands. Each node hears the same frame from every
neighbour in range, times `MESH_REPEATS`, and throws all but the first away.
A node with three neighbours in range processes roughly ten copies of every
frame. Each copy costs a slot in the receive ring and a linear scan of a
32-entry dedup array - cheap, but it is the reason `MESH_RX_QUEUE_DEPTH` is 12
and not 3, and it is the reason nothing is done in the receive callback
(§5).

*Channel time:* 11 nodes x 3 repeats = 33 transmissions per STATE frame, one
STATE every 2 s while the car moves. `espnow_mesh.h` puts a 42-byte frame at
about 1.3 ms of payload time at the LR rate, which makes the floor of the
estimate ~43 ms per 2 s, around 2% occupancy. A and C carry one more node:
12 x 3 = 36 transmissions, ~47 ms, around 2.3%. That is a floor, not a
measurement: it ignores the 802.11 preamble, the MAC header and CSMA backoff,
all of which are substantial at 250 kbps. Nobody has put a spectrum analyser on
this.

*No delivery guarantee, and no way to ask.* Broadcast frames are not
acknowledged. The bridge cannot tell you whether floor 9 got the frame. That is
not a flaw of flooding specifically, but it is why §10 is half this document -
the counters on each node are the only visibility there is.

### The one honest failure of "routes around it"

A flood routes around a dead node only if some other node has a path. In a
stairwell the range is roughly a small number of floors, and nobody here has
measured how many. If floor 5 reaches 3 and 7 directly, losing 4 is free.
Losing both 3 **and** 4 cuts the building in half regardless of what the
protocol does. §10.6 is how to find out what the actual range is before
assuming.

---

## 3. The rules, and where each number lives

All but one of these are in `[mesh_base]` in `platformio.ini`, which
`bridge_rx_{a,b,c}` and `floor_display_{a,b,c}` all inherit - one edit, but
thirty-five reflashes.

The exception is `MESH_CHANNEL`, which is now a per-elevator value and lives in
`[elev_a]`, `[elev_b]` and `[elev_c]` instead. `[mesh_base]` does not define it
at all, so it is defined in exactly one place per build and a firmware built
with no elevator selected fails to compile rather than quietly inheriting
elevator B's channel.

| flag | value | what it does |
|---|---|---|
| `MESH_CHANNEL` | A 6, B 1, C 11 | the channel the raw frames go out on, per elevator (§13). Nothing associates with an AP |
| `MESH_TX_POWER_QDBM` | 84 | quarter-dBm. `esp_wifi_set_max_tx_power` takes [8, 84] and quantises; 84 lands on the 80 (20 dBm) ceiling |
| `MESH_HOP_LIMIT` | 8 | §7 |
| `MESH_REPEATS` | 3 | §8 |
| `MESH_RELAY_JITTER_MIN_MS` | 5 | §5 |
| `MESH_RELAY_JITTER_MAX_MS` | 40 | §5 |
| `MESH_DEDUP_RING` | 32 | §4 |

Every other value in that table is identical on all thirty-five boards in the
building. The channel is the only thing that distinguishes one shaft's mesh
from another's.

And these are in the headers, not the build:

| constant | value | where |
|---|---|---|
| `MESH_PHY_RATE` | `WIFI_PHY_RATE_LORA_250K` | `espnow_mesh.h` |
| `MESH_RX_QUEUE_DEPTH` | 12 | `espnow_mesh.h` |
| `MESH_RELAY_SLOTS` | 8 | `espnow_mesh.h` |
| `MESH_MAX_PAYLOAD` | 32 | `mesh_packet.h` |
| `MESH_DEDUP_DEPTH` | 32 | `mesh_packet.h` |

`MESH_DEDUP_RING` (the build flag) and `MESH_DEDUP_DEPTH` (the header constant)
are the same window under two names, for the dull reason that the header
predates the flag and is host-testable on its own. `espnow_mesh.cpp`
`static_assert`s that they agree, so editing one and not the other is a compile
error rather than a mesh that dedups over a window the documentation does not
describe.

The only asymmetry anywhere is `MeshRole`. Both roles relay. Only
`MESH_ROLE_ORIGIN` - the bridge - may mint `origSeq`, because a second minter
in the same u16 space produces collisions that the dedup ring reads as
duplicates and swallows without a word.

---

## 4. The dedup ring, and the wrap that would have bitten once

Every node keeps 32 `origSeq` values in a ring, oldest evicted first. A frame
is relayed if and only if its `origSeq` is not among them. That is the whole
rule; `meshDedupSeenOrInsert()` in `mesh_packet.h` is three lines.

**Membership, never comparison.** The rule you would write first is "relay only
if this `origSeq` is newer than the highest I have seen". It is smaller, it is
O(1), and it is a trap.

`origSeq` is u16. At one STATE every 2 s the bridge reaches 65535 in about a
day and a half, and then goes back to 0. Under the comparison rule, every frame
after the wrap looks older than the stored high-water mark, so every node
refuses to relay it. The mesh stops dead. It does not recover when the next
frame arrives, or the next hundred - it recovers when the counter climbs all
the way back past 65534, which is another full cycle, another day and a half.

Think about what that looks like from the corridor. Every display freezes at
once. Nobody is around at 4 a.m. By the time anyone looks it is either still
broken with no obvious cause or it has fixed itself. Then it does not happen
again for a day and a half, which is long enough that it never gets correlated
with anything. This is the class of bug that gets written off as "it does that
sometimes".

The ring tests equality, so the wrap is not an event it can observe. 65535 and
0 are two values and neither is greater than the other as far as this code is
concerned. **Nothing in `mesh_packet.h` compares two `origSeq` values with
`<` or `>`, and nothing added to it should.**

**Why the ring carries a `count`.** A freshly `memset` ring is 32 zeroed slots.
Without `count`, membership would find `origSeq` 0 already present and silently
drop the first frame after every reboot - once per display, at exactly the
moment somebody is watching it to see if it came up. `count` grows with the
live entries and saturates at 32. For the same reason the bridge mints from 1,
not 0, and `meshLastOrigSeq()` returning 0 means "nothing yet".

**Why 32.** At the 2 s STATE cadence that is roughly a minute of traffic. The
longest a frame can plausibly still be in flight is a handful of relay jitters
- a few hundred milliseconds - so 32 is two orders of magnitude of slack
against a late duplicate being relayed a second time. In the other direction it
is short enough to stay a linear scan over one cache line's worth of u16, which
is what lets the scan sit in the hot path without anyone thinking about it. The
only thing that would make 32 too small is the bridge minting very much faster
than it does - or a second bridge minting into the same ring, which is §13.1
and is a correctness problem rather than a sizing one.

---

## 5. Relay jitter

Ten displays - eleven in A and C - hear the same frame within microseconds of
each other. If each relays as soon as it has decoded it, all ten transmit into
the same air at the same instant. CSMA does not save you: each radio measures a channel that was
idle a moment ago, so they do not back off from *each other*, they all go at
once. Every listener gets ten overlapping frames and decodes none of them. The
flood dies one hop from the bridge, which is exactly where it was needed.

`MESH_RELAY_JITTER_MIN_MS..MAX_MS` is a uniform random wait of 5 to 40 ms
before a relay goes out. 35 ms of spread against the ~1.3 ms a frame occupies
means the ten transmits queue rather than collide. This is not a tuning
refinement that makes the mesh better. It is the difference between a mesh and
a pile of ESP32s shouting. §13.2 runs the same arithmetic for 35 nodes sharing
one window, which is why the three shafts are three meshes.

**Why not a fixed per-floor delay?** Floor *n* waits *n* x 4 ms. Deterministic,
no RNG, provably collision-free, and it reads as the tidier design.

It would work until the day two displays are swapped. Now two nodes hold the
same delay slot and collide on every single frame, forever, and nothing
anywhere reports it - the frames stop arriving on the floors past them.
Worse, a per-floor delay is a per-floor *build*, which throws away the property
that makes the flood worth having: that a display carries no identity and a
spare is a drop-in. A random draw needs no configuration, cannot be
mis-assigned, and degrades gracefully - two nodes that happen to draw the same
millisecond collide once and draw again next frame.

**What it costs.** Up to 40 ms per hop of added latency, average 22.5 ms. On
the 5-hop worst case that is up to 200 ms, typically around 110 ms. Against a
2 s STATE cadence and a display whose staleness threshold is 10 s, that is
invisible.

**And it is why nothing is done in the receive callback.**
`esp_now_register_recv_cb` installs a callback on the WiFi task, not on
`loop()`. A CRC over the frame, a `Serial.printf`, a TFT write, and above all a
40 ms jitter wait, all stall the stack that is trying to hand over the next
frame - and frames dropped inside the driver during that stall are dropped
where this layer cannot even count them. The callback does one thing: `memcpy`
into a ring and return. Every decision happens in `meshService()`, from
`loop()`. That is also what lets the counters be read without a lock.

---

## 6. WIFI_PROTOCOL_LR

Espressif's long-range PHY is worth roughly 7 dB of sensitivity over 802.11b.
It is proprietary - only ESP32s speak it - and every node in this system is an
ESP32, so the usual reason not to use it does not apply here. Floor-to-floor
through reinforced concrete is exactly the marginal link that 7 dB is for.

`MESH_PHY_RATE` is `WIFI_PHY_RATE_LORA_250K`, the more sensitive of the two LR
rates. At 42 bytes the airtime difference against the 500K rate is under a
millisecond, which nothing in this system can measure, so there is no reason to
take the faster one.

### The catch, and it is a real one

**Setting `WIFI_PROTOCOL_LR` alone does not change what ESP-NOW transmits at.**
The protocol bitmask tells the PHY what it is *allowed* to do. ESP-NOW picks
its own transmit rate per peer, and unless that is set too, your frames go out
at an ordinary 802.11b rate and you have bought exactly 0 dB while convincing
yourself you bought 7.

The call that sets it differs between IDF versions:

| IDF | call |
|---|---|
| v4.x | `esp_wifi_config_espnow_rate(WIFI_IF_STA, MESH_PHY_RATE)` |
| v5.x | `esp_now_set_peer_rate_config(peer_addr, &rate_cfg)`, per peer |

`esp_wifi_config_espnow_rate` is deprecated in v5 and eventually removed.
`espnow_mesh.cpp` uses the v5 form only: one `esp_now_set_peer_rate_config()`
against the broadcast peer, immediately after `esp_now_add_peer()`, with
`phymode = WIFI_PHY_MODE_LR` and `rate = MESH_PHY_RATE`. It halts on failure
rather than carrying on at a legacy rate. There is no version fallback in it,
so if the `espressif32` platform ever moves to an IDF where that name is gone -
or back to one where it does not yet exist - the build breaks on that line. The
fix is the other call in the table, not deleting the call, which compiles fine
and quietly costs you the whole 7 dB.

### Three more things LR does to you

- **A laptop cannot see the traffic.** LR is not 802.11b/g/n. Wireshark in
  monitor mode, a phone WiFi scanner, and every other conventional tool see
  nothing at all. There is no packet capture available for this link. The
  per-node counters in §10 are the only instrumentation that exists, which is
  why there are sixteen of them.
- **A half-deployed change makes nodes deaf in one direction.** If some boards
  have the rate set and some do not, the ones transmitting at LR are inaudible
  to the ones that are not - and possibly not vice versa. Symptoms look like a
  range problem. Reflash every node in the shaft or none - eleven in B, twelve
  in A and C.
- **7 dB is a link budget number, not a floor count.** How many extra floors it
  buys depends on the per-floor attenuation of this particular stairwell, which
  is unmeasured. 6 dB is a factor of two in free-space range; through
  structural concrete it may be well under one extra floor. Do not plan the
  layout around it - measure it (§10.6).

Transmit power is `MESH_TX_POWER_QDBM = 84`, which the PHY quantises down to
the 80 quarter-dBm (20 dBm) hardware ceiling. `meshBringUp()` reads the value
back and reports what was actually granted rather than what was asked for,
because the ceiling also depends on the calibration data flashed into the
module, and the XIAO ESP32S3 and the ESP32-WROOM-32 are not the same module.
Check the banner; do not assume.

---

## 7. Hop limit 8

In elevator B the bridge is on floor 5. Floor 10 is five landings above it and
floor 1 is four below, so the worst case, in the pessimistic assumption that
each hop only ever reaches the adjacent floor, is **5 hops**. `MESH_HOP_LIMIT`
is 8.

**Eleven landings do not change that, and it is worth showing the arithmetic
rather than asserting it.** A and C add a basement below floor 1 and keep the
bridge mid-shaft on floor 5. The new landing extends the shaft in the direction
that had a hop to spare: 5 -> 4 -> 3 -> 2 -> 1 -> B is five hops, exactly
matching the five up to floor 10. The worst case is 5 hops in all three shafts
and the three hops of slack survive intact. It is the asymmetry of the original
placement that pays for the basement - a bridge one floor lower would put the
basement four hops away and floor 10 six, and the slack would be 2.

What the basement does change is the quality of the pessimism, not the count.
A basement is below grade and under the lobby slab, and floor-to-floor
attenuation there is not the same as between two upper floors. If nothing above
floor 1 can cross that slab, the basement display is reachable by exactly one
path in a design whose argument is that there are several (§2), and losing the
floor 1 display takes it dark. That is a range fact to measure (§10.6), and not
one a larger hop limit reaches: `dropHopExhausted` climbing on a basement
display would mean the opposite, that frames are arriving there by some path far
longer than the shaft.

The frame is minted at 8 and each relay decrements before sending. A copy
arriving with 1 hop left is delivered to that node and goes no further:
`meshDecrementHop()` returns false when the decrement leaves 0, and the node
counts `dropHopExhausted`. So the delivery radius is 8 hops from the bridge.

The three hops of slack are for paths that are not straight lines:

- A detour. Floor 6 is unplugged, so the frame goes 5 -> 7 -> 8 the long way
  round via whatever node happened to hear it, and the path is longer than the
  floor count suggests.
- A path that dips and climbs. RF through a stairwell does not respect floor
  numbering, and the first copy to arrive somewhere is not always the one that
  took the fewest hops.
- Range that turns out worse than assumed, so the "each hop is one floor"
  pessimism is not pessimistic enough.

There is no reason to raise it. A frame that has taken 8 hops in a shaft of ten
or eleven landings has gone somewhere strange, and letting it go further only
adds channel time. There is also little reason to lower it: `hop` costs a byte
whatever its value, and the flood is bounded by the dedup ring anyway - every
node relays each `origSeq` exactly once, so the total transmission count is 11,
or 12 in A and C, regardless of the hop limit. The hop limit is a backstop
against a frame circulating, not the thing that bounds the flood.

**One consequence worth knowing.** A node relays the *first* copy it hears and
ignores the rest, including a copy that arrives later with more hops remaining.
So the hop count a frame carries onward is whichever path got there first, not
the shortest. In practice those are usually the same thing, because a shorter
path has accumulated fewer relay jitters, but it is not guaranteed. If you ever
see `dropHopExhausted` climbing on a display five floors from the bridge, this
is why, and it means the path is longer than the map suggests.

---

## 8. Three sends

`MESH_REPEATS = 3`: every frame goes out three times back to back, both when
the bridge mints it and when a node relays it.

**Why.** A flood has no delivery guarantee and broadcast frames are not
acknowledged - there is no retry, because there is nothing to retry *on*. A
single collision or a single fade and the frame is gone from that link.
Three copies plus multiple paths make loss unlikely. They do not make it
impossible, and the system is built to admit that: spec §9 lists "a display
showing a stale floor" as the expected failure mode, which is why staleness is
displayed rather than hidden.

**What it costs.** Channel time, linearly. The ~2% occupancy estimate in §2
already has the three repeats in it; at one send each it would be around 0.7%.
Everything past the first copy to arrive is a dedup hit at every receiver,
costing a receive-ring slot and a 32-entry scan. It also raises the collision
probability for everyone else on this shaft's channel, which is the trade being
made in §11.

**Why not more.** Repeats are the cheapest knob to reach for when delivery is
short, and also the one most likely to make things worse - past some point you
are colliding with your own retransmissions and with the fresh transmissions of
every other node in the shaft. If 3 is not enough, the problem is almost
certainly range, antenna placement or channel congestion, and those are worth
ruling out first.

---

## 9. Tuning, in the order worth trying

Nothing here changes without reflashing every board in the shaft - eleven in B,
twelve in A and C. `[mesh_base]` is inherited by `bridge_rx_{a,b,c}` and
`floor_display_{a,b,c}`, so an edit there is one line and thirty-five
`pio run -t upload`s. `MESH_CHANNEL` is the one row below that can be changed
for a single shaft (§11). A node flashed with a different
`MESH_CHANNEL` than the rest of its shaft is completely deaf to them and
reports nothing unusual about itself.

| if | try | cost |
|---|---|---|
| one display marginal | move the board; rotate it; get it off the metal doorframe | free |
| this shaft's channel is busy (§11) | move this shaft's `MESH_CHANNEL`; with 1, 6 and 11 all in use that means swapping with another shaft | one or two shafts reflashed, no runtime cost |
| delivery short everywhere | `MESH_REPEATS` 3 -> 5 | +67% channel time, more self-collision |
| relay collisions suspected | `MESH_RELAY_JITTER_MAX_MS` 40 -> 60 | +20 ms latency per hop |
| mint rate rises a lot | `MESH_DEDUP_RING` 32 -> 64 | longer scan, still trivial |

Not worth turning: `MESH_TX_POWER_QDBM` is already pinned to the hardware
ceiling. `MESH_HOP_LIMIT` is a backstop, not a throughput knob (§7).
`MESH_PHY_RATE` at 500K would halve an airtime nobody can measure in exchange
for sensitivity this link is short of.

---

## 10. Debugging in the stairwell

### 10.1 Getting a console

Every display is a CYD on a USB-C supply. Unplug the supply, plug the board
into the laptop, and it powers up on USB with the console on the same cable:

```
pio device monitor -e floor_display_a        # 115200; _b, _c for the others
```

The bridge is a XIAO ESP32S3, same idea, USB-C on the native port:

```
pio device monitor -e bridge_rx_a            # _b, _c for the others
```

There is no `floor_display` or `bridge_rx` environment to monitor: identity is a
build flag (§13), so every environment carries its shaft's letter and the
unsuffixed names fail.

`monitor_dtr = 0` and `monitor_rts = 0` are already set for both XIAO
environments. Those lines are wired to the reset and boot0 logic on that port -
the same ones esptool pulses - so a monitor that asserts them on open, which
most do, holds the board in reset and you get a silent console on a port that
looks perfectly healthy. If you use some other terminal program on the XIAO
nodes, turn DTR and RTS off in it.

`meshPrintCounters()` puts one line of everything in §10.3 on the console.
`meshBringUp()` prints a banner at boot with the role and the transmit power
the PHY actually granted.

### 10.2 What quiet looks like

**Before deciding the mesh is broken, check whether anything is being sent.**

STATE goes out every 2 s only while the car is moving, plus 10 s after it stops
(`STATE_HOLD_AFTER_STOP_MS`). STATS goes out every 60 s regardless. So an
elevator nobody is riding produces **one frame per minute**, and at 2 a.m. a
mesh carrying one frame a minute is a healthy mesh.

If you are debugging at a quiet hour, ride the elevator. Two round trips is
around a hundred STATE frames, which is enough to compute a delivery ratio from.

### 10.3 The counters

From `MeshCounters` in `espnow_mesh.h`. All written only from `loop()`, so they
can be read without a lock, and all reset on reboot - note the starting values
and work in differences.

| counter | meaning |
|---|---|
| `heard` | frames the WiFi task handed us, before any validation |
| `accepted` | decoded, and new to this node |
| `deduped` | decoded, but this `origSeq` was already in the ring |
| `relayed` | put back on the air after the jitter wait |
| `published` | origin only: frames minted here |
| `sends` | `esp_now_send` calls the driver accepted |
| `sendFails` | `esp_now_send` calls it did not |

Drops, by reason. The first five mirror `MeshDecodeStatus`:

| counter | meaning |
|---|---|
| `dropShort` | too small to be a frame at all |
| `dropMagic` | first two bytes are not `0x5E1E` |
| `dropVersion` | a node running a different build of this format |
| `dropLength` | `len` disagrees with the frame, or overruns the payload |
| `dropCrc` | CRC mismatch |
| `dropHopExhausted` | reached its hop limit here and goes no further |
| `dropRxQueueFull` | `loop()` did not drain the receive ring fast enough |
| `dropRelayFull` | more distinct frames in flight than `MESH_RELAY_SLOTS` |
| `dropOversize` | `meshPublishLora()` was asked to flood something this node may not - the wrong role, or a body too big for the envelope. Should never move on a display |

**How to read them.**

- `heard` is the top of the funnel. Everything in it should end up in
  `accepted`, `deduped`, or one of the drop counters. If the arithmetic does
  not close, that is a bug in `espnow_mesh.cpp`, not something to chase in the
  stairwell.
- **`deduped` much larger than `accepted` is healthy**, not a problem. It is
  the flood working: this node is hearing the same frame from several
  neighbours and from three repeats each, and keeping one. A node with
  `deduped` around ten times `accepted` has several good paths to the bridge.
  A node with `deduped` near zero and `accepted` healthy has exactly one path
  and no redundancy - it works today and it is one unplugged neighbour from
  going dark.
- `accepted` should be roughly `relayed`. Frames accepted but not relayed are
  `dropHopExhausted` (this node is at the edge of the radius) or
  `dropRelayFull` (more distinct frames inside a 40 ms window than there are
  slots, which at the 2 s cadence should be zero).
- `sends` should be about three times `relayed` (plus `published` on the
  bridge), because each frame goes out `MESH_REPEATS` times back to back. A
  ratio well under 3 means sends are being refused - check `sendFails`.
- `sendFails` climbing means the driver is refusing the frame, not that the air
  is busy. A busy channel shows up as backoff inside the driver, not here.
- `dropRxQueueFull` non-zero means `loop()` is being starved - usually a long
  blocking draw or a blocking NVS write somewhere in the main loop, not a mesh
  problem.

### 10.4 Which layer is broken

Start at the bridge. The cases separate cleanly.

| symptom | layer | what to do |
|---|---|---|
| bridge `published` flat, every display stale | **LoRa is down** | the mesh is fine and idle. Go to the car node |
| bridge `published` climbing, every display `heard` = 0 | **mesh is broken at the bridge** | see below |
| bridge `published` climbing, most displays fine, one stale | **that one display** | §10.5 |
| a contiguous run of floors stale, the rest fine | **a cut, not a node** | the boundary floor is the failed relay; §2 |

**`published` flat** means the bridge is not receiving LoRa. Nothing about the
mesh is implicated. Check that the car node is alive and its battery is not
flat, and remember ALGORITHM.md §9: floor 9 is the one landing whose SNR
crosses the SF10 decode floor, an intermittent shadow, so a car parked there
legitimately goes quiet. Bridge antenna placement should favour the upper
shaft.

**Every display deaf while `published` climbs** is nearly always one of three
things, in order of likelihood:

1. The bridge was flashed with a different `MESH_CHANNEL` than the displays, or
   vice versa. Check the boot banners against each other. The bridge's
   `[bridge] elevator 'B' (txId 0x..): LoRa 915.0 MHz, mesh ch1`
   (`bridge_rx.cpp`) names both radios because it has both. A display has no
   LoRa radio and prints no frequency: its `[ui] elevator 'B': ...`
   (`floor_display.cpp`) names the shaft and its label table, and the
   `[mesh] channel N` line from `meshBringUp()` - which both node types print -
   is where its channel appears. With three elevators the usual form of this
   mistake is a board built for the wrong one (§13.3).
2. The LR rate did not get set on the bridge - see §6. Symptom: the bridge
   transmits, the displays are on the right channel, and nobody hears anything.
3. `meshBringUp()` failed. It halts with an explanation on Serial rather than
   limping, so the console will say so.

**A contiguous run of stale floors** is the interesting case and it is *not* a
protocol failure. Floors 6-10 fine and 1-4 stale means the flood cannot cross
between floor 5 and floor 4, and no node on either side has a path around it.
Either the floor 4 display is dead, or the range is short enough that 5 cannot
reach 3. That is §10.6.

### 10.5 A display that never relays

`relayed` is stuck at 0 on one floor. Work down this list; each step
distinguishes the next.

**1. Is it alive?** Console banner, screen lit. If there is no banner, it is a
power or boot problem and nothing below applies. If the screen is on and the
console silent, check the baud and the cable.

Do not read the screen as evidence at this step. `floor_display.cpp` calls
`displayUpdate()` only once a frame has arrived, so a board that has heard
nothing since boot is still showing the `Simmevator / waiting for the mesh`
splash. That is what every fault below looks like from the corridor, and it is
also what a board looks like thirty seconds after a power cut. The counters,
not the glass, separate them.

**2. Is `heard` moving?**

*`heard` = 0.* It is receiving nothing at all. Three causes:

- Wrong channel. It was built for another elevator (§13.3), or flashed from a
  tree with a different `MESH_CHANNEL`, or never reflashed when the channel was
  changed. Check its banner against a working display's.
- Its LR rate is not set and the others' is, or the reverse - the half-deployed
  case in §6. Same symptom as wrong channel.
- **Genuinely out of range of both neighbours.** This is the one thing on the
  list that is a real RF fact rather than a flashing mistake. Confirm it by
  carrying the board up or down one floor with a USB power bank and watching
  `heard`. If it starts counting a floor away, the node is fine and the
  *mount* is the problem. Move it off the metal doorframe, rotate it, get it
  away from the lift machinery.

*`heard` climbing, `accepted` = 0.* It is hearing frames and discarding all of
them. The drop breakdown says which:

| dominant drop | meaning |
|---|---|
| `dropMagic` | it is hearing somebody else's ESP-NOW traffic, not ours. Harmless in itself - but if it is hearing *only* that, it is still out of range of the mesh |
| `dropVersion` | a node in the building is running an older build of the frame format. Reflash it |
| `dropCrc` | see below - almost never interference |
| `dropShort` / `dropLength` | a truncated or padded frame. If this is not zero, something is malformed at the source |

**`dropCrc` climbing is not an interference symptom.** A frame mangled on the
air fails the 802.11 FCS and is discarded by the driver before ESP-NOW ever
sees it - it never reaches `heard`. A frame reaching the CRC check has passed
the FCS on its last link. So a rising `dropCrc` means one of:

- A relay somewhere decremented `hop` and did not rewrite the CRC. The CRC
  covers the header and the header contains the field just changed, so that
  relay's frames are unreadable to everyone downstream. Classic symptom: the
  mesh works one hop from the bridge and nowhere else.
- Corruption in a relay's RAM between receive and retransmit, which the fresh
  FCS then certifies as good.
- An unrelated ESP-NOW device whose frame happens to open with `0x1E 0x5E` and
  is the right length. Rare, and it will be a trickle rather than a rising
  fraction.

The first is a firmware bug and is the reason to check it first.

*`heard` and `accepted` both climbing, `relayed` = 0.* The node is doing
everything except transmitting. Check `sendFails` and `dropHopExhausted`. If
`dropHopExhausted` is the whole of it, the frames arriving here have already
taken 8 hops, which in this building means something is much further away than
the floor map suggests.

**3. It relays, but nobody hears it.** `relayed` climbing on this node, and the
neighbouring floors' `heard` not moving in step. Now it is the transmit side:
check the granted transmit power in its boot banner against a working display's
- the two board types do not necessarily grant the same - and check the mount
and orientation again.

### 10.6 Measuring delivery, and measuring range

**Per-display delivery ratio.** The only number that actually answers "is the
mesh good enough".

1. Note `published` on the bridge and `accepted` on the display under test.
2. Ride the elevator for ten minutes.
3. Note both again.
4. `delta accepted / delta published` is that display's delivery ratio.

Two caveats. `published` counts what the bridge minted, including STATS, so
compare over the same wall-clock window on both nodes. And `accepted` counts
frames new to that node, which is exactly "frames delivered here" - it is the
right numerator.

**Who is falling behind, quickly.** `meshLastOrigSeq()` on a relay is the last
`origSeq` it accepted. Read it on two displays within a few seconds of each
other: they should be within a handful of each other. A display 400 behind has
missed 400 frames and is not a marginal case, it is a broken one.

**Actual floor-to-floor range.** This is worth an hour, once, because several
assumptions above rest on it and none of them is measured.

Put a display on a USB power bank. Stand it on floor 5 next to the bridge and
confirm `heard` climbs. Walk it up one floor at a time, waiting long enough at
each for several frames, and record `heard` per minute at each floor. Where it
stops counting is the bridge's single-hop reach. Repeat downward. Then repeat
from a display on floor 8 to find a display-to-display reach, which will differ
- the bridge and the CYDs have different antennas and different granted power.

If the callback on this IDF version carries radio metadata, the `rssi` argument
to the frame handler is the frame as this node heard it and turns that survey
into numbers rather than a yes/no. **If it does not, `rssi` is 0** - that is
documented in `espnow_mesh.h`, and a screen full of zeroes is the API telling
you it has nothing, not a signal of -0 dBm.

The result tells you whether "floor 5 can still reach floor 7" is true in this
building, which is the entire premise of §2.

### 10.7 The foreign-origin warning, and what it is good for

The bridge registers a mesh receive handler that does nothing but complain.
Every frame it mints comes back to it from its own displays and is swallowed by
its dedup ring, so anything that survives to the handler carries an `origSeq`
this bridge did not mint. `onMeshFrame()` in `bridge_rx.cpp` counts it in
`foreignOrigins` and prints the `origSeq`, the type, the source MAC, the RSSI
and the channel, with the reason attached: *a second origin on channel N would
collide in the origSeq space*. A non-zero `foreignOrigins` also appears in the
bridge's periodic summary line.

That guard was written for the single-elevator system, where the only thing it
could plausibly catch was a forgotten bench node. **With three bridges in one
building it is the detector for the one configuration mistake that §13.1 says
is otherwise invisible: two shafts sharing a channel.** Nothing else in the
system reports that condition. The displays do not - from a display's side the
symptom is `deduped` rising and `accepted` falling, and §10.3 documents a high
`deduped` as the flood working properly.

It is bridge-local, not part of `MeshCounters`, so it is read on the bridge
console and nowhere else.

**And that is the limit of it: `foreignOrigins` is a bench instrument, not a
field instrument.** The bridge is a wall-powered box mid-shaft; reading its
console means standing in front of it with a laptop, which is not a thing that
happens during normal operation. It earns its place on the bench, with the three
shafts' transmitters and bridges powered at once, where "no `foreignOrigins`
field on any of the three summary lines" is a pass condition that is sharper
than anything the building produces.

Read that as an absence, not as a zero. `printSummary()` in `bridge_rx.cpp`
wraps the field in `if (foreignOrigins)`, so a clean bridge prints no
`foreignOrigins=` at all; somebody told to look for a 0 will hunt for a field
that is never there and report the test as inconclusive.

**Keep at least a metre between any transmitter and any bridge on that bench.**
The transmitters run at 22 dBm and the SX1262's absolute maximum RF input is
around +10 dBm (`platformio.ini` next to `LORA_TX_POWER=22`, tabulated in
HARDWARE.md §3.1). Free space at 915 MHz puts +10.3 dBm into a receiver at
0.10 m - over the absolute maximum, with the crossover at 0.104 m - against
-9.7 dBm at 1.00 m, which is about 20 dB of margin. A bench laid out as "all
six boards on one table" ends up with boards 5-10 cm apart, and what that
damages is a bridge's front end, quietly, for later.

In the field the argument that the shafts do not interfere is the channel
separation itself, and the instruments that are actually visible are the
displays.

---

## 11. Interference, per shaft

Channel 1 is 2401-2423 MHz and it is the most popular channel in most
buildings, including this one. That is elevator B's channel; A is on 6 and C on
11 (§13), which are usually less crowded and neither of which is empty.
Nothing here associates with an access point, so the frames go out regardless
of what else is on the channel - but they collide with it.

### Telling interference apart from everything else

Interference does **not** show up as CRC errors. §10.5 is the reason: a
corrupted frame dies at the 802.11 FCS inside the driver, before this layer
counts it anywhere. A node cannot count what it never heard.

Interference shows up as **frames that never arrive at all**:

- `heard` on a display drops without anything about it changing.
- The delivery ratio in §10.6 falls, uniformly across displays rather than on
  one.
- It varies with time of day - worse in the evening, better at 4 a.m. That is
  the strongest single tell, because RF geometry does not change on a schedule
  and a lecture hall full of laptops does.
- `sendFails` stays flat. A busy channel makes the driver back off, not refuse.

### The survey

You cannot see LR traffic with any conventional tool (§6), but you can see
*everybody else's* traffic, and that is the question:

1. Stand in the stairwell with a phone WiFi analyser.
2. Count access points and signal strength on channels 1, 6 and 11.
3. Do it at the worst hour, not a convenient one.

**Do this in each shaft separately.** Congestion is a property of the stairwell
rather than of the building - the APs that matter are the ones within a few
floors of that shaft, and three shafts far enough apart to be given separate
meshes are far enough apart to have different neighbours. A survey taken in
shaft B says nothing about shaft C.

If this shaft's channel is carrying several strong APs and another of the three
is comparatively empty there, the change is worth making.

### Making the change

Because `MESH_CHANNEL` is a per-elevator build flag (§3), **any one shaft can
move channel on its own** - a degree of freedom the single-elevator system did
not have. Change the value in that elevator's section of `platformio.ini` and
reflash that shaft's boards, eleven in B or twelve in A and C. The other two
shafts are untouched and never notice.

**The one rule is that two shafts within earshot must not share a channel.**
Sharing puts two bridges' `origSeq` into one dedup space, which is §13.1, and
that failure is invisible from the corridor. `foreignOrigins` on the bridge
(§10.7) is what catches it, on the bench.

That rule makes a move a swap rather than a migration, because 1, 6 and 11 are
the only three non-overlapping 2.4 GHz channels and all three are in use. If
shaft A's stairwell is thick with APs on 6 while 11 is clear there, the change
is A and C trading channels - and C's stairwell should be surveyed on 6 first,
because that trade solves A's problem by handing C whatever is on 6 near shaft
C.
Picking 3 or 9 to escape into instead buys partial overlap with two neighbours
in place of full overlap with one, which is worse, and it does it to a
neighbouring shaft as well as to the building's access points.

If all three channels are busy in one stairwell, no channel fixes it and the
answer is the first row of §9 - the mount, the orientation, the metal doorframe
- not a build flag.

A node left on the old channel is completely deaf and completely silent about
it - its `heard` sits at 0 and it looks exactly like a range problem. There is
no channel negotiation, no scan, and no fallback; agreement on the channel is
an assumption baked into the design, and it is the first thing to check
whenever a node has gone inexplicably quiet. With three channels in the
building there is now a second thing to check alongside it: that the node is on
its own shaft's channel and not a neighbour's (§13.3).

---

## 12. What is not measured

Stated plainly, because several of the numbers above are derivations and one
document reading like all of them are measurements is how a derivation becomes
a fact.

- **Channel occupancy.** The ~2% in §2 is arithmetic from a payload-time figure
  in `espnow_mesh.h`. It ignores the 802.11 preamble, the MAC header and CSMA
  backoff, all significant at 250 kbps. Nobody has put an analyser on it.
- **Floor-to-floor range.** Unknown. "Floor 5 may still reach floor 7", which
  is the argument for flooding over a chain, is plausible and untested.
  §10.6 is how to test it.
- **What 7 dB buys in this stairwell.** The 7 dB is Espressif's figure for the
  LR PHY. Its translation into floors depends on per-floor attenuation here,
  which nobody has measured.
- **Delivery ratio.** No number exists for any display. The procedure in §10.6
  produces one in ten minutes and nothing in this repository has run it.
- **Collision rate at three repeats.** `MESH_REPEATS = 3` is a judgement, not
  an optimum. Whether 2 would do or 5 would help is unknown.
- **Actual mesh latency.** The 110-200 ms in §5 is jitter arithmetic over an
  assumed 5-hop path, with no airtime, no CSMA and no processing time in it.
- **Whether the three shafts can hear each other at all.** §13 assumes they can
  and separates them so that the answer does not matter. Nobody has measured
  cross-shaft audibility in this building. The bench test that backs the
  separation does not settle it either: with the boards spread evenly over one
  bench the near-far ratio is zero by construction, so what it proves is
  adjacent-channel rejection with the interferer at *equal* power to the wanted
  signal. That is a real stress case - in the building a foreign car is
  normally far weaker - but it is not the worst case. The worst case is a
  foreign car close to a bridge whose own car is ten floors away, and a bench
  cannot produce it.

The LoRa leg, by contrast, has real numbers behind it: 5.21% batch loss over
3 h at SF10/BW125/14 dBm, 49 dropouts, worst case 40 s, from ALGORITHM.md §9.
That link was measured in this shaft. This one has not been.

---

## 13. Three meshes in one building

Three shafts, three bridges, 32 displays. Each shaft floods on its own channel
and the three never hear each other:

| | elevator A | elevator B | elevator C |
|---|---|---|---|
| `MESH_CHANNEL` | 6 | 1 | 11 |
| landings | B, 1-10 | 1-10 | B, 1-10 |
| nodes on the channel | 12 | 11 | 12 |

1, 6 and 11 are the three non-overlapping 2.4 GHz channels (§11), so the three
floods do not merely ignore each other's frames, they stay out of each other's
spectrum. Everything else in this file is identical on all thirty-five boards.

The question worth answering here is not why this works. It is why the obvious
alternative does not - one building-wide flood carrying elevator-tagged frames,
on one channel, with `ff:ff:ff:ff:ff:ff` already meaning "everybody". Two of the
numbers this document spends the most time on are the reason, and both of them
fail in the way this system finds hardest to notice: quietly, for hours, with
every counter looking healthy.

### 13.1 One flood cannot carry three origins

§3 disposes of this in a sentence - only `MESH_ROLE_ORIGIN` may mint `origSeq`,
"because a second minter in the same u16 space produces collisions that the
dedup ring reads as duplicates and swallows without a word". Three bridges on
one channel is that sentence, twice over. §4 is why it cannot be repaired at the
receiver.

The ring is **membership, never comparison** (§4). `meshDedupSeenOrInsert()`
asks one question - is this u16 among the last 32 I accepted - and the ring
stores nothing but the value, because in a single-origin mesh there is nothing
else to store. So bridge A's frame 4113 and bridge C's frame 4113 are not two
frames that happen to share a number. To every display on that channel they are
*the same frame*, and the second to arrive is dropped as a duplicate. The
equality test that makes the u16 wrap a non-event (§4) is exactly what makes a
second minter undetectable: the ring cannot tell a repeat from a coincidence,
and it was never asked to.

Work out how often the coincidence happens and it turns out not to be a
coincidence at all. The ring holds the last 32 values a node accepted, from any
origin, so with three floods interleaved it spans about eleven frames from each
- roughly 22 s at the 2 s STATE cadence rather than the minute §4 assumes. A
value C mints is eaten if A minted the same value inside that window, which is
to say if A's counter is ahead of C's by fewer than about eleven. And the three
bridges mint at the same nominal rate by construction, one STATE every 2 s while
their car moves, so their counters do not sweep past each other and move on.
They separate only at the difference between the cars' active duty cycles, and
that is slow: one percentage point of difference at the 0.5 frames per second
cadence is `0.01 x 0.5 x 3600 = 18` frames an hour. Two counters eleven apart
can sit eleven apart all afternoon.

So the failure is not an occasional lost frame. It is: **whenever two of the
three counters are within about eleven of each other, every single frame from
the trailing bridge is discarded by every display in the building, and it stays
that way for as long as the drift keeps them there - hours.** Then it clears on
its own, and comes back when they next converge, and since both counters wrap at
65535 they converge again and again.

Now read that from the corridor. One shaft's displays go stale together for an
afternoon while the other two shafts are fine. The bridge's `published` is
climbing normally. On the displays, `heard` is normal, `accepted` has collapsed
and `deduped` has risen to take up the slack - and §10.3 says in as many words
that a high `deduped` is the flood working properly. The one counter that moves
is the one the debugging section tells you not to worry about.

Fixing it inside the flood means an origin field in the envelope, which means a
`MESH_VERSION` bump and every node in the building reflashed to a new frame
format. The channel plan costs one build flag.

### 13.2 One flood cannot fit in the jitter window

§5 is the other number. The relay jitter is a uniform 5-40 ms draw: 35 ms of
spread against the ~1.3 ms a 42-byte frame occupies at the LR rate, which is
what turns ten simultaneous relays into ten that queue. It is "the difference
between a mesh and a pile of ESP32s shouting", not a refinement.

Count what a building-wide flood puts into that same window. One frame reaching
every node, each node relaying it once, `MESH_REPEATS` copies each:

```
32 displays x 3 repeats        =  96 transmissions
96 x 1.3 ms                    = 125 ms of payload time
125 ms into a 35 ms window     = ~3.5x oversubscribed
```

plus the minting bridge's own three sends and the other two bridges relaying,
which is nine more. Against one shaft today:

```
11 nodes x 3 repeats           =  33 transmissions
33 x 1.3 ms                    =  43 ms into the same 35 ms window = ~1.2x
```

Today's window is therefore already about full, and full is fine: the jitter
spreads the *start* times, CSMA serialises the handful that still overlap, and a
node that has to wait is waiting behind one or two others. At 3.5x the queue
cannot drain inside the window at all. Transmissions land hundreds of
milliseconds after the instant that was drawn for them, which means the draw no
longer determines when anything goes out, which means the jitter has stopped
spreading anything. That is the all-at-once condition §5 exists to prevent,
re-created by the very frames the jitter was protecting. And unlike two displays
that happen to draw the same millisecond, it does not clear on the next frame:
every frame produces the same pile-up.

Both figures are floors, in the sense §12 gives that word - payload time only,
no 802.11 preamble, no MAC header, no backoff. The true numbers are larger,
which makes the 3.5x worse rather than better. The receive path would feel it
too: `MESH_RX_QUEUE_DEPTH` is 12 because a node processes roughly ten copies of
each frame (§2), and three floods on one channel triple that.

### 13.3 What separation buys, and what it costs

On its own channel each shaft is the single-origin, one-shaft system that every
number above was derived for: one minter in the u16 space, ten or eleven
displays sharing the jitter window, a dedup ring spanning a minute of one
bridge's traffic. So nothing in §1 through §12 needs a caveat about the other
two shafts, and the only figures that move are the node counts in A and C,
which carry the ~2% occupancy estimate to ~2.3% (§2). Those are still
derivations rather than measurements (§12); the point is only that three
elevators do not make them any less true than they were for one.

What is given up is cross-shaft path diversity - a display in shaft A can never
be relayed to through shaft C. Since the whole argument for flooding is
floor-to-floor reach inside one stairwell (§2), and the shafts are far apart,
that is not a loss of anything that was wanted.

What is added is a new way to mis-flash a board. §9's warning - a node on the
wrong channel is deaf and silent about it - is now the whole of it, because a
display built for the wrong elevator carries the wrong channel *and* the wrong
label table, and the channel decides first. 1, 6 and 11 do not overlap, so such
a display cannot hear its own landing's bridge at all: `heard` sits at 0, it
never reaches the `if (gSeenAnyFrame)` gate in `floor_display.cpp`, and it
shows the `waiting for the mesh` splash for as long as it is left there.

It does not show a wrong floor, and it does not show CHECK SHAFT. CHECK SHAFT
compares the car's learned `nFloors` against this build's label table, which
takes a frame to arrive, and this board never gets one - which is also why the
wrong label table it is carrying never gets to be read out loud. A CHECK SHAFT
seen in the field is the other fault entirely, and because every display in a
shaft holds the same table and hears the same bridge it appears on all eleven
or twelve at once, never on one.

So the symptom is indistinguishable from a range problem, and the first thing
to check is the boot banner, which names the elevator. Between A and C, whose
label tables are identical, the banner's elevator letter is the only thing that
separates them. The mitigations are outside this file: per-elevator board
labelling and the commissioning states in `FLASHING.md`. From the mesh side,
the instrument is §10.7, and it is a bench instrument.
