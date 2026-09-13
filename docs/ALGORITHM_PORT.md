# The C++ port of the floor algorithm

`src/floor_monitor.h` is a port. The original is
`elevatormons/tools/floor_algorithm.py`, and the measurements that justify every
constant in it are in `elevatormons/docs/ALGORITHM.md`.

This document is the contract between the two: what corresponds to what, the
four places a port like this goes quietly wrong, the hardening the firmware
needs that an offline script does not, and the evidence that the port is
faithful. If you are here because you want to change the algorithm, read
section 7 first.

---

## 1. The rule

**`floor_algorithm.py` is the reference and stays the reference.** It lives in
`elevatormons`, which is the bring-up bench: this project reads from it and
never writes to it. `sim/compare_to_python.py` imports the module from that repo
by path rather than vendoring a copy, because a copy drifts and a drifted golden
reference is worse than no golden reference at all.

Three things follow from that, and they are the whole point of this document.

**A divergence is a bug in the port.** If the C++ and the Python disagree on a
sample, the C++ is wrong until proven otherwise. It does not matter which answer
looks better on the trace. The Python is what was measured against real data, so
"the C++ handles this case more sensibly" means the port has stopped being a
port, and every number in ALGORITHM.md sections 5 and 8 stops applying to the
firmware.

**The tuning constants are not re-tuned here.** ALGORITHM.md section 7 lists
each one with the measurement behind it - `STILL_STD_M = 0.08 m` sits in the gap
between 0.05 m of stop noise and a car doing up to 1.78 m/s; `STILL_WIN = 5`
because the shortest dwell in the capture is 5 s; `CONFIRM_N = 3` because a
floor the car coasted through is travel and not a stop. They are carried over
with their values and their reasons, in the same order, at the top of
`floor_monitor.h`. They are deliberately not `-D` flags in `platformio.ini`:
making them build-time knobs would invite exactly the re-tuning that invalidates
the acceptance gates in section 5 below.

**A change starts in Python.** Section 7 has the procedure.

### Constant mapping

Same values, renamed with a `FLOOR_` prefix to keep them out of a preprocessor
namespace shared with Arduino headers. The trade-off behind each is in
ALGORITHM.md section 7 and repeated in the comment above each `#define`.

| `floor_algorithm.py` | `floor_monitor.h` | value |
|---|---|---|
| `STILL_STD_M` | `FLOOR_STILL_STD_M` | 0.08 m |
| `STILL_WIN` | `FLOOR_STILL_WIN` | 5 |
| `MIN_JUMP_M` | `FLOOR_MIN_JUMP_M` | 1.2 m |
| `MAX_DRIFT_MPS` | `FLOOR_MAX_DRIFT_MPS` | 0.02 m/s |
| `DWELL_SKIP_S` | `FLOOR_DWELL_SKIP_S` | 4.0 s |
| `MIN_DWELL_RATE_S` | `FLOOR_MIN_DWELL_RATE_S` | 30.0 s |
| `RATE_HISTORY` | `FLOOR_RATE_HISTORY` | 7 |
| `REVISIT_MIN_S` | `FLOOR_REVISIT_MIN_S` | 30.0 s |
| `REVISIT_MAX_S` | `FLOOR_REVISIT_MAX_S` | 900.0 s |
| `RATE_TRUST_K` | `FLOOR_RATE_TRUST_K` | 1.0 |
| `DWELL_TAIL_N` | `FLOOR_DWELL_TAIL_N` | 3 |
| `DRIFT_RATE_CAP` | `FLOOR_DRIFT_RATE_CAP` | 0.05 m/s |
| `BOOTSTRAP_JUMPS` | `FLOOR_BOOTSTRAP_JUMPS` | 12 |
| `OFF_LATTICE` | `FLOOR_OFF_LATTICE` | 0.30 |
| `RESYNC_S` | `FLOOR_RESYNC_S` | 25.0 |
| `CONFIRM_N` | `FLOOR_CONFIRM_N` | 3 |
| `alpha` (dataclass field) | `FLOOR_HEIGHT_ALPHA` | 0.25 |
| `0.05` inside `_refine_pitch` | `FLOOR_PITCH_GAIN` | 0.05 |
| `lo`/`hi`/`step` of `estimate_pitch` | `FLOOR_PITCH_LO` / `_HI` / `_STEP` | 2.2 / 4.0 / 5e-4 m |

---

## 2. Function-by-function correspondence

Names and body order match on purpose, so the two files can be put side by side
and read. If you add a function to one, add it to the other in the same place.

| `floor_algorithm.py` | `floor_monitor.h` | note |
|---|---|---|
| `estimate_pitch()` | `FloorMonitor::estimatePitch()` | static and public, so the unit tests can reach it. Reproduces two numpy behaviours - see section 3.5 |
| `FloorMonitor` dataclass | `class FloorMonitor` | the dataclass keyword arguments become `configure()`; construction is `reset()` |
| `Broadcast` | `FloorBroadcast` | every Python field plus `posFloors`, `direction`, `dist24hM`, `modelReady`, which an offline analysis did not need and the transmitter does |
| `update()` | `update()` | identical body, with two guards prepended - section 4.1 and 4.2 |
| `_close_dwell()` | `closeDwell()` | |
| `_arrive()` | `arrive()` | the `forced` argument means the same thing |
| `_confirm()` | `confirm()` | the `_unconfirmed` tuple becomes five members, cleared by a bool |
| `_revisit_rate()` | `revisitRate()` | the `_floor_last` dict becomes three parallel 64-entry arrays plus a seen flag |
| `_trusted_rate()` | `trustedRate()` | including the `+ 1e-18` in the denominator |
| `_replay()` | `replay()` | |
| `_learn()` | `learn()` | |
| `_refine_pitch()` | `refinePitch()` | |
| `_height_of()` | `heightOf()` | reads outside the table fall back to nominal rather than clamping - section 4.4 |
| `_out()` | `out()` | also caches the frame in `last_`, which is what a dropped sample re-serves |
| `datum()` | `datum()` | |
| `n_floors` property | `nFloors()` | |
| `rise` property | `rise()` | |
| `floor_1based()` | `floor1Based()` | |
| `simulate()` | the loop in `sim/replay_main.cpp` | feeds one sample at a time, nothing read ahead |
| `learned_heights()` | `height1Based(int floor1)` | a per-floor accessor rather than an array, since nothing on the node wants the whole ladder at once |
| `heights` dict | `heights_[64]` + `occupied_` bitmap | section 3.4 |
| `stop_counts` dict | `stopCounts_[64]` | section 3.4. Saturates at 0xFFFF instead of growing; it is only read to tell a real floor from a stray index, and 65535 confirmed stops at one landing is not a number this building reaches |
| `_boot`, `_pending` lists | `boot_[12]`, `pend*_[12]` rings | bounded. Both are emptied at the twelfth jump in practice, but a bound is still a bound, and dropping the oldest is what a list-with-a-limit would do |
| - | `save()` / `restore()` / `clearState()` / `FloorModelState` | no Python counterpart. Spec section 5.1 |
| - | `rollOdometer()` / `addDistance()` / `dist24hM()` | no Python counterpart. Spec section 5.2 |
| - | `direction()` | the display arrow. Values match STATE byte bits 1-2 |
| - | `slot()` / `slotConst()` / `pushRateObs()` / `pushParkTail()` / `pushBoot()` | plumbing for the fixed-size containers |
| - | `floor_detail::median` / `stdPopulation` / `roundHalfEven` / `clampD` / `sortSmall` | the numpy behaviours the port has to supply itself - section 3 |

One deliberate difference in `out()`: the Python rounds inside `_out()` because
its immediate consumer is a CSV. The C++ carries the doubles unrounded, and the
replay harness does the rounding when it formats. Rounding on the way out of the
algorithm would be a lossy step in the middle of the port, and the STATE packet
quantises to its own fixed-point fields anyway.

---

## 3. The four ways a port like this diverges silently

Silently is the operative word. None of these produce a compile error, a crash,
or an obviously wrong trace. Each one moves a threshold slightly and shows up
weeks later as a floor count that is occasionally 11.

### 3.1 `double`, never `float`

`trustedRate()` computes

```
w = med^2 / (med^2 + (k*mad)^2 + 1e-18)
```

on weather rates of around 1e-4 m/s. Squaring those gives about 1e-8, and the
line is a 1e-8 over 1e-8 quotient. float32 carries roughly 7 decimal digits;
after the squaring there is nothing left to divide with, and the shrinkage
weight that decides whether a storm correction is applied or discarded comes
back as noise. That weight is the mechanism ALGORITHM.md section 4 describes as
the reason the compensation "can never do harm", so losing it is not a rounding
issue, it is losing the safety property.

Everything on the path is `double`: the sample buffer, the rate ring, the
heights, the parked reference. There is no `float` anywhere in
`floor_monitor.h`. The ESP32-S3 has a single-precision FPU, so the doubles are
software-emulated - which is fine, because the algorithm runs once a second and
the heaviest thing it ever does is the pitch sweep, about 43k double operations,
once.

### 3.2 Population standard deviation

`np.std` defaults to `ddof = 0`, the population form, dividing by *n*. The
sample form divides by *n* − 1, which over a 5-sample window is larger by
`sqrt(5/4)` = 1.118, so about 12%.

`STILL_STD_M` is 0.08 m, placed in the gap between 0.05 m rms of noise inside a
stop and the car's motion. Using the sample std is equivalent to moving that
threshold down to about 0.072 m against the same data - into the noise side of
the gap, where genuine dwells start reading as motion. `stdPopulation()` divides
by `n`.

### 3.3 numpy's even-length median

`np.median` averages the two middle values when the input has even length.
Python's own `statistics.median` does the same; a naive `a[n/2]` after a sort
does not.

This branch is live. The weather-rate ring `_rate_obs` holds between 1 and 7
observations and is even-length roughly half the time while it fills, and that
is precisely the early window - the first usable rate estimate lands at 6.7 min
per ALGORITHM.md section 4 - where the ring is short and one observation matters
most. `floor_detail::median()` implements the averaging branch. The stillness
window is 5 and always odd, but it uses the same helper.

### 3.4 The dicts become fixed tables

`heights` and `stop_counts` are Python dicts keyed by a signed floor index that
starts at 0 and can go negative before `_replay()` re-bases the ladder. There is
no dict on the node and no allocation in this file, so they are 64-entry arrays
at origin +32:

| | |
|---|---|
| index range | −32 to +31, against a building of 10 floors |
| occupancy | `uint64_t occupied_`, bit *k* set means `heights_[k]` holds a value |
| write outside the range | clamps to the end slot and sets `tableClamped_` |
| read outside the range | returns the nominal `index * pitch`, never a neighbour's slot |

The occupancy bitmap is what replaces `key in dict`. Without it, "learned height
0.0" and "never visited" are the same double, and `minIndex()`, `nFloors()` and
`_learn()`'s first-observation branch all depend on telling them apart.

Clamping on write rather than growing is the microcontroller trade: the
alternative to a bound is a crash, and an index 32 floors outside the building
is a runaway model rather than a real landing. `tableClamped` is the flag that
says so, and section 4.3 is about making sure it survives.

### 3.5 A fifth one, found while porting

Not on the original list of four, but worth the same care. Python's `round()`
and `np.round()` both break exact ties to even; C's `lround()` rounds ties away
from zero and `nearbyint()` depends on the runtime rounding mode.
`floor_detail::roundHalfEven()` writes the rule out. A tie in `jump / pitch` is
a measure-zero event on real data, but when it happens it is a single-sample
floor disagreement with no other symptom, which is the most expensive kind of
bug to chase.

`estimatePitch()` also reproduces two `np.arange` properties: it accumulates
`p += step` rather than computing `lo + i*step`, which differs in the last bits
from the third candidate on, and its element count is `ceil((hi - lo) / step)`,
which for 2.2 to 4.0 in 5e-4 steps is 3600 candidates and not the 3601 an
inclusive sweep would give. Neither changes the answer at 0.5 mm resolution.
Both are matched so the comparison stays bit-for-bit rather than
approximately-right.

---

## 4. What the firmware has that the reference does not

The Python is handed a clean, monotonic, gap-free CSV by a test harness. The
firmware is handed a real BMP390 over I²C and a real clock derived from
`millis()`. Everything in this section exists because of that difference, and
none of it changes the algorithm on well-formed input - each guard is a branch
that does not fire.

### 4.1 The non-finite guard

A BMP390 conversion that fails mid-read hands back a non-finite altitude. One of
those is not one bad sample, it is permanent damage:

- if it is the first sample, it goes straight into `ref_`, and `ref_` is never
  re-seeded;
- every clamp, median and running mean downstream propagates NaN untouched;
- NaN compares false against everything, so the stillness test never trips again
  and the state machine stops advancing.

The unit is then dead until someone power-cycles it, with no error visible other
than a floor that stopped changing. So `update()` drops the sample before
anything reads it:

```cpp
if (!isfinite(t) || !isfinite(alt)) {
  sensorErr_ = true;
  sensorRejects_++;
  return last_;
}
```

Returning `last_` rather than a zeroed frame matters: a held floor is what a
display should show while the sensor is out, not a 0. The caller also gets
`sensorError()`, which is bit 4 of the STATE byte and bit 0 of the STATS flags,
so the condition is visible from the corridor rather than inferred.

Dropping at the top is also what keeps NaN out of everything downstream:
`departRef_` only ever copies `ref_`, and the rate ring's observations are
differences of ring medians over a bounded span. `pushRateObs()` carries a
second `isfinite` check anyway, because a single NaN in that ring poisons the
median and the median is fed forward into `ref_` on every sample thereafter.

### 4.2 The non-positive `dt` guard

A caller deriving `t` from `millis()/1000.0` wraps every 49.7 days. The sample
after 4294967.295 s arrives as 0.0, so `dt` is about −4.29e6 s. Both halves of
the parked correction are a rate times `dt`:

```cpp
if (dt > 0.0) {
  ref_ += driftRate_ * dt;
  double step = clampD(level - ref_, -maxDriftMps_ * dt, maxDriftMps_ * dt);
  ref_ += step;
  drift_ += step + driftRate_ * dt;
}
```

Without the guard, the feed-forward term alone moves the reference hundreds of
metres at a typical drift rate, and the follower's cap becomes ±0.02 × dt, which
hands `clampD` its bounds backwards. `test_millis_wrap_does_not_slam_the_reference`
records the measurement with both guards backed out: one sample moves the datum
by 85 km, which takes the floor with it.

Skipping the carry costs exactly one sample period of drift, which the parked
follower picks up on the next sample. `dt == 0` is a duplicate timestamp and the
arithmetic is already a no-op for it.

The second guard in that pair is `floor_detail::clampD()`, which normalises its
bounds if they arrive reversed, so its answer does not depend on which
comparison it happens to test first. With the `dt > 0` check in place the
reversed case is unreachable in the port; it is belt and braces on a path where
the failure is silent and enormous.

### 4.3 `tableClamped` survives the NVS round trip

`tableClamped_` says an index ran off the end of the 64-slot table, which means
the building model has run away and the record is not trustworthy. It is carried
in `FloorModelState` and checked on the way back in:

```cpp
if (s->tableClamped) return false;   // cold boot instead
```

Carrying the flag through persistence is the point. Without it the flag is
cleared by the save/restore cycle while `restored_` is set - and `restored_`
declares the model ready, which is exactly what suppresses the 5-minute
bootstrap that would otherwise rebuild the ladder from scratch. A runaway model
would be laundered clean by a reboot and come back as a confident wrong answer.
That is what `FLOOR_STATE_SCHEMA` was bumped to 2 for, and
`test_table_clamp_survives_the_nvs_round_trip` is the case.

`restore()` also refuses a record from a different schema, and `nvs_model`
refuses one with a bad magic, a short length or a failed CRC. All four lead to
one outcome - the `FloorMonitor` is left untouched and the caller is told it is
a cold boot - because a partially applied model produces confident wrong floors
where a genuine cold boot withholds output.

The 24 h hour buckets are written but deliberately not loaded. They are indexed
by uptime, uptime restarts at 0 on the boot that reads them, and with no RTC
nothing on the node knows how long the unit was off. A bucket filled last
Tuesday would be reported as part of "the last 24 h". The window starts empty
and grows back, which is the claim the display already makes.

### 4.4 Reads fall back to the nominal ladder

`slot()` clamps on the write side. `slotConst()` returns −1 on the read side,
and every read-side caller - `heightOf()`, `heightOr()`, `stopCountAt()` -
handles it:

```cpp
double heightOf(int f) const {
  int k = slotConst(f);
  if (k >= 0 && (occupied_ & (1ULL << k))) return heights_[k];
  return (double)f * pitch_;   // nominal ladder, same as an unlearned landing
}
```

This mirrors the Python exactly - `self.heights.get(f, f * self.pitch)` - and
that is the reason for the asymmetry with the write side. A read that clamped
would hand back floor 31's learned height when asked for floor 40 and say
nothing about it, which is a wrong answer that looks like a right one, and
`datum()` and `posFloors` would go quietly wrong rather than falling back to the
nominal grid. `test_read_paths_do_not_alias_an_out_of_range_index` pins it.

### 4.5 A restore brings back the building, not the position

The Python never restarts, so it never has to decide what to believe about a
saved position. The firmware does, and the answer is: nothing.

`restore()` loads the pitch, the ladder, the stop counts and the odometer. It
does not load `ref_`, which is an absolute altitude the weather has moved since
it was saved; `refFromLive_` takes it from the first sample instead, without the
cold-boot branch that would overwrite the ladder. And it keeps the saved floor
index only as a placeholder, setting `positionUnknown_`.

While unknown, the broadcast reports `modelReady = false`, so the transmitter
sends floor 0 and every display shows `--`. `confirm()` still counts the stop but
skips `bumpStopCount()` and `learn()`, because the index they would write to may
be shifted by whole floors. `anchorStep()`, called on every arrival, tracks the
span of indices seen since boot. When it equals the learned span, the lowest
index seen must be the lowest landing, the shift is applied to `floor_`,
`departFloor_` and the unconfirmed landing, and the index-keyed revisit history is
dropped. A span wider than the building restarts the search.

None of this runs in the replay, which never restores - the floor sequence is
still identical to the Python. `test_restore_withholds_the_floor_until_both_ends_are_visited`
covers it: a four-landing model is saved on the top floor, restored with the car
elsewhere and 5 m of weather added, and the floor stays withheld through a visit
to the top and appears, correct, after the visit to the bottom. Against the old
restore that test fails. `test_cold_boot_is_never_position_unknown` checks a
fresh node is not held back on the same account.

---

## 5. The verification

This is the part that makes the rest of the document true rather than
aspirational. The replay streams the real 3.04 h capture through the **actual
C++ `FloorMonitor`** - `sim/replay_main.cpp` includes `src/floor_monitor.h`
itself, not a host reimplementation of it - and `sim/compare_to_python.py` diffs
the result against `floor_algorithm.py` sample by sample.

A host-side reimplementation would only prove that the reimplementation works.
That is why `floor_monitor.h` depends on nothing but `<stdint.h>`, `<string.h>`
and `<math.h>`: the `native` and `sim` environments compile the exact code the
ESP32 runs.

### 5.1 Commands

From the repo root. `prepare_data.py` and `compare_to_python.py` need `numpy`
and `pandas`; the comparator also adds `elevatormons/tools` to `sys.path` and
imports `floor_algorithm` from there, so that repo has to be checked out beside
this one.

```
cd /Users/nv/Documents/Embedded/simmevator

# 1. Gap-fill the capture onto an even 1 Hz grid -> sim/filled_capture.csv
python3 sim/prepare_data.py

# 2. Build the replay harness -> .pio/build/sim/program
pio run -e sim

# 3. Stream the capture through the real FloorMonitor.
#    argv[1] is the output CSV; argv[2] is the input, defaulting to
#    sim/filled_capture.csv, which is why this is run from the repo root.
.pio/build/sim/program sim/cpp_broadcasts.csv

# 4. Regenerate the golden trace from the Python and diff the C++ against it.
#    Exit status is the verdict. With no argument it only regenerates and
#    checks the golden.
python3 sim/compare_to_python.py sim/cpp_broadcasts.csv

# and the unit tests, which are a separate line of evidence
pio test -e native
```

Step 3 also prints a one-line summary to stderr, so someone running the harness
by hand gets the gate values without going through the comparator.

### 5.2 Results

| quantity | required | measured |
|---|---|---|
| floor sequence vs Python | identical over all 10,950 samples | identical |
| floors served | 10 | 10 |
| learned pitch | 2.871 m ± 1 mm | 2.870 m |
| distance travelled | 3409 m ± 0.1% (±3.4 m) | 3408.3 m |
| floor-to-floor moves | 357 ± 1 | 357 |
| confirmed stops | 255 ± 1 | 255 |

The pitch passes at the edge of its tolerance, not in the middle of it. Both
sides report 2.870 m against a 2.871 ± 0.001 m gate, so it is one least
significant digit from failing. That is not a port problem - the Python golden
reports the same 2.870 - but it is worth knowing before anyone tightens the
tolerance or looks at that row and assumes there is margin behind it.

Two further observations from the current artifacts in `sim/`:

- `cpp_broadcasts.csv` and `golden_broadcasts.csv` are **byte-identical**, not
  merely equal in the floor column. All eleven columns agree, including
  `confidence`, `drift_rate_mps` and `datum`, to the precision each is printed
  at. That is most of what `replay_main.cpp` is: reproducing CPython's `repr()`
  for a float64 and `round(x, n)`'s ties-to-even so that a formatting mismatch
  cannot masquerade as an algorithm divergence.
- The contractual gate is narrower than that. `compare_to_python.py` compares
  the `floor` column sample for sample and checks both sides against the five
  numeric gates. Byte identity is a stronger result that the harness does not
  require, and a future change that alters only `datum`'s last printed digit
  would break the identity without breaking the gate.

**The gate that matters is the first row.** The absolute numbers only bound how
far the gap-filled run may drift from the published measurement; the floor
sequence is what "the port is faithful" means.

### 5.3 The unit tests

`test/test_floor_monitor/` is the other half, and it covers what a single
3 h replay cannot: synthetic lattice ascents and descents, door cycles below
`MIN_JUMP_M`, off-lattice plateau rejection and the `RESYNC_S` override,
bootstrap replay re-basing the index, `trustedRate()` under clustered versus
scattered observations, and one case per hardening item in section 4. Three of
them - `test_round_half_even_matches_python`, `test_median_averages_the_two_middle_values`
and `test_std_is_the_population_form` - exist only to pin section 3's numeric
helpers, because those are the divergences that a replay would hide behind a
still-passing floor sequence until the day it does not.

---

## 6. Why trips and stops are 357/255 and not 356/256

This is the subtlest thing in the project. Someone will find the discrepancy
against ALGORITHM.md section 5 and assume the port is off by one. It is not.

### What the capture actually is

`elevatormons/data/baro-20260910-195011.csv` is a **radio log, not a sensor
log**. It is what a laptop received over LoRa, so it inherits the link's losses:
10,380 samples where 10,950 seconds elapsed, which is 49 dropouts totalling 570
missing samples, 5.21% of the stream. The worst hole is 41 s between two
consecutive real samples, 40 invented grid points wide. Those are the same 49
dropouts and the same 5.21% that ALGORITHM.md section 9 measures as the link's
loss rate.

**The node never sees any of them.** It holds the BMP390 on its own I²C bus.
Radio loss is something that happens after the floor has already been decided -
that is the whole architectural argument in ALGORITHM.md section 9 and spec
section 1.

So replaying the capture as-is would measure the port against a stream the
firmware can never be handed. Worse, it would measure it against a *misleading*
one: the stillness test is a trailing 5-sample window, and a missing second
silently reshapes it, so the gaps do not merely remove information, they change
what the algorithm computes.

### What the fill does

`sim/prepare_data.py` interpolates onto an even 1 Hz grid of 10,950 samples and
marks every invented sample in a `filled` column, so nothing is hidden. Two
details are load-bearing:

- **Interpolation is on pressure, not altitude.** Altitude is a nonlinear
  function of pressure, so interpolating the derived quantity puts the fill
  slightly off the curve the sensor would have traced - small enough to look
  fine, large enough to be wrong. Altitude is recomputed from the interpolated
  pressure with the same barometric formula the firmware uses.
- **The grid is in phase with the real samples.** The receiver's arrival stamps
  land on whole seconds for this capture, so every original sample keeps its own
  value rather than being resampled off it.

### The one-in-each-direction difference

Filling the worst hole - the 41 s one - invents a smooth ride across it. A
smooth ride is one extra floor-to-floor move, and the dwell it replaces is no
longer long enough to confirm. Hence:

| | ALGORITHM.md section 5 | this replay |
|---|---|---|
| input | the raw capture, with its 49 dropouts | the gap-filled 1 Hz grid |
| floor-to-floor moves | 356 | 357 |
| confirmed stops | 256 | 255 |

**The Python reference produces 357 / 255 on this input too.** That is the
sentence that settles it. The two implementations agree exactly; what changed is
the input, and it changed in the direction of what the node would actually have
seen. The comparator checks *both* sides against the gates, so a future
regression surfaces as the C++ disagreeing with the Python, which is a real
failure, rather than as a number that someone quietly moved to make a table
green.

If you are staring at 357 against 356 and wondering which is right: they are
answers to different questions. 356 is how many moves are visible in the radio
log. 357 is how many the car made, as best the fill can reconstruct it. Neither
is evidence about the port. The floor sequence is.

---

## 7. Changing the algorithm safely

In this order, every time. The order is the whole procedure - doing step 5
before step 1 is how a port stops being a port.

1. **Change `floor_algorithm.py` first**, in `elevatormons`. Not here. If the
   change is worth making, it is worth making to the reference, and the
   reference is where the analysis tooling and the test data live.
2. **Re-measure there.** ALGORITHM.md is the record of what the algorithm does:
   the performance table in section 5, the storm and turbulence sweeps, the
   constants in section 7. A change that moves any of those moves the document
   too.
3. **Regenerate the golden trace.** `python3 sim/compare_to_python.py` with no
   arguments rewrites `sim/golden_broadcasts.csv` from the Python and prints the
   gates.
4. **Diff the new golden against the old one in git, and understand every
   changed sample.** Not "the counts still look right" - every changed sample.
   A behaviour change that moves 4 samples is a different thing from one that
   moves 4000, and if you cannot say which one you made, you do not yet know
   what you changed. This is the step that gets skipped; it is also the only one
   that catches a change that is correct on this capture and wrong in general.
5. **Port the change to `floor_monitor.h`,** keeping the function names and the
   body order aligned so the two files still diff by eye. If the change needs a
   new numeric helper, check it against section 3 first: the question is always
   what numpy does, not what looks reasonable.
6. **Run the evidence, in this order:** `pio test -e native`, then the replay,
   then `python3 sim/compare_to_python.py sim/cpp_broadcasts.csv`. The unit
   tests fail faster and point at a specific behaviour; the replay tells you
   whether 3 hours of real building still works.
7. **Accept the new golden and adjust the gates only once steps 4 and 6 both
   pass,** and write down why in the comment above `GATES` in
   `compare_to_python.py`, the way the 357/255 note is written down. A gate
   changed without a reason recorded next to it is indistinguishable from a gate
   changed to make a failure go away.
8. **If `FloorModelState`'s layout changed, bump `FLOOR_STATE_SCHEMA`.** Old
   records are then discarded as a cold boot instead of being reinterpreted
   under the new layout.

The trap in step 5: it is easy to "fix" something in the C++ that the Python
does differently, because the C++ is the code you are currently reading. Resist
it. A C++-only improvement means the firmware no longer has any measured
evidence behind it, and the next person to run the comparator inherits a failure
they did not cause and cannot interpret.

---

## 8. Limits carried forward

Everything in ALGORITHM.md section 8 still applies, unchanged by the port:

- the first five minutes are not trustworthy, so a fresh unit withholds output
  until the pitch is established;
- there is no absolute anchor - the floor index is integrated, so a bad rounding
  stays wrong until the car happens to correct it;
- turbulence headroom is roughly 1.5 to 2× a gusty evening;
- the ladder clamp assumes a near-uniform pitch;
- the constants are tuned to one building, one sensor and one capture.

Read that section rather than trusting this summary of it. Persisting the model
to NVS removes the cold-boot window in practice but changes none of them, and
the new limits this system adds - the flood mesh's lack of a delivery guarantee,
the uncalibrated battery reading, the uptime-relative 24 h window - are in spec
section 9.
